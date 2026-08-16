#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2i + YOLOPv2 lane & stop-line node (ROS2 / Humble)

Converted from the original ROS1/Noetic node.

Key ROS1 -> ROS2 changes:
- rospy  -> rclpy ; the node now subclasses rclpy.node.Node
- rospy.get_param(~x)      -> declare_parameter("x") + get_parameter("x").value
- rospy.Publisher/Subscriber -> create_publisher / create_subscription
- rospy.Timer(Duration)    -> create_timer(period_sec, cb)   (cb takes NO event arg)
- rospy.loginfo/logwarn/.. -> self.get_logger().info/warn/error
- *_throttle(...)          -> logger call with throttle_duration_sec=...
- latched publishers       -> QoS durability = TRANSIENT_LOCAL
- dynamic_reconfigure      -> ROS2 parameters + add_on_set_parameters_callback
                              (tune live with `ros2 param set /yolopv2_zed_node <name> <val>`)
- CameraInfo.K             -> msg.k   (ROS2 message fields are snake_case!)
- header.stamp.to_sec()    -> Time.from_msg(...).nanoseconds * 1e-9
- rospy.is_shutdown()      -> not rclpy.ok()
- rospy.on_shutdown()      -> cleanup called from main()'s finally block
"""
import os, time, threading, csv, struct
from datetime import datetime, timezone
from collections import deque

import numpy as np, cv2, torch

# python-can: 하위 MCU로 steer/rpm 제어 프레임 전송용.
# 미설치여도 can_enable=false 면 노드는 정상 동작(경고만).
try:
    import can
    _HAS_CAN = True
except Exception:
    _HAS_CAN = False

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSHistoryPolicy, QoSReliabilityPolicy)
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image as RosImage, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import message_filters

# === utils (keep your existing package module) ===
from zed_camera.utils.utils import (
    select_device, scale_coords, non_max_suppression, split_for_trace_model,
    lane_line_mask, plot_one_box, AverageMeter, letterbox
)


def time_synchronized() -> float:
    if torch.cuda.is_available():
        try: torch.cuda.synchronize()
        except Exception: pass
    return time.time()


# ---------- BEV/차선 (순수 numpy/cv2 — 변경 없음) ----------
def build_perspective_from_pct(pct_list, w, h):
    px = lambda p, total: float(p) * total * 0.01
    src = np.float32([
        [px(pct_list[0], w), px(pct_list[1], h)],
        [px(pct_list[2], w), px(pct_list[3], h)],
        [px(pct_list[4], w), px(pct_list[5], h)],
        [px(pct_list[6], w), px(pct_list[7], h)],
    ])
    dst = np.float32([
        [w * 0.10, 3],
        [w * 0.90, 3],
        [w * 0.05, h],
        [w * 0.90, h]
    ])
    return cv2.getPerspectiveTransform(src, dst), src


def bird_eye_view_transform(img: np.ndarray, M: np.ndarray):
    if img is None or img.size == 0 or M is None or not (isinstance(M, np.ndarray) and M.shape == (3, 3)):
        return None
    h, w = img.shape[:2]
    try:
        return cv2.warpPerspective(img, M, (w, h))
    except cv2.error:
        return None


def sliding_window_with_boxes_and_angle(binary_warped: np.ndarray,
                                        window_count: int = 10,
                                        margin: int = 50,
                                        min_pixels: int = 50):
    h, w = binary_warped.shape
    win_h = max(1, h // max(1, window_count))
    hist = np.sum(binary_warped[h // 2:, :], axis=0)
    mid = w // 2
    x_left = int(np.argmax(hist[:mid])) if mid > 0 else 0
    x_right = int(np.argmax(hist[mid:]) + mid) if w - mid > 0 else w - 1
    nz_y, nz_x = binary_warped.nonzero()
    boxes = []
    left_inds, right_inds = [], []
    for win in range(window_count):
        y_low, y_high = h - (win + 1) * win_h, h - win * win_h
        xl_low, xl_hi = x_left - margin, x_left + margin
        xr_low, xr_hi = x_right - margin, x_right + margin
        boxes.extend([((xl_low, y_low), (xl_hi, y_high)),
                      ((xr_low, y_low), (xr_hi, y_high))])
        good_l = ((nz_y >= y_low) & (nz_y < y_high) &
                  (nz_x >= xl_low) & (nz_x < xl_hi)).nonzero()[0]
        good_r = ((nz_y >= y_low) & (nz_y < y_high) &
                  (nz_x >= xr_low) & (nz_x < xr_hi)).nonzero()[0]
        left_inds.append(good_l); right_inds.append(good_r)
        if len(good_l) > min_pixels: x_left = int(np.mean(nz_x[good_l]))
        if len(good_r) > min_pixels: x_right = int(np.mean(nz_x[good_r]))
    left_inds  = np.concatenate(left_inds)  if left_inds  else np.array([])
    right_inds = np.concatenate(right_inds) if right_inds else np.array([])
    left_fit, right_fit = np.array([0, 0, 0]), np.array([0, 0, 0])
    if left_inds.size and right_inds.size:
        left_fit  = np.polyfit(nz_y[left_inds],  nz_x[left_inds],  2)
        right_fit = np.polyfit(nz_y[right_inds], nz_x[right_inds], 2)
    lane_center = (np.polyval(left_fit, h) + np.polyval(right_fit, h)) / 2
    offset      = lane_center - (w / 2)
    angle_deg   = np.arctan(offset / max(1e-6, h)) * 180 / np.pi
    return angle_deg, boxes, left_fit, right_fit


def detect_stop_line(binary_warped: np.ndarray,
                     horizontal_line_y_offset: int = 50,
                     segment_count: int = 10,
                     threshold: int = 5,
                     min_segments: int = 5) -> bool:
    h, w = binary_warped.shape
    y = max(0, min(h - 1, h - horizontal_line_y_offset))
    row = binary_warped[int(y), :]
    seg_w = max(1, w // max(1, segment_count))
    hits = sum(np.sum(row[i * seg_w:(i + 1) * seg_w] > 0) >= threshold
               for i in range(segment_count))
    return hits >= min_segments


# ---------- 모델 초기화 (CPU/CUDA 안전) ----------
def initialize_model(weights: str, device: torch.device, half: bool):
    map_loc = 'cpu' if device.type == 'cpu' else None
    model = torch.jit.load(weights, map_location=map_loc)
    model = model.to(device).eval()
    if half and device.type != 'cpu':
        model.half()
    if device.type != 'cpu':
        torch.backends.cudnn.benchmark = True
    return model


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


# ---------- CAN 브리지 (pcan_jetson_live.py 프레임 규격과 동일) ----------
class CanBridge:
    """카메라 노드가 계산한 steer/rpm 을 하위 MCU로 내려보내고,
    (옵션) 구동/조향 피드백을 읽는다.

    프레임 규격 (pcan_jetson_live.py 와 반드시 동일하게 유지):
      TX 0x200 (제어):  <hhBBH> = rpm, steer, enable, stop_mode, reserved(0)
      RX 0x102 (구동):  <hhhh>  = enc, rpm*10, pwm_duty, target_rpm
      RX 0x101 (조향):  <HHhh>  = cur_pot, tgt_pot, cur_angle*10, tgt_angle*10
    """
    DRIVE_STATUS_ID    = 0x102
    STEERING_STATUS_ID = 0x101

    def __init__(self, channel="can0", bitrate=500000, tx_id=0x200,
                 setup_interface=False, logger=None):
        if not _HAS_CAN:
            raise RuntimeError("python-can 미설치 (pip install python-can)")
        self.tx_id  = int(tx_id)
        self.logger = logger
        if setup_interface:
            self._setup_interface(channel, bitrate)
        # socketcan bus 연결
        self.bus = can.interface.Bus(interface="socketcan", channel=channel)
        self.drive_status    = None
        self.steering_status = None

    @staticmethod
    def _setup_interface(channel, bitrate):
        # 원본 pcan_jetson_live.py 의 setup_can_interface 와 동일 동작.
        subprocess_run = __import__("subprocess").run
        subprocess_run(["sudo", "ip", "link", "set", channel, "down"], check=False)
        result = subprocess_run(
            ["sudo", "ip", "link", "set", channel, "up",
             "type", "can", "bitrate", str(int(bitrate))],
            check=False)
        if result.returncode != 0:
            raise RuntimeError(
                f"'{channel}' 인터페이스를 올리지 못했습니다. "
                "어댑터 연결/‌sudo 권한을 확인하세요.")

    def send_control(self, rpm, steer, enable, stop_mode):
        data = struct.pack("<hhBBH",
                           int(rpm), int(steer),
                           int(enable), int(stop_mode), 0)
        msg = can.Message(arbitration_id=self.tx_id, data=data,
                          is_extended_id=False)
        self.bus.send(msg)

    def poll_feedback(self):
        """버퍼에 쌓인 RX 프레임을 모두 소진하며 최신 상태 갱신 (non-blocking)."""
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if msg.arbitration_id == self.DRIVE_STATUS_ID and len(msg.data) == 8:
                enc, rpm_x10, pwm, trpm = struct.unpack("<hhhh", msg.data)
                self.drive_status = {"encoder_count": enc, "rpm": rpm_x10 / 10.0,
                                     "pwm_duty": pwm, "target_rpm": trpm}
            elif msg.arbitration_id == self.STEERING_STATUS_ID and len(msg.data) == 8:
                cp, tp, ca, ta = struct.unpack("<HHhh", msg.data)
                self.steering_status = {"current_pot": cp, "target_pot": tp,
                                        "current_angle": ca / 10.0,
                                        "target_angle": ta / 10.0}

    def shutdown(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


class YoloPv2ZedNode(Node):
    def __init__(self):
        super().__init__("yolopv2_zed_node")
        self.bridge = CvBridge()

        # ------- 파라미터 선언 & 읽기 (ROS1 ~private param 대응) -------
        self.weights   = self._param("weights",   "")
        self.img_size  = int(self._param("img_size", 640))
        self.conf_th   = float(self._param("conf_thres", 0.3))
        self.iou_th    = float(self._param("iou_thres", 0.45))
        self.agnostic  = bool(self._param("agnostic_nms", False))
        self.save_txt  = bool(self._param("save_txt", False))
        self.save_conf = bool(self._param("save_conf", False))

        # 기본 토픽 (ROS2 zed wrapper 이름으로 갱신, launch로 덮어쓰기 가능)
        self.rgb_topic   = self._param("rgb_topic",   "/zed/zed_node/rgb/color/rect/image")
        self.depth_topic = self._param("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.cinfo_topic = self._param("cinfo_topic", "/zed/zed_node/rgb/color/rect/camera_info")
        self.odom_topic  = self._param("odom_topic",  "/zed/zed_node/odom")

        # ------- GUI/디버그 창 -------
        self.show_windows = bool(self._param("show_windows", True))
        if self.show_windows:
            try:
                if not os.environ.get("DISPLAY", ""):
                    raise RuntimeError("DISPLAY not set")
                cv2.namedWindow("Original + HUD", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Bird-eye view", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Stop Line", cv2.WINDOW_NORMAL)
            except Exception as e:
                self.get_logger().warn(f"[gui] disabling windows ({e}); still publishing debug images")
                self.show_windows = False

        # ------- 디버그 이미지 퍼블리셔 (ROS1 latch -> TRANSIENT_LOCAL) -------
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_img_hud      = self.create_publisher(RosImage, "~/dbg/hud",      latched_qos)
        self.pub_img_bev      = self.create_publisher(RosImage, "~/dbg/bev",      latched_qos)
        self.pub_img_stop     = self.create_publisher(RosImage, "~/dbg/stop",     latched_qos)
        self.pub_img_lanemask = self.create_publisher(RosImage, "~/dbg/lanemask", latched_qos)
        self.pub_img_bevmask  = self.create_publisher(RosImage, "~/dbg/bev_mask", latched_qos)

        # ------- 디바이스 정규화 + 폴백 -------
        raw_dev = str(self._param("device", "0")).lower()
        if raw_dev.startswith("cuda:"):
            raw_dev = raw_dev.split(":", 1)[1]  # 'cuda:0' -> '0'
        self.device_id = raw_dev
        try:
            self.dev = select_device(self.device_id)
        except AssertionError as e:
            self.get_logger().warn(f"[Device] {e} -> falling back to CPU")
            self.dev = select_device("cpu")

        self.get_logger().info(
            f"[Device] using='{self.device_id}', torch_cuda={torch.cuda.is_available()}, "
            f"cuda_count={torch.cuda.device_count()}, torch_ver={torch.__version__}")
        self.get_logger().info(
            f"[Topics] rgb={self.rgb_topic}, depth={self.depth_topic}, "
            f"cinfo={self.cinfo_topic}, odom={self.odom_topic}")
        self.get_logger().info("[Debug topics] ~/dbg/hud, ~/dbg/bev, ~/dbg/stop, ~/dbg/lanemask, ~/dbg/bev_mask")

        assert os.path.isfile(self.weights), f"weights not found: {self.weights}"
        self.half = (self.dev.type != 'cpu')
        self.model = initialize_model(self.weights, self.dev, self.half)
        if torch.cuda.is_available(): torch.cuda.empty_cache()

        # ------- state -------
        self.t_inf = AverageMeter(); self.t_nms = AverageMeter(); self.t_split = AverageMeter()
        self.persp_M = None; self.frame_wh = None; self.K = None
        self.quit = False

        # 조향/속도/모터RPM 퍼블리셔 + 50ms 주기 퍼블리시
        self.pub_steer = self.create_publisher(Float32, "~/steering_deg", 10)
        self.pub_speed = self.create_publisher(Float32, "~/speed_mps",    10)
        self.pub_rpm   = self.create_publisher(Float32, "~/motor_rpm",    10)
        self.speed_mps = float('nan')     # Odometry로 갱신
        self.steering_deg = float('nan')  # 추론으로 갱신

        # --- 모터 RPM 계산용 파라미터 (ros2 param set 으로 런타임 변경 가능) ---
        #   motor_rpm = speed_mps * 60 * gear_ratio / (2*pi*wheel_radius)
        # wheel_radius: 바퀴 반지름[m], gear_ratio: 모터:바퀴 회전비(예: 20:1 -> 20.0)
        self.wheel_radius = float(self._param("wheel_radius", 0.05))   # 기본 5cm
        self.gear_ratio   = float(self._param("gear_ratio",   1.0))    # 기본 1:1
        self._warned_no_odom = False
        self.create_timer(0.05, self._publish_timer_cb)  # 50ms

        # =========================================================
        # ------- CAN (하위 MCU로 steer/rpm 내려보내기) -------
        # =========================================================
        # 마스터 스위치. false 면 CAN 없이 비전만 동작.
        self.can_enable = bool(self._param("can_enable", True))
        self.can_channel = str(self._param("can_channel", "can0"))
        self.can_bitrate = int(self._param("can_bitrate", 500000))
        # tx_id 는 int 로 (예: 512 == 0x200). 기본 0x200.
        self.can_tx_id   = int(self._param("can_tx_id", 0x200))
        # 노드가 직접 sudo ip link 로 인터페이스 설정 (사용자 선택)
        self.can_setup_interface = bool(self._param("can_setup_interface", True))
        # CAN 초기화 실패 시 노드를 죽일지(True) / 경고만 하고 계속(False)
        self.can_required = bool(self._param("can_required", False))
        # 피드백(0x101/0x102) 읽기 여부
        self.can_read_feedback = bool(self._param("can_read_feedback", True))

        # --- 제어 명령 관련 파라미터 (런타임 ros2 param set 가능) ---
        # enable: 1=구동 허용 (사용자 선택: 기본 1). 단 target_rpm=0 이면 실제로 안 움직임.
        self.motor_enable = int(self._param("motor_enable", 1))
        # stop_mode: 0=normal, 1=flat stop, 2=hill stop
        self.stop_mode    = int(self._param("stop_mode", 0))
        # rpm 명령: 고정 목표 rpm 사용 (사용자 선택). 0 이면 정지 상태로 대기.
        self.can_target_rpm = int(self._param("can_target_rpm", 0))

        # 조향 변환: vision steering_deg -> MCU steer 명령(도 단위, -45~45).
        #   steer_cmd = steering_deg * steer_sign * steer_gain
        # 조향 방향이 반대면 steer_sign 을 -1 로.
        self.steer_sign = int(self._param("steer_sign", 1))
        self.steer_gain = float(self._param("steer_gain", 1.0))

        # 안전 클램프 (pcan_jetson_live.py 와 동일 범위)
        self.rpm_min   = int(self._param("rpm_min",   -300))
        self.rpm_max   = int(self._param("rpm_max",    300))
        self.steer_min = int(self._param("steer_min", -45))
        self.steer_max = int(self._param("steer_max",  45))

        self.can = None
        if self.can_enable:
            if not _HAS_CAN:
                msg = "[can] python-can 미설치 -> CAN 비활성 (pip install python-can)"
                if self.can_required:
                    raise RuntimeError(msg)
                self.get_logger().error(msg)
            else:
                try:
                    self.can = CanBridge(
                        channel=self.can_channel,
                        bitrate=self.can_bitrate,
                        tx_id=self.can_tx_id,
                        setup_interface=self.can_setup_interface,
                        logger=self.get_logger(),
                    )
                    self.get_logger().info(
                        f"[can] up: ch={self.can_channel}@{self.can_bitrate} "
                        f"tx_id=0x{self.can_tx_id:03X} enable={self.motor_enable} "
                        f"target_rpm={self.can_target_rpm}")
                except Exception as e:
                    if self.can_required:
                        raise
                    self.get_logger().error(f"[can] init 실패 -> CAN 비활성: {e}")
                    self.can = None
        else:
            self.get_logger().info("[can] can_enable=false -> CAN 비활성")

        # ------- CSV logging -------
        # NOTE: 원본은 /home/root1/... 하드코딩. 파라미터로 뺐고 기본값은 홈 아래로 변경.
        default_csv_dir = os.path.expanduser("~/ros2_ws/src/zed_camera/csv")
        csv_dir = str(self._param("csv_dir", default_csv_dir))
        os.makedirs(csv_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(csv_dir, f"lane_log_{ts}.csv")
        self._csv_flush_every = int(self._param("csv_flush_every", 20))
        self._csv_count = 0
        self.log_lock = threading.Lock()
        self.csv_fh, self.csv_writer = None, None
        try:
            self.csv_fh = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow([
                "timestamp",
                "left_a", "left_b", "left_c",
                "right_a", "right_b", "right_c",
                "speed_mps", "motor_rpm", "steering_deg"
            ])
            self.csv_fh.flush()
            self.get_logger().info(f"[csv] logging to {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"[csv] failed to open log file: {e}")
            self.csv_fh, self.csv_writer = None, None

        # ------- 튜닝 파라미터 (구 dynamic_reconfigure LaneTunerConfig 대체) -------
        # 기본값
        self.src_pct = [40.0, 80.0, 60.0, 80.0, 25.0, 100.0, 70.0, 100.0] # x, y 노란점 좌표 기본값
        self.sw_count, self.sw_margin, self.sw_minpix = 10, 50, 50
        self.stop_y_offset, self.stop_segments, self.stop_threshold, self.stop_min_hits = 50, 10, 5, 5

        # 파라미터로 선언(런타임 ros2 param set 가능)
        self.src_pct = [
            float(self._param("src_p1_x", self.src_pct[0])), float(self._param("src_p1_y", self.src_pct[1])),
            float(self._param("src_p2_x", self.src_pct[2])), float(self._param("src_p2_y", self.src_pct[3])),
            float(self._param("src_p3_x", self.src_pct[4])), float(self._param("src_p3_y", self.src_pct[5])),
            float(self._param("src_p4_x", self.src_pct[6])), float(self._param("src_p4_y", self.src_pct[7])),
        ]
        self.sw_count  = int(self._param("sw_count",  self.sw_count))
        self.sw_margin = int(self._param("sw_margin", self.sw_margin))
        self.sw_minpix = int(self._param("sw_minpix", self.sw_minpix))
        self.stop_y_offset  = int(self._param("stop_y_offset",  self.stop_y_offset))
        self.stop_segments  = int(self._param("stop_segments",  self.stop_segments))
        self.stop_threshold = int(self._param("stop_threshold", self.stop_threshold))
        self.stop_min_hits  = int(self._param("stop_min_hits",  self.stop_min_hits))
        self.src_pts = []

        # 런타임 파라미터 변경 콜백 (dynamic_reconfigure 대체)
        self.add_on_set_parameters_callback(self._on_param_change)

        # ------- queue/worker -------
        self.q = deque(maxlen=2)
        self.lock = threading.Lock()
        self.worker = threading.Thread(target=self._infer_loop, daemon=True)
        self.worker.start()

        # ------- 동기화 구독 -------
        # ZED wrapper의 image 토픽 QoS에 맞춰야 함. 기본(RELIABLE)로 안 들어오면
        # sensor_qos(BEST_EFFORT)로 바꿔보세요 (아래 주석 참고).
        sensor_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,   # 필요시 BEST_EFFORT
        )
        sub_rgb = message_filters.Subscriber(self, RosImage, self.rgb_topic,   qos_profile=sensor_qos)
        sub_dep = message_filters.Subscriber(self, RosImage, self.depth_topic, qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_dep], queue_size=30, slop=0.30)
        self.sync.registerCallback(self._on_sync)

        self.create_subscription(CameraInfo, self.cinfo_topic, self._on_caminfo, 1)
        self.create_subscription(Odometry,   self.odom_topic,  self._on_odom,    5)

        self.get_logger().info("yolopv2_zed_node ready.")

    # ---------- 파라미터 헬퍼 ----------
    def _param(self, name, default):
        """declare + get. 이미 선언됐으면 값만 반환."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_param_change(self, params):
        """dynamic_reconfigure 대체: ros2 param set 시 호출."""
        for p in params:
            n, v = p.name, p.value
            if   n == "src_p1_x": self.src_pct[0] = float(v)
            elif n == "src_p1_y": self.src_pct[1] = float(v)
            elif n == "src_p2_x": self.src_pct[2] = float(v)
            elif n == "src_p2_y": self.src_pct[3] = float(v)
            elif n == "src_p3_x": self.src_pct[4] = float(v)
            elif n == "src_p3_y": self.src_pct[5] = float(v)
            elif n == "src_p4_x": self.src_pct[6] = float(v)
            elif n == "src_p4_y": self.src_pct[7] = float(v)
            elif n == "sw_count":       self.sw_count = int(v)
            elif n == "sw_margin":      self.sw_margin = int(v)
            elif n == "sw_minpix":      self.sw_minpix = int(v)
            elif n == "stop_y_offset":  self.stop_y_offset = int(v)
            elif n == "stop_segments":  self.stop_segments = int(v)
            elif n == "stop_threshold": self.stop_threshold = int(v)
            elif n == "stop_min_hits":  self.stop_min_hits = int(v)
            elif n == "wheel_radius":   self.wheel_radius = float(v)
            elif n == "gear_ratio":     self.gear_ratio = float(v)
            # --- CAN 제어 (런타임 튜닝: ros2 param set) ---
            elif n == "motor_enable":   self.motor_enable = int(v)
            elif n == "stop_mode":      self.stop_mode = int(v)
            elif n == "can_target_rpm": self.can_target_rpm = int(v)
            elif n == "steer_sign":     self.steer_sign = int(v)
            elif n == "steer_gain":     self.steer_gain = float(v)
            elif n == "rpm_min":        self.rpm_min = int(v)
            elif n == "rpm_max":        self.rpm_max = int(v)
            elif n == "steer_min":      self.steer_min = int(v)
            elif n == "steer_max":      self.steer_max = int(v)

        if self.frame_wh is not None:
            w0, h0 = self.frame_wh
            self.persp_M, self.src_pts = build_perspective_from_pct(self.src_pct, w0, h0)
        self.get_logger().info(
            f"[reconf] src%={self.src_pct}, sw=({self.sw_count},{self.sw_margin},{self.sw_minpix}), "
            f"stop=({self.stop_y_offset},{self.stop_segments},{self.stop_threshold},{self.stop_min_hits})",
            throttle_duration_sec=1.0)
        return SetParametersResult(successful=True)

    # ---------- ROS 콜백 ----------
    def _on_caminfo(self, msg: CameraInfo):
        # ROS2: CameraInfo.K -> msg.k (snake_case)
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    def _on_odom(self, msg: Odometry):
        self.speed_mps = float(msg.twist.twist.linear.x)

    def _mps_to_motor_rpm(self, speed_mps):
        """선속도[m/s] -> 모터 RPM (바퀴 RPM x 기어비)."""
        if speed_mps is None or np.isnan(speed_mps):
            return float('nan')
        circ = 2.0 * np.pi * self.wheel_radius  # 바퀴 둘레[m]
        if circ <= 1e-9:
            return float('nan')
        wheel_rpm = (float(speed_mps) / circ) * 60.0
        return wheel_rpm * self.gear_ratio

    def _on_sync(self, rgb_msg: RosImage, depth_msg: RosImage):
        if self.quit: return
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        depth = np.array(depth, dtype=np.float32, copy=True)
        depth = np.nan_to_num(depth, copy=False, nan=0.0, posinf=0.0, neginf=0.0)

        if self.frame_wh is None:
            h0, w0 = rgb.shape[:2]
            self.frame_wh = (w0, h0)

        self.persp_M, self.src_pts = build_perspective_from_pct(self.src_pct, *self.frame_wh)

        img = letterbox(rgb, self.img_size, stride=32)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img = np.ascontiguousarray(img)

        # 타임스탬프: ROS2는 to_sec()가 없음 -> Time.from_msg 사용
        try:
            tstamp = Time.from_msg(rgb_msg.header.stamp).nanoseconds * 1e-9
        except Exception:
            tstamp = time.time()

        with self.lock:
            if not self.quit:
                self.q.append((rgb, depth, img, tstamp))

    def _log_row(self, tstamp, left_fit, right_fit, speed_mps, steering_deg):
        if self.csv_writer is None:
            return
        try:
            ts_iso = datetime.fromtimestamp(float(tstamp), tz=timezone.utc).isoformat()
        except Exception:
            ts_iso = ""

        def coeffs(f):
            try:
                if f is None: return (float('nan'),) * 3
                f = np.array(f).reshape(-1)
                if f.size < 3: return (float('nan'),) * 3
                if np.allclose(f[:3], 0.0): return (float('nan'),) * 3
                return (float(f[0]), float(f[1]), float(f[2]))
            except Exception:
                return (float('nan'),) * 3

        la, lb, lc = coeffs(left_fit)
        ra, rb, rc = coeffs(right_fit)
        sp = 0.0 if (speed_mps is None or np.isnan(speed_mps)) else float(speed_mps)
        rpm = self._mps_to_motor_rpm(speed_mps)
        rpm = 0.0 if np.isnan(rpm) else float(rpm)
        st = float(steering_deg) if (steering_deg is not None and not np.isnan(steering_deg)) else float('nan')
        with self.log_lock:
            try:
                self.csv_writer.writerow([ts_iso, la, lb, lc, ra, rb, rc, sp, rpm, st])
                self._csv_count += 1
                if (self._csv_count % self._csv_flush_every) == 0:
                    self.csv_fh.flush()
            except Exception as e:
                self.get_logger().warn(f"[csv] write failed: {e}", throttle_duration_sec=5.0)

    def _infer_loop(self):
        while rclpy.ok() and not self.quit:
            item = None
            with self.lock:
                if self.q: item = self.q.popleft()
            if item is None:
                time.sleep(0.002); continue
            rgb, depth, img_np, tstamp = item

            img_t = torch.from_numpy(img_np).to(self.dev)
            img_t = img_t.half() if self.half else img_t.float()
            img_t /= 255.0
            if img_t.ndim == 3: img_t = img_t.unsqueeze(0)

            # 추론
            t1 = time_synchronized()
            with torch.no_grad():
                (pred_raw, anchor_grid), seg, ll = self.model(img_t)
            t2 = time_synchronized(); self.t_inf.update(t2 - t1)

            ts1 = time_synchronized()
            pred = split_for_trace_model(pred_raw, anchor_grid)
            ts2 = time_synchronized(); self.t_split.update(ts2 - ts1)

            t3 = time_synchronized()
            pred = non_max_suppression(pred, self.conf_th, self.iou_th,
                                       classes=None, agnostic=self.agnostic)
            t4 = time_synchronized(); self.t_nms.update(t4 - t3)

            # 차선 마스크
            ll_mask = cv2.resize(lane_line_mask(ll), rgb.shape[1::-1],
                                 interpolation=cv2.INTER_NEAREST).astype(np.uint8) * 255

            sp = self.speed_mps if not np.isnan(self.speed_mps) else 0.0

            vis = rgb.copy()
            bev = None
            stop_flag = False

            if np.any(ll_mask):
                bev = bird_eye_view_transform(ll_mask, self.persp_M)
                if bev is not None:
                    steering, boxes, left_fit, right_fit = sliding_window_with_boxes_and_angle(
                        bev, window_count=self.sw_count, margin=self.sw_margin, min_pixels=self.sw_minpix
                    )
                    self.steering_deg = float(steering)
                    stop_flag = detect_stop_line(
                        bev, horizontal_line_y_offset=self.stop_y_offset,
                        segment_count=self.stop_segments, threshold=self.stop_threshold,
                        min_segments=self.stop_min_hits
                    )
                    self._log_row(tstamp, left_fit, right_fit, sp, self.steering_deg)
                    cv2.putText(vis, f"Steering: {self.steering_deg:6.2f} deg", (30, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    self.steering_deg = float('nan')
                    self._log_row(tstamp, None, None, sp, self.steering_deg)
            else:
                self.steering_deg = float('nan')
                self._log_row(tstamp, None, None, sp, self.steering_deg)

            cv2.putText(vis, f"Speed: {sp:5.2f} m/s", (30, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            rpm_disp = self._mps_to_motor_rpm(sp)
            rpm_disp = 0.0 if np.isnan(rpm_disp) else rpm_disp
            cv2.putText(vis, f"Motor: {rpm_disp:6.1f} rpm", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            for x, y in getattr(self, "src_pts", []):
                cv2.circle(vis, (int(x), int(y)), 5, (0, 255, 255), -1)

            # --- object detection 박스 표시 비활성화 (필요시 주석 해제) ---
            # if len(pred) and pred[0] is not None and len(pred[0]):
            #     d = pred[0]
            #     d[:, :4] = scale_coords(img_t.shape[2:], d[:, :4], vis.shape).round()
            #     for *xyxy, conf, cls in reversed(d):
            #         plot_one_box(xyxy, vis, line_thickness=3)

            lanemask_bgr = cv2.cvtColor(ll_mask, cv2.COLOR_GRAY2BGR)
            if bev is None:
                bev = ll_mask.copy()
            bev_u8 = bev if bev.dtype == np.uint8 else np.clip(bev, 0, 255).astype(np.uint8)
            bev_vis = cv2.cvtColor(bev_u8, cv2.COLOR_GRAY2BGR)

            stop_img = bev_vis.copy()
            y_line = stop_img.shape[0] - int(self.stop_y_offset)
            y_line = max(0, min(stop_img.shape[0] - 1, y_line))
            cv2.line(stop_img, (0, y_line), (stop_img.shape[1], y_line), (0, 0, 255), 2)
            txt = 'Stop Line Detected' if stop_flag else 'No Stop Line'
            cv2.putText(stop_img, txt, (50, max(0, y_line - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 0, 255) if stop_flag else (200, 200, 200), 2)

            self._show(vis, bev_vis, stop_img, lanemask_bgr, bev_u8, self._fps_est())

    # 50ms 퍼블리시 타이머 콜백 (ROS2: 인자 없음)
    def _publish_timer_cb(self):
        steer = self.steering_deg if not np.isnan(self.steering_deg) else 0.0
        try:
            self.pub_steer.publish(Float32(data=float(steer)))
        except Exception:
            pass

        if np.isnan(self.speed_mps):
            if not self._warned_no_odom:
                self.get_logger().warn(
                    f"[odom] no data from {self.odom_topic}; publishing 0.0 m/s",
                    throttle_duration_sec=5.0)
                self._warned_no_odom = True
            sp = 0.0
        else:
            sp = float(self.speed_mps)
        try:
            self.pub_speed.publish(Float32(data=sp))
        except Exception:
            pass

        # 모터 RPM 퍼블리시 (odom 없으면 0.0)
        rpm = self._mps_to_motor_rpm(self.speed_mps)
        rpm = 0.0 if np.isnan(rpm) else float(rpm)
        try:
            self.pub_rpm.publish(Float32(data=rpm))
        except Exception:
            pass

        # ------- CAN 으로 steer/rpm 하위 MCU에 내려보내기 -------
        if self.can is not None:
            # 조향 명령: vision steering_deg -> 부호/게인 -> 정수 -> 클램프(-45~45)
            steer_cmd = int(round(steer * self.steer_sign * self.steer_gain))
            steer_cmd = clamp(steer_cmd, self.steer_min, self.steer_max)

            # rpm 명령: 사용자 선택 = 고정 목표 rpm (odom 역산값이 아님)
            rpm_cmd = clamp(int(self.can_target_rpm), self.rpm_min, self.rpm_max)

            try:
                # stop_mode 는 항상 0(normal)만 송신
                self.can.send_control(rpm_cmd, steer_cmd,
                                      self.motor_enable, 0)
                if self.can_read_feedback:
                    self.can.poll_feedback()
            except Exception as e:
                self.get_logger().warn(f"[can] send 실패: {e}",
                                       throttle_duration_sec=2.0)

    def _fps_est(self):
        total = self.t_inf.avg + self.t_split.avg + self.t_nms.avg + 1e-6
        return 1.0 / total

    def _show(self, hud_bgr, bev_bgr, stop_bgr, lanemask_bgr, bev_mask_u8, fps):
        try:
            hud = hud_bgr.copy()
            cv2.putText(hud, f"FPS: {fps:.1f}", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            self.pub_img_hud.publish(self.bridge.cv2_to_imgmsg(hud, "bgr8"))
            self.pub_img_bev.publish(self.bridge.cv2_to_imgmsg(bev_bgr, "bgr8"))
            self.pub_img_stop.publish(self.bridge.cv2_to_imgmsg(stop_bgr, "bgr8"))
            self.pub_img_lanemask.publish(self.bridge.cv2_to_imgmsg(lanemask_bgr, "bgr8"))
            self.pub_img_bevmask.publish(self.bridge.cv2_to_imgmsg(bev_mask_u8, encoding="mono8"))
        except Exception as e:
            self.get_logger().warn(f"[pub] image publish failed: {e}", throttle_duration_sec=2.0)

        if self.show_windows:
            try:
                cv2.imshow("Original + HUD", hud)
                cv2.imshow("Bird-eye view", bev_bgr)
                cv2.imshow("Stop Line", stop_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.quit = True
                    try: cv2.destroyAllWindows()
                    except Exception: pass
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().warn(f"[gui] imshow failed: {e}", throttle_duration_sec=2.0)

    def on_shutdown(self):
        self.quit = True

        # CAN 안전 정지: rpm=0, enable=0, stop_mode=0 로 내려보내고 버스 종료
        if getattr(self, "can", None) is not None:
            try:
                self.can.send_control(0, 0, 0, 0)
            except Exception:
                pass
            try:
                self.can.shutdown()
                self.get_logger().info("[can] safe-stop 전송 후 종료")
            except Exception:
                pass

        try: cv2.destroyAllWindows()
        except Exception: pass
        try:
            if getattr(self, "csv_fh", None):
                self.csv_fh.flush()
                self.csv_fh.close()
                self.get_logger().info(f"[csv] saved to {self.csv_path}")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloPv2ZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
