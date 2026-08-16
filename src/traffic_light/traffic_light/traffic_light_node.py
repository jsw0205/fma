#!/usr/bin/env python3
"""OAK-D + YOLO traffic light detector, wrapped as a ROS2 node.

2026-08-06: swapped in the newer standalone script from
~/Downloads/traffic_light.zip (best.pt model, green-suppression logic,
manual exposure/white-balance) in place of the original
~/Downloads/test__sunny/test_sunny.py port - same wrapping approach as
before (module-level script -> proper Node class), not a straight drop-in:
the zip's own traffic_light_node.py has no main() at all despite its
setup.py's entry_points pointing at one (only "works" because Python runs
all module-level code, including the `while True` camera loop, as a side
effect of the entry-point's `import` - the loop just never returns control
to reach the nonexistent main() until 'q' is pressed). Kept every actual
behavior identical (thresholds, green-suppression, exposure/WB values) and
folded it into the existing resilient node structure instead of the
scripty version.

Publishes GO/STOP on /traffic_light (std_msgs/String) for control_arbiter's
"traffic_light" event zones to consume.

Red-light debounce logic (unchanged from either script): last
BUFFER_SIZE frames, STOP if at least STOP_THRESHOLD of them saw a red
light above CONF_THRESHOLD confidence. New in the swapped-in version:
if a green light is *also* seen above threshold in the same frame, that
frame doesn't count as red at all (green_beats_red param, default True -
the zip's own comment flags this as something to reconsider before a
real competition run, so it's a toggle here rather than hardcoded).

Manual exposure/white-balance (new): the zip sets a short exposure
(1000us) + fixed ISO/white-balance so a bright red/green LED doesn't
overexpose to white and lose its detectable color - toggle with
use_manual_exposure (default True per the zip), tune with
manual_exposure_us/manual_iso/manual_wb_k.

depthai/ultralytics are optional at import time and the OAK-D is optional
at runtime - if either is missing/not connected, the node starts and logs
a warning instead of crashing, so `colcon build`/launch integration work
even before the camera is actually plugged in.
"""
import csv
import os
import threading
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import depthai as dai
except ImportError:
    dai = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light_node")

        self.declare_parameter(
            "model_path",
            "/home/a/ros2_ws/src/traffic_light/weights/best.pt",
        )
        self.declare_parameter("conf_threshold", 0.8)
        self.declare_parameter("buffer_size", 30)
        self.declare_parameter("stop_threshold", 25)
        # STOP<->GO 히스테리시스: 한 번 STOP 되면 red_count가 이 값 밑으로
        # 확실히 떨어질 때만 GO로 복귀 - 프레임마다 검출이 100% 일관되진
        # 않아서(거리/각도로 가끔 놓침) 단일 문턱만 쓰면 STOP 직후에도
        # GO가 깜빡깜빡 섞여 나옴.
        self.declare_parameter("stop_exit_threshold", 15)
        self.declare_parameter("device", "0")
        self.declare_parameter("max_fps", 30.0)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("log_csv", True)
        self.declare_parameter(
            "csv_path",
            os.path.expanduser(
                "~/logs/traffic_light/traffic_light_log_"
                + time.strftime("%Y%m%d_%H%M%S") + ".csv"
            ),
        )
        # 대회 가서는 필요 없을 수 있음 - 카메라 프레임을 사람이 직접 보고
        # 판단할 것 (원본 스크립트 주석 그대로). 짧은 노출로 LED 색이
        # 하얗게 날아가는(오버노출) 걸 막는 목적.
        self.declare_parameter("use_manual_exposure", True)
        self.declare_parameter("manual_exposure_us", 1000)
        self.declare_parameter("manual_iso", 100)
        self.declare_parameter("manual_wb_k", 1500)
        # 같은 프레임에 green도 threshold 이상으로 잡히면 red 무시 - 대회
        # 가서 제거를 고려해볼 것(원본 스크립트 주석 그대로 유지).
        self.declare_parameter("green_beats_red", True)

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.stop_threshold = int(self.get_parameter("stop_threshold").value)
        self.stop_exit_threshold = int(self.get_parameter("stop_exit_threshold").value)
        self._committed_state = "GO"
        self.device = str(self.get_parameter("device").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.min_frame_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.log_csv = bool(self.get_parameter("log_csv").value)
        self.use_manual_exposure = bool(self.get_parameter("use_manual_exposure").value)
        self.manual_exposure_us = int(self.get_parameter("manual_exposure_us").value)
        self.manual_iso = int(self.get_parameter("manual_iso").value)
        self.manual_wb_k = int(self.get_parameter("manual_wb_k").value)
        self.green_beats_red = bool(self.get_parameter("green_beats_red").value)

        self.pub = self.create_publisher(String, "/traffic_light", 10)

        self.red_buffer = deque(maxlen=self.buffer_size)
        self.csv_writer = None
        self.csv_file = None
        self._start_time = time.time()

        self.model = None
        self.device_handle = None
        self._running = False

        if YOLO is None:
            self.get_logger().warn(
                "ultralytics not installed (pip3 install ultralytics) - "
                "traffic_light_node idle, not publishing."
            )
            return
        if dai is None:
            self.get_logger().warn(
                "depthai not installed (pip3 install depthai) - "
                "traffic_light_node idle, not publishing."
            )
            return

        model_path = self.get_parameter("model_path").value
        try:
            self.model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f"failed to load YOLO model {model_path!r}: {exc}")
            return

        try:
            self._pipeline = self._build_pipeline()
            self.device_handle = dai.Device(self._pipeline)
            if self.use_manual_exposure:
                self._apply_manual_exposure()
        except Exception as exc:
            self.get_logger().warn(
                f"OAK-D not available ({exc}) - traffic_light_node idle, "
                "will not publish until it's connected and the node is restarted."
            )
            self.device_handle = None
            return

        if self.log_csv:
            self._open_csv()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info("traffic_light_node running.")

        # conf_threshold/exposure 등은 __init__에서 한 번만 읽고 끝이라
        # `ros2 param set`이 파라미터 서버 값만 바꾸고 실제 동작에는 반영이
        # 안 됐음(2026-08-06 확인) - 라이브 튜닝 되게 최소한만 다시 연결.
        self.add_on_set_parameters_callback(self._on_param_change)

    def _on_param_change(self, params):
        for p in params:
            if p.name == "conf_threshold":
                self.conf_threshold = float(p.value)
            elif p.name == "stop_threshold":
                self.stop_threshold = int(p.value)
            elif p.name == "stop_exit_threshold":
                self.stop_exit_threshold = int(p.value)
            elif p.name == "green_beats_red":
                self.green_beats_red = bool(p.value)
            elif p.name == "show_debug":
                self.show_debug = bool(p.value)
            elif p.name == "use_manual_exposure":
                self.use_manual_exposure = bool(p.value)
            elif p.name == "manual_exposure_us":
                self.manual_exposure_us = int(p.value)
            elif p.name == "manual_iso":
                self.manual_iso = int(p.value)
            elif p.name == "manual_wb_k":
                self.manual_wb_k = int(p.value)

        touched_exposure = any(
            p.name in ("manual_exposure_us", "manual_iso", "manual_wb_k", "use_manual_exposure")
            for p in params
        )
        if touched_exposure and self.use_manual_exposure and self.device_handle is not None:
            try:
                self._apply_manual_exposure()
            except Exception as exc:
                self.get_logger().warn(f"failed to re-apply manual exposure: {exc}")

        return SetParametersResult(successful=True)

    def _build_pipeline(self):
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(1280, 720)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)

        control_in = pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")
        control_in.out.link(cam_rgb.inputControl)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)
        return pipeline

    def _apply_manual_exposure(self):
        control_queue = self.device_handle.getInputQueue("control")
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(self.manual_exposure_us, self.manual_iso)
        ctrl.setManualWhiteBalance(self.manual_wb_k)
        control_queue.send(ctrl)
        self.get_logger().info(
            f"manual exposure applied: {self.manual_exposure_us}us "
            f"ISO{self.manual_iso} WB{self.manual_wb_k}K"
        )

    def _open_csv(self):
        csv_path = self.get_parameter("csv_path").value
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["Time", "State", "FPS", "Confidence", "Red_Count", "Buffer_Count"]
        )

    def _loop(self):
        q_rgb = self.device_handle.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        prev_time = time.time()

        while self._running and rclpy.ok():
            loop_start = time.time()

            in_rgb = q_rgb.tryGet()
            if in_rgb is None:
                time.sleep(0.005)
                continue
            frame = in_rgb.getCvFrame()

            results = self.model(frame, verbose=False, device=self.device)

            current_red = False
            current_green = False
            max_conf = 0.0
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls]
                    if conf > max_conf:
                        max_conf = conf
                    # 부분일치 - 모델마다 클래스 이름이 "red"/"green" 이
                    # 아니라 "red-traffic-lights"/"green-traffic-lights"
                    # 같은 식일 수 있음 (2026-08-06 새 best.pt에서 실제로
                    # 이랬음 - 정확 일치였으면 이 모델로는 아예 안 잡혔음).
                    label_lower = label.lower()
                    if "green" in label_lower and conf >= self.conf_threshold:
                        current_green = True
                    if "red" in label_lower and conf >= self.conf_threshold:
                        current_red = True

                    if self.show_debug and cv2 is not None:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame, f"{label} {conf:.2f}", (x1, max(y1 - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                        )

            if self.green_beats_red and current_green:
                current_red = False

            self.red_buffer.append(current_red)
            red_count = sum(self.red_buffer)
            buffer_full = len(self.red_buffer) == self.buffer_size
            if self._committed_state == "GO":
                if buffer_full and red_count >= self.stop_threshold:
                    self._committed_state = "STOP"
            else:
                if buffer_full and red_count < self.stop_exit_threshold:
                    self._committed_state = "GO"
            state = self._committed_state

            self.pub.publish(String(data=state))

            now = time.time()
            fps = 1.0 / (now - prev_time) if now > prev_time else 0.0
            prev_time = now

            if self.csv_writer is not None:
                self.csv_writer.writerow([
                    f"{now - self._start_time:.2f}", state, f"{fps:.1f}",
                    f"{max_conf:.2f}", red_count, len(self.red_buffer),
                ])
                self.csv_file.flush()

            if self.show_debug and cv2 is not None:
                color = (0, 0, 255) if state == "STOP" else (0, 255, 0)
                cv2.putText(frame, state, (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 4)
                cv2.imshow("Traffic Light", frame)
                cv2.waitKey(1)

            elapsed = time.time() - loop_start
            remaining = self.min_frame_interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def shutdown(self):
        self._running = False
        if self.csv_file is not None:
            self.csv_file.close()
        if self.show_debug and cv2 is not None:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
