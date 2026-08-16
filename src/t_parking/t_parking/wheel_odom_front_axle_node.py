#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_odom_front_axle_node.py

전방 구동축(앞 차축) 엔코더로 base_link(= 뒤 차축) odom 을 만든다.

[왜 별도 처리가 필요한가]
엔코더가 뒤 차축에 있으면 엔코더 거리 = base_link 이동거리이므로 그대로 적분하면 된다.
그런데 구동모터가 앞에 달려 있으면 엔코더는 '앞 차축이 그리는 호의 길이'를 센다.
조향각 delta 에서 두 축의 반경 관계는

        R_rear  = L / tan(delta)
        R_front = L / sin(delta) = sqrt(R_rear^2 + L^2)

이므로 같은 회전각에 대해 앞 차축이 더 많이 움직인다.

        ds_front / ds_rear = R_front / R_rear = 1 / cos(delta)

따라서 앞 차축 엔코더 거리 ds_f 를 그대로 base_link 이동거리로 쓰면
조향할 때마다 1/cos(delta) 배 과대평가된다. delta=30deg 에서 15% 오차다.
주차는 90도 선회 중 거리적분으로 시작점/깊이를 판정하므로 이 오차가 그대로
슬롯 편심으로 나타난다.

올바른 전축구동 자전거모델:

        ds_r  = ds_f * cos(delta)            # 뒤 차축 이동거리
        dpsi  = ds_f * sin(delta) / L        # 요 변화 (= ds_r * tan(delta) / L)
        x    += ds_r * cos(psi + dpsi/2)
        y    += ds_r * sin(psi + dpsi/2)
        psi  += dpsi

[요(yaw) 소스]
yaw_source:
  'encoder' : 위 자전거모델로 적분 (조향각 정확도에 민감)
  'imu'     : /imu/data 의 yaw 사용 (권장. 90도 선회 누적오차에 강함)
  'fused'   : imu 를 기준으로 하고 imu 끊기면 encoder 로 폴백
주차 노드는 yaw 로 세그먼트를 전환하므로 실차에서는 imu 또는 fused 를 권장한다.

[CAN 파싱 - 반드시 확인할 부분]
채널 can0 / bitrate 500000 은 알고 있으나, 엔코더 카운트가 실려오는
CAN ID 와 바이트 배치는 차량 펌웨어마다 다르므로 파라미터로 뺐다.

    encoder_can_id      : 엔코더 프레임 ID (0 이면 모든 ID 허용)
    count_byte_offset   : 카운트 시작 바이트 (0~7)
    count_byte_len      : 카운트 바이트 수 (1, 2, 4)
    count_little_endian : 바이트 순서
    count_signed        : 부호 있는 정수인지

값을 모르면 dump_frames:=true 로 띄워 프레임을 관찰한 뒤 채우면 된다.
CAN 링크는 이 노드가 올리지 않는다. 미리 아래를 실행할 것:

    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up

encoder_topic 을 주면 CAN 대신 그 토픽(std_msgs/Int32, 누적 카운트)을 쓴다.
"""

import math
import socket
import struct
import threading

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Int32
from tf2_ros import TransformBroadcaster

from .geometry import clamp, normalize_angle, quat_from_yaw, yaw_from_quat

CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


class WheelOdomFrontAxle(Node):

    def __init__(self):
        super().__init__('wheel_odom_front_axle')

        # ---------------- frames / topics ----------------
        self.odom_frame = self.p('odom_frame', 'odom')
        self.base_frame = self.p('base_frame', 'base_link')
        self.odom_topic = self.p('odom_topic', '/wheel_odom')
        self.publish_tf = bool(self.p('publish_tf', True))

        # ---------------- 차량 / 엔코더 ----------------
        self.wheel_base = float(self.p('wheel_base', 0.735))
        self.counts_per_m = float(self.p('counts_per_m', 341.295007))
        self.m_per_count = float(self.p('m_per_count', 0.002930016494))
        if self.m_per_count <= 0.0 and self.counts_per_m > 0.0:
            self.m_per_count = 1.0 / self.counts_per_m
        self.encoder_modulus = int(self.p('encoder_modulus', 65536))
        self.encoder_sign = float(self.p('encoder_sign', 1.0))
        self.encoder_on_front_axle = bool(self.p('encoder_on_front_axle', True))
        self.max_jump_counts = int(self.p('max_jump_counts', 20000))

        # ---------------- 조향 ----------------
        self.steer_topic = self.p('steer_cmd_topic', '/cmd_steer')
        self.max_steer_cmd = int(self.p('max_steer_cmd', 30))
        self.max_steer_angle_left_deg = float(self.p('max_steer_angle_left_deg', 30.0))
        self.max_steer_angle_right_deg = float(self.p('max_steer_angle_right_deg', 30.0))
        # cmd 부호 규약: 음수 = 좌, 양수 = 우 (주차 노드와 동일)
        self.steer_cmd_sign = float(self.p('steer_cmd_sign', 1.0))
        self.steer_lag_alpha = float(self.p('steer_lag_alpha', 0.35))

        # ---------------- yaw source ----------------
        self.yaw_source = str(self.p('yaw_source', 'fused')).strip().lower()
        self.imu_topic = self.p('imu_topic', '/imu/data')
        self.imu_timeout = float(self.p('imu_timeout_sec', 0.5))

        # ---------------- CAN ----------------
        self.encoder_topic = str(self.p('encoder_topic', '')).strip()
        self.can_interface = str(self.p('can_interface', 'can0'))
        self.encoder_can_id = int(self.p('encoder_can_id', 0))
        self.count_byte_offset = int(self.p('count_byte_offset', 0))
        self.count_byte_len = int(self.p('count_byte_len', 2))
        self.count_little_endian = bool(self.p('count_little_endian', True))
        self.count_signed = bool(self.p('count_signed', False))
        self.dump_frames = bool(self.p('dump_frames', False))

        self.publish_rate = float(self.p('publish_rate', 50.0))

        # ---------------- state ----------------
        self.x = self.y = self.psi = 0.0
        self.psi_enc = 0.0
        self.vx = self.wz = 0.0
        self.steer_cmd = 0.0
        self.steer_filt = 0.0
        self.last_count = None
        self.count_accum = 0
        self.pending_counts = 0
        self.imu_yaw = None
        self.imu_yaw0 = None
        self.imu_last = None
        self.lock = threading.Lock()
        self._prev_t = None

        # ---------------- ros io ----------------
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Int16, self.steer_topic, self.steer_cb, 10)
        if self.yaw_source in ('imu', 'fused'):
            self.create_subscription(Imu, self.imu_topic, self.imu_cb, 20)
        if self.encoder_topic:
            self.create_subscription(Int32, self.encoder_topic, self.encoder_cb, 20)
            self.get_logger().warn('[odom] encoder from topic %s' % self.encoder_topic)
        else:
            self._start_can_thread()

        self.create_timer(1.0 / max(self.publish_rate, 1.0), self.on_timer)

        self.get_logger().warn(
            '[odom] front_axle=%s WB=%.3f m/count=%.9f modulus=%d sign=%+.1f '
            'yaw_source=%s'
            % (self.encoder_on_front_axle, self.wheel_base, self.m_per_count,
               self.encoder_modulus, self.encoder_sign, self.yaw_source))

    def p(self, name, default):
        self.declare_parameter(name, default)
        v = self.get_parameter(name).value
        return default if v is None else v

    # ==================================================================
    def steer_cb(self, msg):
        self.steer_cmd = float(msg.data) * self.steer_cmd_sign

    def imu_cb(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = y
        self.imu_yaw = normalize_angle(y - self.imu_yaw0)
        self.imu_last = float(self.get_clock().now().nanoseconds) * 1e-9
        self.wz = float(msg.angular_velocity.z)

    def encoder_cb(self, msg):
        self._push_count(int(msg.data))

    # ==================================================================
    def _start_can_thread(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.settimeout(0.2)
            self.sock.bind((self.can_interface,))
        except Exception as e:
            self.get_logger().error(
                '[odom] CAN bind 실패 (%s): %s | '
                'sudo ip link set %s type can bitrate 500000 && '
                'sudo ip link set %s up 확인'
                % (self.can_interface, e, self.can_interface, self.can_interface))
            self.sock = None
            return
        t = threading.Thread(target=self._can_loop, daemon=True)
        t.start()
        self.get_logger().warn('[odom] CAN %s 수신 시작 (id=0x%X off=%d len=%d le=%s)'
                               % (self.can_interface, self.encoder_can_id,
                                  self.count_byte_offset, self.count_byte_len,
                                  self.count_little_endian))

    def _can_loop(self):
        while rclpy.ok() and self.sock is not None:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except Exception:
                continue
            if len(frame) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            can_id &= 0x1FFFFFFF
            if self.dump_frames:
                self.get_logger().info('[can] id=0x%03X dlc=%d data=%s'
                                       % (can_id, dlc, data[:dlc].hex()))
            if self.encoder_can_id and can_id != self.encoder_can_id:
                continue
            cnt = self._extract_count(data, dlc)
            if cnt is not None:
                self._push_count(cnt)

    def _extract_count(self, data, dlc):
        o, n = self.count_byte_offset, self.count_byte_len
        if o + n > max(dlc, 0) or o < 0 or n not in (1, 2, 4):
            return None
        raw = data[o:o + n]
        order = 'little' if self.count_little_endian else 'big'
        return int.from_bytes(raw, order, signed=self.count_signed)

    # ==================================================================
    def _push_count(self, cnt):
        """modulus 랩어라운드를 최단거리로 풀어 누적 카운트를 만든다."""
        with self.lock:
            if self.last_count is None:
                self.last_count = int(cnt)
                return
            mod = self.encoder_modulus
            d = int(cnt) - int(self.last_count)
            if mod > 0:
                d = (d + mod // 2) % mod - mod // 2
            if abs(d) > self.max_jump_counts:
                self.get_logger().warn('[odom] 비정상 카운트 점프 %d 무시' % d)
                self.last_count = int(cnt)
                return
            self.last_count = int(cnt)
            self.pending_counts += d

    # ==================================================================
    def steer_angle(self):
        """cmd -> 조향각 [rad], 좌(+yaw 증가) 부호 포함."""
        cmd = self.steer_filt
        if self.max_steer_cmd <= 0:
            return 0.0
        frac = clamp(abs(cmd) / float(self.max_steer_cmd), 0.0, 1.0)
        dmax = self.max_steer_angle_left_deg if cmd < 0.0 else self.max_steer_angle_right_deg
        mag = math.radians(dmax) * frac
        # cmd 음수 = 좌 = +yaw
        return -math.copysign(mag, cmd) if abs(cmd) > 1e-9 else 0.0

    def on_timer(self):
        now = self.get_clock().now()
        t = float(now.nanoseconds) * 1e-9
        dt = (t - self._prev_t) if self._prev_t is not None else (1.0 / self.publish_rate)
        self._prev_t = t
        dt = max(dt, 1e-4)

        # 조향 1차 지연 반영 (실제 조향은 명령을 즉시 따르지 않는다)
        a = clamp(self.steer_lag_alpha, 0.0, 1.0)
        self.steer_filt = (1.0 - a) * self.steer_filt + a * self.steer_cmd

        with self.lock:
            counts = self.pending_counts
            self.pending_counts = 0

        ds_enc = float(counts) * self.m_per_count * self.encoder_sign
        delta = self.steer_angle()

        if self.encoder_on_front_axle:
            # 앞 차축 호길이 -> 뒤 차축 이동거리 / 요변화
            ds_r = ds_enc * math.cos(delta)
            dpsi_enc = (ds_enc * math.sin(delta) / self.wheel_base) \
                if self.wheel_base > 1e-6 else 0.0
        else:
            ds_r = ds_enc
            dpsi_enc = (ds_r * math.tan(delta) / self.wheel_base) \
                if self.wheel_base > 1e-6 else 0.0

        self.psi_enc = normalize_angle(self.psi_enc + dpsi_enc)

        # ---- yaw 선택 ----
        imu_fresh = (self.imu_yaw is not None and self.imu_last is not None
                     and (t - self.imu_last) <= self.imu_timeout)
        if self.yaw_source == 'imu' and imu_fresh:
            psi_new = self.imu_yaw
        elif self.yaw_source == 'fused' and imu_fresh:
            psi_new = self.imu_yaw
        else:
            psi_new = self.psi_enc

        dpsi = normalize_angle(psi_new - self.psi)

        # 중점(midpoint) 적분
        self.x += ds_r * math.cos(self.psi + 0.5 * dpsi)
        self.y += ds_r * math.sin(self.psi + 0.5 * dpsi)
        self.psi = normalize_angle(psi_new)

        self.vx = ds_r / dt
        if not imu_fresh:
            self.wz = dpsi / dt

        self.publish(now)

    # ==================================================================
    def publish(self, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        qx, qy, qz, qw = quat_from_yaw(self.psi)
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.angular.z = float(self.wz)
        # 대각 공분산 (직진 정확, 횡/요는 보수적)
        cov = [0.0] * 36
        cov[0] = 0.002
        cov[7] = 0.010
        cov[35] = 0.010
        msg.pose.covariance = cov
        self.pub_odom.publish(msg)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(self.x)
            tf.transform.translation.y = float(self.y)
            tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.tf_bc.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomFrontAxle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
