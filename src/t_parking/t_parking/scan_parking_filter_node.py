#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_parking_filter_node.py

/scan -> /scan_parking. ROS1 front_scan_filter_300_60.py 의 ROS2 대체.

[중요 : 각도 규약]
ROS1 판은 angle_offset_deg 하나로 처리했지만, 지금은 라이다가 물리적으로
뒤집혀 있어서(바닥이 천장) 회전만으로는 표현이 안 된다. 반사(mirror)가 섞인다.

    p_base = Rz(laser_yaw) * Rx(pi) * p_sensor
    => 차량프레임 유효각 theta_v = laser_yaw + laser_yaw_extra + sign * theta_raw
       뒤집힘이면 sign = -1

사용자 실측 규약 (raw 180=앞, 90=좌, 270=우, 0=뒤) 은
    laser_yaw = pi, sign = -1
과 정확히 일치한다. 검산:
    raw 180 -> pi - pi   =    0 deg (앞)   OK
    raw  90 -> pi - pi/2 =  +90 deg (좌)   OK
    raw 270 -> pi - 3pi/2=  -90 deg (우)   OK
    raw   0 -> pi - 0    =  180 deg (뒤)   OK

이 노드는 각도 배열을 다시 쓰지 않는다. 원본 배열 구조를 유지하고
차량프레임 각도로 판정해서 통과 못한 빔만 inf 로 만든다.
=> 변환은 주차 노드 한 곳에서만 일어나므로 규약이 어긋날 여지가 없다.

publish_debug_frame=true 로 두면 통과빔 개수/좌우 최소거리를 주기적으로 찍어
장착 방향 검증에 쓸 수 있다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanParkingFilter(Node):

    def __init__(self):
        super().__init__('scan_parking_filter')

        self.input_topic = self.p('input_scan', '/scan')
        self.output_topic = self.p('output_scan', '/scan_parking')

        self.min_range = float(self.p('min_range', 0.25))
        self.max_range = float(self.p('max_range', 8.0))

        # 라이다 장착 (주차 노드와 동일하게 유지할 것)
        self.laser_yaw = float(self.p('laser_yaw', math.pi))
        self.laser_yaw_extra = float(self.p('laser_yaw_extra', 0.0))
        sign = float(self.p('laser_angle_sign', -1.0))
        self.sign = 1.0 if sign >= 0.0 else -1.0

        # 차량프레임 유지 각도창 (0 = 정면, + = 좌)
        self.keep_min_deg = float(self.p('keep_min_deg', -135.0))
        self.keep_max_deg = float(self.p('keep_max_deg', 135.0))

        self.debug = bool(self.p('publish_debug_frame', True))
        self.debug_period = float(self.p('debug_period_sec', 2.0))
        self._last_dbg = 0.0

        self.pub = self.create_publisher(LaserScan, self.output_topic,
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.input_topic, self.cb,
                                 qos_profile_sensor_data)

        self.get_logger().warn(
            '[scan_filter] %s -> %s | laser_yaw=%.3f sign=%+.0f keep=[%.0f, %.0f]deg '
            'range=[%.2f, %.2f]'
            % (self.input_topic, self.output_topic, self.laser_yaw, self.sign,
               self.keep_min_deg, self.keep_max_deg, self.min_range, self.max_range))

    def p(self, name, default):
        self.declare_parameter(name, default)
        v = self.get_parameter(name).value
        return default if v is None else v

    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_deg(a):
        return (a + 180.0) % 360.0 - 180.0

    def cb(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        r = np.asarray(msg.ranges, dtype=float)
        raw = msg.angle_min + np.arange(n, dtype=float) * msg.angle_increment
        theta_v = self.laser_yaw + self.laser_yaw_extra + self.sign * raw
        deg = self._wrap_deg(np.degrees(theta_v))

        lo, hi = self.keep_min_deg, self.keep_max_deg
        if lo <= hi:
            ang_ok = (deg >= lo) & (deg <= hi)
        else:  # 0도를 넘어가는 창
            ang_ok = (deg >= lo) | (deg <= hi)

        rng_ok = np.isfinite(r) & (r >= self.min_range) & (r <= self.max_range)
        keep = ang_ok & rng_ok

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = float(self.min_range)
        out.range_max = float(self.max_range)
        filtered = np.where(keep, r, float('inf'))
        out.ranges = [float(v) for v in filtered]
        if len(msg.intensities) == n:
            inten = np.asarray(msg.intensities, dtype=float)
            out.intensities = [float(v) for v in np.where(keep, inten, 0.0)]
        self.pub.publish(out)

        if self.debug:
            t = float(self.get_clock().now().nanoseconds) * 1e-9
            if t - self._last_dbg >= self.debug_period:
                self._last_dbg = t
                self._report(deg, r, keep)

    def _report(self, deg, r, keep):
        """장착 방향 검증용. 정면/좌/우/후방 섹터 최소거리를 찍는다."""
        def sector_min(center, half=10.0):
            d = np.abs(self._wrap_deg(deg - center))
            m = (d <= half) & np.isfinite(r) & (r > 0.05)
            return float(np.min(r[m])) if bool(np.any(m)) else float('nan')

        self.get_logger().info(
            '[scan_filter] keep=%d/%d | 정면(0deg)=%.2f 좌(+90)=%.2f '
            '우(-90)=%.2f 후방(180)=%.2f'
            % (int(np.count_nonzero(keep)), len(r), sector_min(0.0),
               sector_min(90.0), sector_min(-90.0), sector_min(180.0)))


def main(args=None):
    rclpy.init(args=args)
    node = ScanParkingFilter()
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
