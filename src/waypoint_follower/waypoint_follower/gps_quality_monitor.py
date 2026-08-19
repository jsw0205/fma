#!/usr/bin/env python3
"""GPS Fixed/Float 판정 + EMI 진단용 실시간 모니터.

ZED 전원/스트리밍 유무에 따른 GPS 품질 변화를 명확한 라벨과 함께 보려고
2026-08-18 만듦 - "그 값이 뭐였는지 모르겠다"는 일이 다시 없게, /navpvt
(carrSoln, h_acc, num_sv, p_dop)와 /monhw(jam_ind, jamming_state,
noise_per_ms)를 한 줄에 같이 찍음.

사용법:
    ros2 run waypoint_follower gps_quality_monitor
(ublox_gps_node가 pvt/hw 퍼블리셔 켜져 있어야 함 - f9p_bringup 기본 설정
이미 둘 다 true, ublox_rover.yaml 참고)
"""
import rclpy
from rclpy.node import Node
from ublox_msgs.msg import NavPVT, MonHW

CARR_SOLN_NAMES = {0: "NONE", 64: "FLOAT", 128: "FIXED"}
JAMMING_STATE_NAMES = {0: "UNKNOWN/OFF", 4: "OK", 8: "WARNING", 12: "CRITICAL"}


class GpsQualityMonitor(Node):
    def __init__(self):
        super().__init__("gps_quality_monitor")
        self.navpvt = None
        self.monhw = None
        self.create_subscription(NavPVT, "/navpvt", self._on_navpvt, 10)
        self.create_subscription(MonHW, "/monhw", self._on_monhw, 10)
        self.create_timer(0.5, self._print_status)
        self.get_logger().info(
            "gps_quality_monitor 시작 - /navpvt, /monhw 기다리는 중..."
        )

    def _on_navpvt(self, msg):
        self.navpvt = msg

    def _on_monhw(self, msg):
        self.monhw = msg

    def _print_status(self):
        if self.navpvt is None:
            print("[navpvt 없음 - ublox_gps_node가 pvt:true로 떠 있는지 확인]")
            return

        carr = self.navpvt.flags & self.navpvt.FLAGS_CARRIER_PHASE_MASK
        carr_name = CARR_SOLN_NAMES.get(carr, f"?({carr})")
        h_acc_cm = self.navpvt.h_acc / 10.0  # mm -> cm
        p_dop = self.navpvt.p_dop / 100.0
        num_sv = self.navpvt.num_sv

        if self.monhw is not None:
            jam_ind = self.monhw.jam_ind
            jstate = self.monhw.flags & self.monhw.FLAGS_JAMMING_STATE_MASK
            jstate_name = JAMMING_STATE_NAMES.get(jstate, f"?({jstate})")
            noise = self.monhw.noise_per_ms
            mon_str = f"jam_ind={jam_ind:3d}/255 jamming={jstate_name:12s} noise={noise}"
        else:
            mon_str = "[monhw 없음]"

        print(
            f"carrSoln={carr_name:5s} h_acc={h_acc_cm:8.2f}cm "
            f"pDOP={p_dop:5.2f} numSV={num_sv:2d}  |  {mon_str}"
        )


def main():
    rclpy.init()
    node = GpsQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
