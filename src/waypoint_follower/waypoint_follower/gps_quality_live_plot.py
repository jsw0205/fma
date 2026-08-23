#!/usr/bin/env python3
"""GPS Fixed 판정 관련 값들 실시간 그래프 - EMI 차폐(알루미늄 호일 등) 효과
확인용 (2026-08-19).

`gps_quality_monitor.py`(텍스트 한 줄 출력)의 시각화 버전. 호일을
씌우거나 뗄 때 각 값이 어떻게/얼마나 빨리 바뀌는지 그래프로 바로 보임 -
"변화속도"까지 눈으로 확인 가능하게 최근 N초 롤링 윈도우로 계속 흘러감.

사용법:
    ros2 run waypoint_follower gps_quality_live_plot
    ros2 run waypoint_follower gps_quality_live_plot --ros-args -p window_sec:=120.0

4개 패널:
  1. carrSoln (NONE/FLOAT/FIXED) - 계단식
  2. h_acc (cm) - 낮을수록 좋음
  3. jam_ind (0-255) + jamming_state 배경색 (초록=OK, 노랑=WARNING, 빨강=CRITICAL)
  4. noise_per_ms, numSV (두 축)

matplotlib GUI 필요 (디스플레이 있는 환경에서 실행).
"""
import threading
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.font_manager as fm

import rclpy
from rclpy.node import Node
from ublox_msgs.msg import NavPVT, MonHW

# 한글 폰트 - 이거 없으면 그래프 제목/라벨의 한글이 네모(□)로 깨짐.
# 이 워크스페이스의 다른 시각화 스크립트들과 동일한 폴백 순서.
for _cand in ["NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "Malgun Gothic"]:
    _matches = [f for f in fm.fontManager.ttflist if _cand.lower() in f.name.lower()]
    if _matches:
        plt.rcParams["font.family"] = _matches[0].name
        break
plt.rcParams["axes.unicode_minus"] = False

CARR_SOLN_NAMES = {0: "NONE", 64: "FLOAT", 128: "FIXED"}
CARR_SOLN_Y = {0: 0, 64: 1, 128: 2}
JAMMING_COLORS = {0: "0.85", 4: "#c8e6c9", 8: "#fff59d", 12: "#ef9a9a"}
JAMMING_NAMES = {0: "UNKNOWN/OFF", 4: "OK", 8: "WARNING", 12: "CRITICAL"}


class GpsQualityLivePlot(Node):
    def __init__(self, buf):
        super().__init__("gps_quality_live_plot")
        self.declare_parameter("window_sec", 120.0)
        self.buf = buf
        self.t0 = self.get_clock().now()
        self.create_subscription(NavPVT, "/navpvt", self._on_navpvt, 10)
        self.create_subscription(MonHW, "/monhw", self._on_monhw, 10)
        self.get_logger().info(
            "gps_quality_live_plot 시작 - /navpvt, /monhw 기다리는 중..."
        )

    def _now_s(self):
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def _on_navpvt(self, msg):
        t = self._now_s()
        carr = msg.flags & msg.FLAGS_CARRIER_PHASE_MASK
        self.buf["t_pvt"].append(t)
        self.buf["carr"].append(CARR_SOLN_Y.get(carr, 0))
        self.buf["h_acc_cm"].append(msg.h_acc / 10.0)
        self.buf["num_sv"].append(msg.num_sv)
        self.buf["p_dop"].append(msg.p_dop / 100.0)

    def _on_monhw(self, msg):
        t = self._now_s()
        self.buf["t_hw"].append(t)
        self.buf["jam_ind"].append(msg.jam_ind)
        jstate = msg.flags & msg.FLAGS_JAMMING_STATE_MASK
        self.buf["jam_state"].append(jstate)
        self.buf["noise"].append(msg.noise_per_ms)


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    maxlen = 20000
    buf = {
        "t_pvt": deque(maxlen=maxlen), "carr": deque(maxlen=maxlen),
        "h_acc_cm": deque(maxlen=maxlen), "num_sv": deque(maxlen=maxlen),
        "p_dop": deque(maxlen=maxlen),
        "t_hw": deque(maxlen=maxlen), "jam_ind": deque(maxlen=maxlen),
        "jam_state": deque(maxlen=maxlen), "noise": deque(maxlen=maxlen),
    }

    node = GpsQualityLivePlot(buf)
    window_sec = node.get_parameter("window_sec").value

    spin_t = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_t.start()

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle("GPS 품질 실시간 모니터 - EMI 차폐 테스트용 "
                  "(알루미늄 호일 씌우고/떼면서 변화 확인)", fontsize=13, fontweight="bold")
    # 프레임마다 갱신할 "현재값" 텍스트 - 매번 지웠다 새로 만들면 suptitle까지
    # 같이 날아가버려서(fig.texts에 같이 들어있음), 한 번만 만들어두고 이후엔
    # set_text()로만 내용을 바꾼다.
    now_text = fig.text(0.5, 0.955, "", ha="center", fontsize=10, color="darkred")

    ax_carr, ax_acc, ax_jam, ax_noise = axes

    def redraw(_frame):
        for ax in axes:
            ax.cla()

        t_pvt = list(buf["t_pvt"])
        t_hw = list(buf["t_hw"])
        now = max([t_pvt[-1] if t_pvt else 0.0, t_hw[-1] if t_hw else 0.0])
        lo = max(0.0, now - window_sec)

        # 1) carrSoln
        ax_carr.step(t_pvt, list(buf["carr"]), where="post", color="tab:blue", linewidth=1.5)
        ax_carr.set_yticks([0, 1, 2])
        ax_carr.set_yticklabels(["NONE", "FLOAT", "FIXED"])
        ax_carr.set_ylim(-0.3, 2.3)
        ax_carr.set_title("carrSoln", fontsize=11, loc="left")
        ax_carr.grid(True, alpha=0.3)

        # 2) h_acc
        ax_acc.plot(t_pvt, list(buf["h_acc_cm"]), color="tab:purple", linewidth=1.0)
        ax_acc.set_ylabel("h_acc (cm)")
        ax_acc.set_title("수평 오차 추정치 (낮을수록 좋음, log 스케일)", fontsize=11, loc="left")
        ax_acc.set_yscale("log")
        ax_acc.grid(True, alpha=0.3, which="both")

        # 3) jam_ind + jamming_state 배경
        jam_states = list(buf["jam_state"])
        for i in range(len(t_hw) - 1):
            ax_jam.axvspan(t_hw[i], t_hw[i + 1],
                            color=JAMMING_COLORS.get(jam_states[i], "0.9"), alpha=0.6, linewidth=0)
        ax_jam.plot(t_hw, list(buf["jam_ind"]), color="black", linewidth=1.2)
        ax_jam.set_ylabel("jam_ind (0-255)")
        ax_jam.set_ylim(0, 255)
        ax_jam.set_title("재밍 지표 (배경색: 초록=OK 노랑=WARNING 빨강=CRITICAL)",
                          fontsize=11, loc="left")
        ax_jam.grid(True, alpha=0.3)

        # 4) noise_per_ms + numSV (twin axis)
        ax_noise.plot(t_hw, list(buf["noise"]), color="tab:red", linewidth=1.0, label="noise_per_ms")
        ax_noise.set_ylabel("noise_per_ms", color="tab:red")
        ax_noise.tick_params(axis="y", labelcolor="tab:red")
        ax_noise2 = ax_noise.twinx()
        ax_noise2.plot(t_pvt, list(buf["num_sv"]), color="tab:green", linewidth=1.0,
                        alpha=0.7, label="numSV")
        ax_noise2.set_ylabel("numSV", color="tab:green")
        ax_noise2.tick_params(axis="y", labelcolor="tab:green")
        ax_noise.set_title("노이즈 레벨 + 사용 위성수", fontsize=11, loc="left")
        ax_noise.set_xlabel("t (s, 이 스크립트 시작 기준)")
        ax_noise.grid(True, alpha=0.3)

        for ax in axes:
            ax.set_xlim(lo, max(now, lo + 5))

        # 현재값 텍스트 요약 (최신 샘플)
        if buf["carr"] and buf["h_acc_cm"] and buf["jam_ind"]:
            carr_now = {v: k for k, v in CARR_SOLN_Y.items()}[buf["carr"][-1]]
            txt = (f"NOW: carrSoln={CARR_SOLN_NAMES.get(carr_now,'?')}  "
                   f"h_acc={buf['h_acc_cm'][-1]:.1f}cm  "
                   f"jam_ind={buf['jam_ind'][-1]}/255  "
                   f"jamming={JAMMING_NAMES.get(jam_states[-1] if jam_states else 0,'?')}")
            now_text.set_text(txt)

        plt.tight_layout(rect=[0, 0, 1, 0.94])

    ani = animation.FuncAnimation(fig, redraw, interval=500, cache_frame_data=False)
    # ani를 로컬 변수로만 두면 일부 환경(백엔드/GC 타이밍)에서 참조가 끊겨
    # 애니메이션이 첫 프레임 이후 멈추는 matplotlib의 잘 알려진 함정이 있음 -
    # fig에 강한 참조를 하나 더 걸어서 plt.show() 도는 동안 확실히 살아있게 함.
    fig._anim_ref = ani
    plt.show()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
