import argparse
import msvcrt
import os
import struct
import time

import can


DRIVE_STATUS_ID = 0x102
STEERING_STATUS_ID = 0x101

COMMAND_ID = 0x200


def parse_args():
    parser = argparse.ArgumentParser(description="LK TC275 PCAN Python example")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=500_000)
    parser.add_argument("--monitor-only", action="store_true")
    parser.add_argument("--rpm", type=int, default=0)
    parser.add_argument("--steer", type=int, default=0)
    parser.add_argument("--enable", type=int, choices=(0, 1), default=0)
    parser.add_argument("--rpm-step", type=int, default=10)
    parser.add_argument("--steer-step", type=int, default=5)
    parser.add_argument("--max-rpm", type=int, default=200)
    parser.add_argument("--max-steer", type=int, default=20)
    parser.add_argument("--period", type=float, default=0.02)
    return parser.parse_args()


def send_command(bus, rpm, steer, enable, stop_mode):
    # LK command payload:
    # byte0-1: target rpm, signed int16
    # byte2-3: steer angle, signed int16
    # byte4  : enable, uint8
    # byte5  : stop mode, 0=normal, 1=flat stop, 2=hill stop
    # byte6-7: unused
    payload = struct.pack("<hhBBH", rpm, steer, enable, stop_mode, 0)
    bus.send(
        can.Message(
            arbitration_id=COMMAND_ID,
            is_extended_id=False,
            data=payload,
        ),
        timeout=1.0,
    )


def parse_drive_status(data):
    # LK 0x102 payload:
    # byte0-1: encoder count, signed int16
    # byte2-3: actual rpm x10, signed int16
    # byte4-5: output pwm duty %, signed int16
    # byte6-7: target rpm, signed int16
    encoder_count, rpm_x10, pwm_duty, target_rpm = struct.unpack("<hhhh", data)
    return {
        "encoder_count": encoder_count,
        "rpm": rpm_x10 / 10.0,
        "pwm_duty": pwm_duty,
        "target_rpm": target_rpm,
    }


def parse_steering_status(data):
    # LK 0x101 payload:
    # byte0-1: current pot, uint16
    # byte2-3: target pot, uint16
    # byte4-5: current angle x10, signed int16
    # byte6-7: target angle x10, signed int16
    current_pot, target_pot, current_angle_x10, target_angle_x10 = (
        struct.unpack("<HHhh", data)
    )
    return {
        "current_pot": current_pot,
        "target_pot": target_pot,
        "current_angle": current_angle_x10 / 10.0,
        "target_angle": target_angle_x10 / 10.0,
    }


def clamp(value, low, high):
    return max(low, min(high, value))


def read_key():
    if not msvcrt.kbhit():
        return None

    key = msvcrt.getch()
    if key in (b"\x00", b"\xe0"):
        msvcrt.getch()
        return None

    try:
        return key.decode("ascii").lower()
    except UnicodeDecodeError:
        return None


def print_status(rpm, steer, enable, stop_mode, drive_status, steering_status):
    drive_text = "drive: --"
    steering_text = "steer_fb: --"

    if drive_status is not None:
        drive_text = (
            f"enc={drive_status['encoder_count']} "
            f"rpm={drive_status['rpm']:.1f} "
            f"pwm={drive_status['pwm_duty']} "
            f"trpm={drive_status['target_rpm']}"
        )

    if steering_status is not None:
        steering_text = (
            f"pot={steering_status['current_pot']} "
            f"ang={steering_status['current_angle']:.1f} "
            f"tang={steering_status['target_angle']:.1f}"
        )

    if stop_mode == 1:
        stop_text = "flat"
    elif stop_mode == 2:
        stop_text = "hill"
    else:
        stop_text = "run"

    command_line = (
        f"TX 0x200 cmd rpm={rpm:4d} steer={steer:4d} "
        f"en={enable} stop={stop_text} | "
        "w/s rpm a/d steer e en space flat t hill q quit"
    )
    feedback_line = f"RX 0x102 {drive_text} | 0x101 {steering_text}"

    width = os.get_terminal_size().columns
    width = max(1, width - 1)
    print(
        "\r\033[K" + command_line[:width] +
        "\n\033[K" + feedback_line[:width] +
        "\033[F",
        end="",
        flush=True,
    )


def main():
    args = parse_args()
    rpm = args.rpm
    steer = args.steer
    enable = args.enable
    stop_mode = 0
    drive_status = None
    steering_status = None
    next_send_time = 0.0

    with can.Bus(
        interface="pcan",
        channel=args.channel,
        bitrate=args.bitrate,
    ) as bus:
        print("PCAN LK terminal control")

        while True:
            key = read_key()
            if key == "q":
                break
            elif key == "w":
                rpm = clamp(rpm + args.rpm_step, -args.max_rpm, args.max_rpm)
                stop_mode = 0
            elif key == "s":
                rpm = clamp(rpm - args.rpm_step, -args.max_rpm, args.max_rpm)
                stop_mode = 0
            elif key == "a":
                steer = clamp(steer - args.steer_step,
                              -args.max_steer, args.max_steer)
            elif key == "d":
                steer = clamp(steer + args.steer_step,
                              -args.max_steer, args.max_steer)
            elif key == "e":
                enable = 0 if enable else 1
            elif key == " ":
                rpm = 0
                stop_mode = 1
            elif key == "t":
                rpm = 0
                stop_mode = 2

            msg = bus.recv(timeout=0.0)
            while msg is not None:
                if msg.arbitration_id == DRIVE_STATUS_ID and len(msg.data) == 8:
                    drive_status = parse_drive_status(msg.data)
                elif msg.arbitration_id == STEERING_STATUS_ID and len(msg.data) == 8:
                    steering_status = parse_steering_status(msg.data)
                msg = bus.recv(timeout=0.0)

            now = time.monotonic()
            if (args.monitor_only is False) and (now >= next_send_time):
                send_command(bus, rpm, steer, enable, stop_mode)
                next_send_time = now + args.period

            print_status(rpm, steer, enable, stop_mode,
                         drive_status, steering_status)
            time.sleep(0.005)

        if args.monitor_only is False:
            send_command(bus, 0, steer, 0, 1)
        print("\n")


if __name__ == "__main__":
    main()
