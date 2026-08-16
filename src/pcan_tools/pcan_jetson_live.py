#!/usr/bin/env python3
import can
import struct
import sys
import termios
import tty
import select
import subprocess
import time
import os


CAN_CHANNEL = "can0"
CAN_BITRATE = 500000
TX_ID = 0x200
DRIVE_STATUS_ID = 0x102
STEERING_STATUS_ID = 0x101

target_rpm = 0
target_steer = 0
enable = 0
stop_mode = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def setup_can_interface(channel, bitrate):
    subprocess.run(["sudo", "ip", "link", "set", channel, "down"], check=False)

    result = subprocess.run(
        ["sudo", "ip", "link", "set", channel, "up", "type", "can", "bitrate", str(bitrate)],
        check=False
    )

    if result.returncode != 0:
        raise SystemExit(
            f"'{channel}' 인터페이스를 올리지 못했습니다. "
            "어댑터가 꽂혀 있는지, sudo 권한이 있는지 확인하세요."
        )


def get_key():
    readable, _, _ = select.select([sys.stdin], [], [], 0)

    if readable:
        return sys.stdin.read(1)

    return None


def make_control_data(rpm, steer, motor_enable, motor_stop_mode):
    # byte0-1: rpm, byte2-3: steer, byte4: enable,
    # byte5: stop mode (0=normal, 1=flat, 2=hill), byte6-7: unused
    return struct.pack(
        "<hhBBH",
        int(rpm),
        int(steer),
        int(motor_enable),
        int(motor_stop_mode),
        0
    )


def send_control(bus):
    data = make_control_data(
        target_rpm,
        target_steer,
        enable,
        stop_mode
    )

    message = can.Message(
        arbitration_id=TX_ID,
        data=data,
        is_extended_id=False
    )

    bus.send(message)


def parse_drive_status(data):
    # byte0-1: encoder count, byte2-3: actual rpm x10,
    # byte4-5: output pwm duty %, byte6-7: target rpm
    encoder_count, rpm_x10, pwm_duty, target_rpm = struct.unpack("<hhhh", data)
    return {
        "encoder_count": encoder_count,
        "rpm": rpm_x10 / 10.0,
        "pwm_duty": pwm_duty,
        "target_rpm": target_rpm,
    }


def parse_steering_status(data):
    # byte0-1: current pot, byte2-3: target pot,
    # byte4-5: current angle x10, byte6-7: target angle x10
    current_pot, target_pot, current_angle_x10, target_angle_x10 = struct.unpack(
        "<HHhh", data
    )
    return {
        "current_pot": current_pot,
        "target_pot": target_pot,
        "current_angle": current_angle_x10 / 10.0,
        "target_angle": target_angle_x10 / 10.0,
    }


def receive_feedback(bus, drive_status, steering_status):
    while True:
        message = bus.recv(timeout=0.0)

        if message is None:
            break

        if message.arbitration_id == DRIVE_STATUS_ID and len(message.data) == 8:
            drive_status = parse_drive_status(message.data)

        elif message.arbitration_id == STEERING_STATUS_ID and len(message.data) == 8:
            steering_status = parse_steering_status(message.data)

    return drive_status, steering_status


def print_status(rpm, steer, motor_enable, motor_stop_mode, drive_status, steering_status):
    if motor_stop_mode == 1:
        stop_text = "flat"
    elif motor_stop_mode == 2:
        stop_text = "hill"
    else:
        stop_text = "run"

    command_line = (
        f"TX 0x{TX_ID:03X} cmd rpm={rpm:4d} steer={steer:3d} "
        f"enable={motor_enable} stop={stop_text} | "
        "w/s rpm a/d steer e en space flat t hill x stop q quit"
    )

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

    feedback_line = (
        f"RX 0x{DRIVE_STATUS_ID:03X} {drive_text} | "
        f"0x{STEERING_STATUS_ID:03X} {steering_text}"
    )

    width = os.get_terminal_size().columns
    width = max(1, width - 1)

    print(
        "\r\033[K" + command_line[:width] +
        "\n\033[K" + feedback_line[:width] +
        "\033[F",
        end="",
        flush=True
    )


def handle_key(key):
    global target_rpm
    global target_steer
    global enable
    global stop_mode

    if key is None:
        return True

    if key == "w":
        target_rpm += 10
        stop_mode = 0

    elif key == "s":
        target_rpm -= 10
        stop_mode = 0

    elif key == "a":
        target_steer -= 1

    elif key == "d":
        target_steer += 1

    elif key == "e":
        enable = 1

    elif key == " ":
        target_rpm = 0
        stop_mode = 1

    elif key == "t":
        target_rpm = 0
        stop_mode = 2

    elif key == "x":
        target_rpm = 0
        target_steer = 0
        enable = 0
        stop_mode = 0

    elif key == "q":
        target_rpm = 0
        target_steer = 0
        enable = 0
        return False

    target_rpm = clamp(target_rpm, -300, 300)
    target_steer = clamp(target_steer, -45, 45)

    return True


def main():
    global target_rpm
    global target_steer
    global enable
    global stop_mode

    old_terminal = termios.tcgetattr(sys.stdin)

    setup_can_interface(CAN_CHANNEL, CAN_BITRATE)

    bus = can.interface.Bus(
        interface="socketcan",
        channel=CAN_CHANNEL
    )

    drive_status = None
    steering_status = None

    print("w/s   : RPM +10 / -10")
    print("a/d   : steer -1 / +1")
    print("e     : enable")
    print("space : flat stop")
    print("t     : hill stop")
    print("x     : stop")
    print("q     : quit")
    print()

    try:
        tty.setcbreak(sys.stdin.fileno())

        running = True

        while running:
            key = get_key()
            running = handle_key(key)

            send_control(bus)
            drive_status, steering_status = receive_feedback(bus, drive_status, steering_status)
            print_status(target_rpm, target_steer, enable, stop_mode, drive_status, steering_status)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    except can.CanError as error:
        print(f"\nCAN error: {error}")

    finally:
        target_rpm = 0
        enable = 0
        stop_mode = 1

        try:
            send_control(bus)
        except can.CanError:
            pass

        bus.shutdown()
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal
        )

        print("\n\nCAN closed")


if __name__ == "__main__":
    main()
