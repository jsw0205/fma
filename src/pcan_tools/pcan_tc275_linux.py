import can
import struct
import sys
import select
import termios
import tty
import time


CAN_CHANNEL = "can0"
CAN_BITRATE = 500000

TX_ID = 0x200
RX_ID = 0x210

target_rpm = 0
target_steer = 0
enable = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def make_control_data(rpm, steer, motor_enable):
    return struct.pack(
        "<hhB3x",
        int(rpm),
        int(steer),
        int(motor_enable)
    )


def send_control(bus):
    data = make_control_data(
        target_rpm,
        target_steer,
        enable
    )

    message = can.Message(
        arbitration_id=TX_ID,
        data=data,
        is_extended_id=False
    )

    bus.send(message)

    print(
        f"TX 0x{TX_ID:03X}: "
        + " ".join(f"{byte:02X}" for byte in data)
    )


def receive_encoder(bus):
    while True:
        message = bus.recv(timeout=0.0)

        if message is None:
            break

        if message.arbitration_id != RX_ID:
            continue

        if len(message.data) < 8:
            continue

        count_a, count_b = struct.unpack(
            "<ii",
            bytes(message.data[:8])
        )

        print(
            f"RX 0x{RX_ID:03X}: "
            + " ".join(f"{byte:02X}" for byte in message.data)
        )
        print(f"count_a={count_a}, count_b={count_b}")


class RawTerminal:
    """stdin을 cbreak 모드로 바꿔 msvcrt.kbhit()/getch()처럼
    Enter 없이 한 글자씩 non-blocking으로 읽을 수 있게 한다."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None

    ch = sys.stdin.read(1)

    if ch == "\x1b":
        for _ in range(2):
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                sys.stdin.read(1)
        return None

    return ch


def handle_keyboard():
    global target_rpm
    global target_steer
    global enable

    key = read_key()

    if key is None:
        return True

    if key == "w":
        target_rpm += 10

    elif key == "s":
        target_rpm -= 10

    elif key == "a":
        target_steer -= 5

    elif key == "d":
        target_steer += 5

    elif key == "e":
        enable = 1

    elif key == "x":
        target_rpm = 0
        target_steer = 0
        enable = 0

    elif key == "q":
        target_rpm = 0
        target_steer = 0
        enable = 0
        return False

    else:
        return True

    target_rpm = clamp(target_rpm, -300, 300)
    target_steer = clamp(target_steer, -45, 45)

    print(
        f"rpm={target_rpm}, "
        f"steer={target_steer}, "
        f"enable={enable}"
    )

    return True


def open_bus(channel, bitrate):
    try:
        return can.interface.Bus(interface="socketcan", channel=channel)
    except OSError as exc:
        raise SystemExit(
            f"'{channel}' 인터페이스를 열 수 없습니다 ({exc}).\n"
            f"먼저 아래 명령으로 인터페이스를 올려주세요:\n"
            f"  sudo ip link set {channel} up type can bitrate {bitrate}\n"
        ) from exc


def main():
    global target_rpm
    global target_steer
    global enable

    bus = open_bus(CAN_CHANNEL, CAN_BITRATE)

    print("SocketCAN connected")
    print("w/s : RPM 증가/감소")
    print("a/d : 조향각 감소/증가")
    print("e   : enable")
    print("x   : 정지")
    print("q   : 종료")

    running = True

    try:
        with RawTerminal():
            while running:
                running = handle_keyboard()

                send_control(bus)
                receive_encoder(bus)

                time.sleep(0.05)

    except can.CanError as error:
        print(f"CAN error: {error}")

    finally:
        target_rpm = 0
        target_steer = 0
        enable = 0

        try:
            send_control(bus)
        except can.CanError:
            pass

        bus.shutdown()
        print("PCAN closed")


if __name__ == "__main__":
    main()
