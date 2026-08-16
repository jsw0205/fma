import struct

import can

TX_ID = 0x200
DRIVE_STATUS_ID = 0x102
STEERING_STATUS_ID = 0x101
# New as of 2026-08-15 (CANoe migration - see waypoint_follower/README_CAN_PROTOCOL.md
# for the full writeup): 0x200/0x101/0x102 above are the original 3 frames the
# firmware already parses/sends - untouched. These two are additions, sent
# alongside them, not replacements:
CONTROL_META_ID = 0x203    # TX (host -> AURIX): what the arbiter decided + why
DIAG_STATUS_ID = 0x104     # RX (AURIX -> host): firmware-side ground truth + health

# CONTROL_META_ID's controller_id byte - coarse "who/why" classification of
# the arbiter_node.py category string for that tick (see arbiter_node.py's
# CATEGORY_TO_CONTROLLER_ID for the exact string->id mapping - kept there
# since the category strings themselves are arbiter business logic, not CAN
# protocol). Small deliberately: fine-grained detail (which avoid sub-state,
# which parking side) stays in arbiter's own CSV log
# (~/.ros/arbiter_logs/arbiter_can_*.csv); this byte is "what would a CANoe
# operator watching the live bus want to see at a glance".
CONTROLLER_SAFE_STOP = 0
CONTROLLER_GPS_FALLBACK = 1
CONTROLLER_CAMERA = 2
CONTROLLER_EVENT_STOP = 3
CONTROLLER_EVENT_GPS_PRIORITY = 4
CONTROLLER_EVENT_GPS_PRIORITY_SLOW = 5
CONTROLLER_EVENT_AVOID_SCAN = 6
CONTROLLER_EVENT_AVOID_ACTIVE = 7
CONTROLLER_EVENT_AVOID_FAILSAFE = 8
CONTROLLER_EVENT_PARKING_LEFT_MAPPING = 9
CONTROLLER_EVENT_PARKING_LEFT_ACTIVE = 10
CONTROLLER_EVENT_PARKING_LEFT_WAIT = 11
CONTROLLER_EVENT_PARKING_RIGHT_MAPPING = 12
CONTROLLER_EVENT_PARKING_RIGHT_ACTIVE = 13
CONTROLLER_EVENT_PARKING_RIGHT_WAIT = 14
CONTROLLER_UNKNOWN = 255

# DIAG_STATUS_ID's fault_flags byte (bitfield) - firmware sets these, host
# only reads. Not yet implemented on the firmware side (separate repo) -
# parse_diag_status() below is ready to receive it once it is.
FAULT_COMM_TIMEOUT = 1 << 0
FAULT_POT_SENSOR = 1 << 1
FAULT_ENCODER_SENSOR = 1 << 2
FAULT_WATCHDOG_TRIP = 1 << 3
FAULT_UNDERVOLTAGE = 1 << 4

# Firmware calibration workaround: Steering.c's STEER_MAX_ANGLE constant is
# still 30.0 even though the vehicle's true max steering lock is ~14.3deg
# (measured 2026-07-26 by fitting a circle to a GPS log driven at full
# lock, radius=2.874m, wheelbase=0.735m -> atan(0.735/2.874)=14.3deg).
# CAN_STEER_SCALE compensates so a commanded true-degree angle actually
# reaches that angle at the wheels. Every sender of CAN steer commands
# (waypoint_follower_node, the camera/GPS arbiter, ...) must go through
# send_control_true_deg (not send_control directly with a raw angle) so
# there's exactly one place this correction lives. Remove once
# STEER_MAX_ANGLE in Steering.c is corrected to the true value (set
# CAN_STEER_SCALE = 1.0, or delete the workaround entirely).
FIRMWARE_STEER_MAX_ANGLE_DEG = 30.0
TRUE_STEER_MAX_ANGLE_DEG = 14.3
CAN_STEER_SCALE = FIRMWARE_STEER_MAX_ANGLE_DEG / TRUE_STEER_MAX_ANGLE_DEG


def open_bus(channel="can0"):
    return can.interface.Bus(interface="socketcan", channel=channel)


def make_control_data(rpm, steer, motor_enable, stop_mode):
    # byte0-1: rpm, byte2-3: steer, byte4: enable,
    # byte5: stop mode (0=normal, 1=flat, 2=hill), byte6-7: unused
    return struct.pack(
        "<hhBBH",
        int(rpm),
        int(steer),
        int(motor_enable),
        int(stop_mode),
        0,
    )


def send_control(bus, rpm, steer, motor_enable, stop_mode):
    data = make_control_data(rpm, steer, motor_enable, stop_mode)
    message = can.Message(arbitration_id=TX_ID, data=data, is_extended_id=False)
    bus.send(message)


def send_control_true_deg(bus, rpm, true_steer_deg, motor_enable, stop_mode):
    """Like send_control, but takes steer in true physical degrees and
    applies CAN_STEER_SCALE before sending - use this instead of
    send_control for any steer value that isn't already firmware-scale."""
    send_control(bus, rpm, true_steer_deg * CAN_STEER_SCALE, motor_enable, stop_mode)


def parse_drive_status(data):
    encoder_count, rpm_x10, pwm_duty, target_rpm = struct.unpack("<hhhh", data)
    return {
        "encoder_count": encoder_count,
        "rpm": rpm_x10 / 10.0,
        "pwm_duty": pwm_duty,
        "target_rpm": target_rpm,
    }


def parse_steering_status(data):
    current_pot, target_pot, current_angle_x10, target_angle_x10 = struct.unpack(
        "<HHhh", data
    )
    return {
        "current_pot": current_pot,
        "target_pot": target_pot,
        "current_angle": current_angle_x10 / 10.0,
        "target_angle": target_angle_x10 / 10.0,
    }


def poll_diag_status(bus, diag_status):
    """Same non-blocking-drain pattern as poll_feedback() below, kept
    separate rather than folded into it so callers that don't care about
    DIAG_STATUS (waypoint_follower_node.py, the only current poll_feedback
    caller) don't need a signature change. Returns the latest parsed
    DIAG_STATUS dict, or `diag_status` unchanged if none arrived this
    call."""
    while True:
        message = bus.recv(timeout=0.0)
        if message is None:
            break
        if message.arbitration_id == DIAG_STATUS_ID and len(message.data) == 8:
            diag_status = parse_diag_status(message.data)
    return diag_status


def poll_feedback(bus, drive_status, steering_status):
    while True:
        message = bus.recv(timeout=0.0)

        if message is None:
            break

        if message.arbitration_id == DRIVE_STATUS_ID and len(message.data) == 8:
            drive_status = parse_drive_status(message.data)

        elif message.arbitration_id == STEERING_STATUS_ID and len(message.data) == 8:
            steering_status = parse_steering_status(message.data)

    return drive_status, steering_status


def make_control_meta_data(target_rpm, target_steer, stop_mode, controller_id, seq):
    # byte0-1: target_rpm, byte2-3: target_steer (same raw firmware-scale as
    # make_control_data's steer field - not re-scaled here), byte4:
    # stop_mode, byte5: controller_id (see CONTROLLER_* constants above),
    # byte6: seq (wraps 0-255, echoed back in DIAG_STATUS.rx_seq_echo for
    # round-trip verification), byte7: unused.
    return struct.pack(
        "<hhBBBB",
        int(target_rpm),
        int(target_steer),
        int(stop_mode) & 0xFF,
        int(controller_id) & 0xFF,
        int(seq) & 0xFF,
        0,
    )


def send_control_meta(bus, target_rpm, target_steer, stop_mode, controller_id, seq):
    """Logging/CANoe-visibility frame - NOT what actually commands the
    vehicle (that's still make_control_data/TX_ID). Send this right
    alongside (same tick as) the real control frame, with the same values,
    plus controller_id/seq which the real frame has no room for."""
    data = make_control_meta_data(target_rpm, target_steer, stop_mode, controller_id, seq)
    message = can.Message(arbitration_id=CONTROL_META_ID, data=data, is_extended_id=False)
    bus.send(message)


def parse_diag_status(data):
    # byte0: applied_stop_mode (firmware's *actual* internal StopHoldEnable-
    # derived mode, not just an echo of what was requested - the whole
    # point of this frame is catching cases where they differ, see the
    # 2026-08 judder investigation this was designed after), byte1:
    # fault_flags (FAULT_* bitfield), byte2-3: steer_pwm_duty, byte4-5:
    # supply_voltage_mV, byte6: rx_seq_echo, byte7: unused.
    # NOT YET SENT BY FIRMWARE - this parser is ready for when it is; see
    # README_CAN_PROTOCOL.md.
    (applied_stop_mode, fault_flags, steer_pwm_duty, supply_voltage_mV,
     rx_seq_echo, _reserved) = struct.unpack("<BBhHBB", data)
    return {
        "applied_stop_mode": applied_stop_mode,
        "fault_flags": fault_flags,
        "steer_pwm_duty": steer_pwm_duty,
        "supply_voltage_mV": supply_voltage_mV,
        "rx_seq_echo": rx_seq_echo,
    }
