from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional

import serial

PACKET_HEADER = [0xFF, 0xFF]
ALL_SERVO_IDS = 0xFE


# packet structure
# Packet ID: 0 ~ 253
# Length: num parameters + 2
# Checksum: ~(ID + Length + Instruction + Param1 + ... + Param N)

# instruction packet:
# Header 1 | Header 2 | Packet ID | Length | Instruction | Param 1 ... Param N | Checksum
# 0xFF     | 0xFF     | 0x00      | 0x00   | 0x00        | 0x00    ... 0x00    | 0x00

# return packet:
# Header 1 | Header 2 | Packet ID | Length | Error | Param 1 ... Param N | Checksum
# 0xFF     | 0xFF     | 0x00      | 0x00   | 0x00  | 0x00    ... 0x00    | 0x00


class Instruction(IntEnum):
    PING = 0x01
    READ_DATA = 0x02
    WRITE_DATA = 0x03
    REGWRITE_DATA = 0x04
    ACTION = 0x05
    SYNCREAD_DATA = 0x82
    SYNCWRITE_DATA = 0x83
    RESET = 0x06


class Register(IntEnum):
    ID = 0x05
    BAUDRATE = 0x06
    RETURN_DELAY = 0x07
    RESPONSE_STATUS_LEVEL = 0x08
    MINIMUM_ANGLE = 0x09  # 2 bytes
    MAXIMUM_ANGLE = 0x0B  # 2 bytes
    MAXIMUM_TEMPERATURE = 0x0D
    MAXIMUM_INPUT_VOLTAGE = 0x0E
    MAXIMUM_TORQUE = 0x10  # 2 bytes
    PHASE = 0x12
    UNLOADING_CONDITIONS = 0x13
    LED_ALARM_CONDITIONS = 0x14
    POSITION_LOOP_P = 0x15
    POSITION_LOOP_D = 0x16
    POSITION_LOOP_I = 0x17
    MINIMUM_STARTING_FORCE = 0x18  # 2 bytes
    CLOCKWISE_INSENSITIVE_ZONE = 0x1A
    ANTICLOCKWISE_INSENSITIVE_ZONE = 0x1B
    PROTECTION_CURRENT = 0x1C  # 2 bytes
    ANGLE_RESOLUTION = 0x1E
    POSITION_CORRECTION = 0x1F  # 2 bytes
    OPERATION_MODE = 0x21
    PROTECTION_TORQUE = 0x22
    PROTECTION_TIME = 0x23
    OVERLOAD_TORQUE = 0x24
    SPEED_CLOSED_LOOP_P = 0x25
    OVERCURRENT_PROTECTION_TIME = 0x26
    SPEED_CLOSED_LOOP_I = 0x27
    TORQUE_SWITCH = 0x28
    ACCELERATION = 0x29
    TARGET_LOCATION = 0x2A  # 2 bytes
    OPERATION_TIME = 0x2C  # 2 bytes
    OPERATION_SPEED = 0x2E  # 2 bytes
    TORQUE_LIMIT = 0x30  # 2 bytes
    LOCK_FLAG = 0x37
    CURRENT_LOCATION = 0x38  # 2 bytes
    CURRENT_SPEED = 0x3A  # 2 bytes
    CURRENT_LOAD = 0x3C  # 2 bytes
    CURRENT_VOLTAGE = 0x3E
    CURRENT_TEMPERATURE = 0x3F
    ASYNC_WRITE_FLAG = 0x40
    SERVO_STATUS = 0x41
    MOVE_FLAG = 0x42
    CURRENT_CURRENT = 0x45  # 2 bytes


@dataclass
class ServoInstruction:
    # header
    # servo ID
    # data length
    # instruction
    # parameters[]
    # checksum

    raw: bytes

    @classmethod
    def build_instruction(
        cls,
        servo_id: int,
        instruction: Instruction,
        parameters: List[int],
    ) -> ServoInstruction:
        data_length = len(parameters) + 2
        raw_data = [servo_id, data_length, instruction.value] + parameters
        checksum = ~sum(raw_data) & 0xFF

        return cls(bytes(PACKET_HEADER + raw_data + [checksum]))


@dataclass(frozen=True)
class ServoResponse:
    # header
    # servo ID
    # data length
    # error
    # parameters[]
    # checksum

    raw: bytes


class ServoBus:
    def __init__(self, port: str, baud: int):
        self._serial_connection = serial.Serial(port, baudrate=baud, timeout=0.1)
        self._serial_connection.reset_input_buffer()

    def ping_servo(self, id: int) -> Optional[bytes]:
        instruction = ServoInstruction.build_instruction(id, Instruction.PING, [])
        self._serial_connection.write(instruction.raw)

        time.sleep(0.005)

        return self._serial_connection.read_all()

    def _set_write_lock(self, id: int, enabled: bool) -> bytes:
        instruction = ServoInstruction.build_instruction(
            id, Instruction.WRITE_DATA, [Register.LOCK_FLAG, 0x01 if enabled else 0x00]
        )
        self._serial_connection.write(instruction.raw)
        time.sleep(0.001)

        return self._serial_connection.read(8)

    def set_servo_id(self, current_id: int, new_id: int) -> bytes:
        instruction = ServoInstruction.build_instruction(
            current_id, Instruction.WRITE_DATA, [Register.ID, new_id]
        )
        self._set_write_lock(current_id, enabled=False)
        self._serial_connection.write(instruction.raw)
        self._set_write_lock(new_id, enabled=True)

        return self._serial_connection.read(6)

    def get_positions(self, id: int) -> Optional[bytes]:
        instruction = ServoInstruction.build_instruction(
            id, Instruction.READ_DATA, [Register.CURRENT_LOCATION, 0x02]
        )
        self._serial_connection.write(instruction.raw)

        time.sleep(0.001)

        return self._serial_connection.read_all()

    def set_min_angle(self, id: int, min_angle: int) -> Optional[bytes]:
        min_angle_bytes = min_angle.to_bytes(2, byteorder="little")
        instruction = ServoInstruction.build_instruction(
            id,
            Instruction.WRITE_DATA,
            [Register.MINIMUM_ANGLE, min_angle_bytes[0], min_angle_bytes[1]],
        )
        self._set_write_lock(id, enabled=False)
        self._serial_connection.write(instruction.raw)
        self._set_write_lock(id, enabled=True)
        self._serial_connection.read_all()

        read_bound = ServoInstruction.build_instruction(
            id, Instruction.READ_DATA, [Register.MINIMUM_ANGLE, 0x02]
        )
        self._serial_connection.write(read_bound.raw)
        time.sleep(0.001)

        return self._serial_connection.read_all()

    def set_max_angle(self, id: int, max_angle: int) -> Optional[bytes]:
        max_angle_bytes = max_angle.to_bytes(2, byteorder="little")
        instruction = ServoInstruction.build_instruction(
            id,
            Instruction.WRITE_DATA,
            [Register.MAXIMUM_ANGLE, max_angle_bytes[0], max_angle_bytes[1]],
        )
        self._set_write_lock(id, enabled=False)
        self._serial_connection.write(instruction.raw)
        self._set_write_lock(id, enabled=True)
        self._serial_connection.read_all()

        read_bound = ServoInstruction.build_instruction(
            id, Instruction.READ_DATA, [Register.MAXIMUM_ANGLE, 0x02]
        )
        self._serial_connection.write(read_bound.raw)
        time.sleep(0.001)

        return self._serial_connection.read_all()

    def write_position(self, id: int, position: int) -> bytes:
        assert 0 <= position <= 4095
        position_bytes = position.to_bytes(2, byteorder="little")
        instruction = ServoInstruction.build_instruction(
            id,
            Instruction.WRITE_DATA,
            [Register.TARGET_LOCATION, position_bytes[0], position_bytes[1]],
        )
        self._serial_connection.write(instruction.raw)

        return self._serial_connection.read(6)

    def reset_servo(self, id: int) -> bytes:
        instruction = ServoInstruction.build_instruction(id, Instruction.RESET, [])
        self._serial_connection.write(instruction.raw)

        return self._serial_connection.read_all()


if __name__ == "__main__":
    servo_bus = ServoBus("/dev/ttyACM0", 1_000_000)

    print("PINGING ALL SERVOS INDIVIDUALLY")
    for i in range(7):
        response = servo_bus.ping_servo(i + 1)
        print(f"ping response: {[hex(byte) for byte in response] if response else response}")

    print("PINGING ALL SERVOS SIMULTANEOUSLY")
    response = servo_bus.ping_servo(ALL_SERVO_IDS)
    print(f"ping response: {[hex(byte) for byte in response] if response else response}")

    print("ALL SERVOS POSITIONS")
    for i in range(7):
        response = servo_bus.get_positions(i + 1)
        if response:
            position = int.from_bytes([response[6], response[5]])
            print(f"ping response: {[hex(byte) for byte in response]}")
            print(f"servo {i + 1} position: {position}")

    # 0-90deg
    # servo_id = 7
    # servo_bus.set_min_angle(servo_id, 3072)
    # servo_bus.set_max_angle(servo_id, 4095)

    # 0-180deg
    # servo_id = 5
    # servo_bus.set_min_angle(servo_id, 2048)
    # servo_bus.set_max_angle(servo_id, 4095)

    # 180-360deg
    # servo_id = 3
    # servo_bus.set_min_angle(servo_id, 0)
    # servo_bus.set_max_angle(servo_id, 2048)

    # 0-180deg
    # servo_id = 2
    # servo_bus.set_min_angle(servo_id, 2048)
    # servo_bus.set_max_angle(servo_id, 4095)

    # 0-180deg
    # servo_id = 1
    # print(servo_bus.set_min_angle(servo_id, 2048))
    # print(servo_bus.set_max_angle(servo_id, 4095))

    # servo_id = 1
    # max = 4095
    # min = 0
    # servo_bus.write_position(servo_id, min)
    # time.sleep(4)
    # servo_bus.write_position(servo_id, max)
    # time.sleep(4)
    # servo_bus.write_position(servo_id, min)
    # time.sleep(4)
