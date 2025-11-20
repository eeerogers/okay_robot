#pragma once

#include <libserial/SerialPort.h>
#include <string>

#define PACKET_HEADER_1 uint8_t(0xFF)
#define PACKET_HEADER_2 uint8_t(0xFF)
#define ALL_SERVOS uint8_t(0xFE)

enum ServoInstruction {
    PING = 0x01,
    READ_DATA = 0x02,
    WRITE_DATA = 0x03,
    REG_WRITE_DATA = 0x04,
    ACTION = 0x05,
    RESET = 0x06,
    SYNCREAD_DATA = 0x82,
    SYNCWRITE_DATA = 0x83,
};

enum ServoRegister {
    ID = 0x05,
    BAUDRATE = 0x06,
    RETURN_DELAY = 0x07,
    RESPONSE_STATUS_LEVEL = 0x08,
    MINIMUM_ANGLE = 0x09, // 2 bytes
    MAXIMUM_ANGLE = 0x0B, // 2 bytes
    MAXIMUM_TEMPERATURE = 0x0D,
    MAXIMUM_INPUT_VOLTAGE = 0x0E,
    MAXIMUM_TORQUE = 0x10, // 2 bytes
    PHASE = 0x12,
    UNLOADING_CONDITIONS = 0x13,
    LED_ALARM_CONDITIONS = 0x14,
    POSITION_LOOP_P = 0x15,
    POSITION_LOOP_D = 0x16,
    POSITION_LOOP_I = 0x17,
    MINIMUM_STARTING_FORCE = 0x18, // 2 bytes
    CLOCKWISE_INSENSITIVE_ZONE = 0x1A,
    ANTICLOCKWISE_INSENSITIVE_ZONE = 0x1B,
    PROTECTION_CURRENT = 0x1C, // 2 bytes
    ANGLE_RESOLUTION = 0x1E,
    POSITION_CORRECTION = 0x1F, // 2 bytes
    OPERATION_MODE = 0x21,
    PROTECTION_TORQUE = 0x22,
    PROTECTION_TIME = 0x23,
    OVERLOAD_TORQUE = 0x24,
    SPEED_CLOSED_LOOP_P = 0x25,
    OVERCURRENT_PROTECTION_TIME = 0x26,
    SPEED_CLOSED_LOOP_I = 0x27,
    TORQUE_SWITCH = 0x28,
    OPERATION_ACCELERATION = 0x29,
    TARGET_POSITION = 0x2A, // 2 bytes
    OPERATION_TIME = 0x2C, // 2 bytes
    OPERATION_SPEED = 0x2E, // 2 bytes
    TORQUE_LIMIT = 0x30, // 2 bytes
    LOCK_FLAG = 0x37,
    CURRENT_POSITION = 0x38, // 2 bytes
    CURRENT_SPEED = 0x3A, // 2 bytes
    CURRENT_LOAD = 0x3C, // 2 bytes
    CURRENT_VOLTAGE = 0x3E,
    CURRENT_TEMPERATURE = 0x3F,
    ASYNC_WRITE_FLAG = 0x40,
    SERVO_STATUS = 0x41,
    MOVE_FLAG = 0x42,
    CURRENT_CURRENT = 0x45, // 2 bytes
};

enum ErrorBits {
    INCORRECT_INPUT_VOLTAGE = 0b00000001,
    OUTSIDE_ANGLE_LIMIT = 0b00000010,
    OVERHEAT = 0b00000100,
    OUT_OF_RANGE = 0b00001000,
    INVALID_CHECKSUM = 0b00010000,
    OVERTORQUE = 0b00100000,
    INVALID_INSTRUCTION = 0b01000000,
};

enum MessageParseState {
    HEADER1,
    HEADER2,
    SERVO_ID,
    LENGTH,
    ERROR,
    DATA,
    CHECKSUM,
    COMPLETE,
};

std::vector<uint8_t> build_packet(uint8_t id, uint8_t instruction, std::vector<uint8_t> data);

class MessageParser {
public:
    void step(uint8_t message_byte);
    std::vector<uint8_t> get_message();
    bool is_complete();
    void reset();

private:
    std::vector<uint8_t> message_buffer_;
    MessageParseState current_state_ = MessageParseState::HEADER1;
    int data_length_ = 0;
};

class ServoBus {
public:
    ~ServoBus();

    void init(std::string port, LibSerial::BaudRate baud);

    void write_data(uint8_t id, std::vector<uint8_t> data);
    void sync_write_data(std::vector<uint8_t> data);
    void reg_write_data(uint8_t id, std::vector<uint8_t> data);
    void execute_reg_write();
    std::vector<uint8_t> read_data(uint8_t id, std::vector<uint8_t> data);
    void sync_read_data(std::vector<uint8_t> data);
    std::vector<uint8_t> read_buffer();
    void ping(uint8_t id);

private:
    LibSerial::SerialPort serial_;
    std::string port_;
    LibSerial::BaudRate baud_;

    MessageParser message_parser_;
};