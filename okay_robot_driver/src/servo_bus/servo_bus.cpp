#include <numeric>
#include <thread>

#include "okay_robot_driver/servo_bus/servo_bus.hpp"

/**
 * Builds and formats a packet to send to the servo bus
 *
 * @param id [uint8_t] the id of the servo to receive the packet
 * @param instruction [uint8_t] the instruction code to send
 * @param data [std::vector<uint8_t>] the data to send with the packet
 * @return A properly formatted array of bytes [std::vector<uint8_t>]
 */
std::vector<uint8_t> build_packet(uint8_t id, uint8_t instruction, std::vector<uint8_t> data)
{
    /*
    packet structure:
        Packet ID: 0 ~ 253
        Length: num parameters + 2
        Checksum: ~(ID + Length + Instruction + Param1 + ... + Param N)

    instruction packet:
        Header 1 | Header 2 | Packet ID | Length | Instruction | Param 1 ... Param N | Checksum
        0xFF     | 0xFF     | 0x00      | 0x00   | 0x00        | 0x00    ... 0x00    | 0x00
    */

    uint8_t data_length = data.size() + 2;
    std::vector<uint8_t> contents
        = { PACKET_HEADER_1, PACKET_HEADER_2, id, data_length, instruction };
    contents.insert(contents.end(), data.begin(), data.end());

    uint8_t checksum = ~std::accumulate(contents.begin() + 2, contents.end(), 0) & 0xFF;
    contents.push_back(checksum);

    return contents;
}

void MessageParser::step(uint8_t byte)
{
    /*
    return packet:
        Header 1 | Header 2 | Packet ID | Length | Error | Param 1 ... Param N | Checksum
        0xFF     | 0xFF     | 0x00      | 0x00   | 0x00  | 0x00    ... 0x00    | 0x00
    */

    switch (this->current_state_) {
    case MessageParseState::HEADER1:
        if (byte == PACKET_HEADER_1) {
            this->message_buffer_.push_back(byte);
            this->current_state_ = MessageParseState::HEADER2;
        } else {
            this->reset();
        }
        break;

    case MessageParseState::HEADER2:
        if (byte == PACKET_HEADER_2) {
            this->message_buffer_.push_back(byte);
            this->current_state_ = MessageParseState::SERVO_ID;
        } else {
            this->reset();
        }
        break;

    case MessageParseState::SERVO_ID:
        this->message_buffer_.push_back(byte);
        this->current_state_ = MessageParseState::LENGTH;
        break;

    case MessageParseState::LENGTH:
        this->message_buffer_.push_back(byte);
        this->data_length_ = int(byte);
        this->current_state_ = MessageParseState::ERROR;
        break;

    case MessageParseState::ERROR:
        this->message_buffer_.push_back(byte);

        if (int(this->message_buffer_.size()) < this->data_length_ + 3)
            this->current_state_ = MessageParseState::DATA;
        else
            this->current_state_ = MessageParseState::CHECKSUM;
        break;

    case MessageParseState::DATA:
        this->message_buffer_.push_back(byte);

        if (int(this->message_buffer_.size()) == this->data_length_ + 3)
            this->current_state_ = MessageParseState::CHECKSUM;
        break;

    case MessageParseState::CHECKSUM:
        // TODO: verify that checksum matches the rest of the packet?
        this->message_buffer_.push_back(byte);
        this->current_state_ = MessageParseState::COMPLETE;
        break;

    case MessageParseState::COMPLETE:
        break;
    }
}

std::vector<uint8_t> MessageParser::get_message() { return this->message_buffer_; }

bool MessageParser::is_complete() { return this->current_state_ == MessageParseState::COMPLETE; }

void MessageParser::reset()
{
    this->message_buffer_.clear();
    this->current_state_ = MessageParseState::HEADER1;
}

ServoBus::~ServoBus()
{
    try {
        this->serial_.Close();
    } catch (const LibSerial::NotOpen&) {
        std::printf("cannot close serial bus, serial bus not open\n");
    }
};

/**
 * Opens the serial port and sets the baud rate. Will return if it fails to open the port
 *
 * @param port [string] the port to open
 * @param baud [LibSerial::BaudRate] the baud rate to set for the port
 * @return No return
 */
void ServoBus::init(std::string port, LibSerial::BaudRate baud)
{
    try {
        this->serial_.Open(port);
        this->serial_.SetBaudRate(baud);

        this->port_ = port;
        this->baud_ = baud;
    }

    catch (const LibSerial::OpenFailed&) {
        std::printf("cannot open serial connection on port %s\n", port.c_str());
        return;
    }
}

/**
 * Writes data to the serial port
 *
 * @param id [uint_t] the id of the servo to receive the message
 * @param data [std::vector<uint8_t>] the bytes to send to the servo
 * @return No return
 */
void ServoBus::write_data(uint8_t id, std::vector<uint8_t> data)
{
    std::vector<uint8_t> message = build_packet(id, ServoInstruction::WRITE_DATA, data);
    this->serial_.Write(message);
}

/**
 *
 */
void ServoBus::sync_write_data(std::vector<uint8_t> data)
{
    std::vector<uint8_t> message = build_packet(0xFE, ServoInstruction::SYNCWRITE_DATA, data);
    this->serial_.Write(message);
}

/**
 * Writes data to a buffer on the serial port to be executed by an Action command at a later time
 *
 * @param id [uint_t] the id of the servo to receive the message
 * @param data [std::vector<uint8_t>] the bytes to send to the servo
 * @return No return
 */
void ServoBus::reg_write_data(uint8_t id, std::vector<uint8_t> data)
{
    std::vector<uint8_t> message = build_packet(id, ServoInstruction::REG_WRITE_DATA, data);
    this->serial_.Write(message);
}

/**
 * Executes the data written to buffer by one or several `reg_write_data` commands
 *
 * @return No return
 */
void ServoBus::execute_reg_write()
{
    std::vector<uint8_t> message
        = build_packet(ALL_SERVOS, ServoInstruction::ACTION, std::vector<uint8_t>());
    this->serial_.Write(message);
}

/**
 * Reads data from the register of the specified servo
 *
 * @param id [uint_t] the id of the servo to request data from
 * @param data [std::vector<uint8_t>] the register and length of data to request
 * @return The raw message containing the requested data
 */
std::vector<uint8_t> ServoBus::read_data(uint8_t id, std::vector<uint8_t> data)
{
    // start by flushing the input buffer
    this->serial_.FlushInputBuffer();

    std::vector<uint8_t> message = build_packet(id, ServoInstruction::READ_DATA, data);
    this->serial_.Write(message);

    return this->read_buffer();
}

/**
 *
 */
void ServoBus::sync_read_data(std::vector<uint8_t> data)
{
    this->serial_.FlushInputBuffer();

    std::vector<uint8_t> message = build_packet(0xFE, ServoInstruction::SYNCREAD_DATA, data);
    this->serial_.Write(message);
}

/**
 * Reads the next message from the serial port
 *
 * @return An array of bytes [std::vector<uint8_t>] containing the raw message data
 */
std::vector<uint8_t> ServoBus::read_buffer()
{
    uint8_t read_byte;
    while (true) {
        try {
            this->serial_.ReadByte(read_byte, 10);
        } catch (LibSerial::ReadTimeout&) {
            printf("timed out reading byte\n");
            return std::vector<uint8_t>();
        }

        this->message_parser_.step(read_byte);

        if (this->message_parser_.is_complete()) {
            auto parsed_message = this->message_parser_.get_message();
            this->message_parser_.reset();
            return parsed_message;
        }
    }
}

/**
 * Sends a ping message to the specified servo
 *
 * @param id [uint8_t] the id of the servo to ping
 * @return No return
 */
void ServoBus::ping(uint8_t id)
{
    std::vector<uint8_t> contents
        = { PACKET_HEADER_1, PACKET_HEADER_2, id, uint8_t(0x02), ServoInstruction::PING };
    uint8_t checksum = ~std::accumulate(contents.begin() + 2, contents.end(), 0) & 0xFF;
    contents.push_back(checksum);

    this->serial_.Write(contents);
}