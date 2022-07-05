#ifndef _SERIAL_COMM_MBED_H_
#define _SERIAL_COMM_MBED_H_

#include "mbed.h"

#define READ_BUFFER_SIZE 256

enum MSG_TYPE{
    MCU_TO_PC = 1,
    PC_TO_MCU = 2
};

class SerialCommunicatorMbed{
private:
    // packet structure
    // $ L d[0] d[1] ... d[L-1] CRC %

    // $   : start bytes
    // L   : length
    // CRC : checksum.
    // %   : end bytes
    // total overhead per meesage: 4 bytes
    unsigned char STX_     = '$';
    unsigned char ETX_     = '%';
    unsigned char MSG_LEN_ = 0;

    BufferedSerial pc;

    // Related to serial read
    int stack_len_ = 0;
    unsigned char serial_stack_[READ_BUFFER_SIZE];

    unsigned char read_buf_[READ_BUFFER_SIZE];
    unsigned char send_buf_[READ_BUFFER_SIZE];

    uint64_t seq = 0;

    // Initialise the digital pin LED1 as an output
    DigitalOut led_recv;

public:
    SerialCommunicatorMbed(int baud_rate);
    
    void send_withChecksum(const uint8_t* data, int len);
    void send_withoutChecksum(const char* data, int len);
    int read_withChecksum(uint8_t* msg);

    bool readable();
    bool writable();

private:
    uint8_t stringChecksum(const uint8_t* s, int idx_start, int idx_end);
};

#endif