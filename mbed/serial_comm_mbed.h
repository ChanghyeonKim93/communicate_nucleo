#ifndef _SERIAL_COMM_MBED_H_
#define _SERIAL_COMM_MBED_H_

#include "mbed.h"

#define BUF_SIZE 1024

enum MSG_TYPE{
    MCU_TO_PC = 1,
    PC_TO_MCU = 2
};

class SerialCommunicatorMbed{
private:
    BufferedSerial buffered_serial_;

    char STX_     = '$';
    char ETX_     = '%';
    char CTX_     = '*';

    bool flagSTXFound;
    bool flagCTXFound;

    // Related to serial read
    char buf_packet_[BUF_SIZE];
    uint32_t idx_packet_;

    uint32_t cnt_read_success_;
    uint32_t cnt_read_fail_;

    char buf_read_[BUF_SIZE];
    char buf_send_[BUF_SIZE];

private:
    char message_read_[BUF_SIZE];
    uint32_t len_message_read_;

    char message_send_[BUF_SIZE];
    uint32_t len_message_send_;

    uint32_t seq_recv_ = 0;
    uint32_t seq_send_ = 0;

    uint32_t seq = 0;

    DigitalOut recv_led_;
    DigitalOut recv_signal_;

public:
    SerialCommunicatorMbed(int baud_rate);
    
    void send_withChecksum(const char* data, int len);
    void send_withoutChecksum(const char* data, int len);

    bool tryToReadSerialBuffer();

    bool readable();
    bool writable();

    int getReceivedMessage(char* msg);

private:
    char stringChecksum(const char* s, int idx_start, int idx_end);
};

#endif