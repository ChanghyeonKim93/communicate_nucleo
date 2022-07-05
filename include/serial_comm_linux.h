#ifndef _SERIAL_COMM_H_
#define _SERIAL_COMM_H_

#define BUF_SIZE 1024

#include <iostream>
#include <time.h>
#include <thread>
#include <chrono>
#include <future>
#include <string>
#include <cstring>
#include <iomanip>
#include <numeric>

// serial
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/time.h>

#include <sys/ioctl.h>
#include <linux/serial.h>

#include "union_struct.h"

using namespace std::chrono_literals;

class SerialCommunicatorLinux {
public:
    SerialCommunicatorLinux(const std::string& portname, const int& baud_rate);
    ~SerialCommunicatorLinux();

    bool isReceiveReady();
    int  getMessage(char* buf);
    bool sendMessage(char* buf, int len);

private:
    void runThreadRX();
    void runThreadTX();
    void processRX(std::shared_future<void> terminate_signal);
    void processTX(std::shared_future<void> terminate_signal);

private:
    void setBaudRate(int baud_rate);
    void setPortName(std::string portname);
    void openSerialPort();
    void closeSerialPort();

private:
    void send_withChecksum(const char* data, int len);

    void copyReadToRecvBuffer(int len);

private:
    char stringChecksum(const char* s, int idx_start, int idx_end);


private:
    std::string portname_;
    int BAUD_RATE_;

    int fd_; // file descriptor...
    
    struct termios term_setting_;

    struct pollfd poll_events_;
    int poll_state_;
    char buf_read_[BUF_SIZE];
    char buf_recv_[BUF_SIZE];
    char buf_send_[BUF_SIZE];
    
    char STX_;
    char CTX_;
    char ETX_;

    int stack_len_;
    char serial_stack_[BUF_SIZE];

private:
    std::shared_future<void> terminate_future_;
    std::promise<void> terminate_promise_;


private:
    uint64_t seq_recv_;
    uint32_t len_recv_;

    uint64_t seq_send_;
    uint32_t len_send_;

private:
    std::thread thread_rx_;
    std::thread thread_tx_;

    std::shared_ptr<std::mutex> mutex_rx_;
    std::shared_ptr<std::mutex> mutex_tx_;
};


#endif