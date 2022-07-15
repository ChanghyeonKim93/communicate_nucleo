#ifndef _SERIAL_COMMUNICATOR_H_
#define _SERIAL_COMMUNICATOR_H_

#define BUF_SIZE 1024

// C++
#include <iostream>
#include <string>
#include <cstring>
#include <numeric>
#include <exception>

// Thread
#include <thread>
#include <chrono>
#include <future>

// Boost serial
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "union_struct.h"
#include "crc16.h" // CRC16 checksum test

#define DLE 0x10
#define STX 0x02
#define ETX 0x03

using namespace std::chrono_literals;

class SerialCommunicator {
public:
    SerialCommunicator(const std::string& portname, const int& baud_rate);
    ~SerialCommunicator();

    // bool isReceiveReady();
    // int  getMessage(char* buf);
    // bool sendMessage(char* buf, int len);

private:
    void runThreadRX();
    void runThreadTX();
    void processRX(std::shared_future<void> terminate_signal);
    void processTX(std::shared_future<void> terminate_signal);

private:
    void setPortName(std::string portname);
    void setBaudRate(int baudrate);
    void checkSupportedBaudRate(int baud_rate);

// Port settings
private:
    void openSerialPort();
    void closeSerialPort();

private:
    void send_withChecksum(const unsigned char* data, int len);

    void copyReadToRecvBuffer(int len);

private:
    unsigned short stringChecksumCRC16_CCITT(const char* s, int idx_start, int idx_end);


// Serial port related (boost::asio::serial )
private:
    std::string portname_;
    int         baud_rate_;

    boost::asio::serial_port*   serial_;
    boost::asio::io_service     io_service_;
    boost::asio::deadline_timer timeout_;

    unsigned char STX_[2];
    unsigned char ETX_[2];

    unsigned char buf_recv_[BUF_SIZE];
    unsigned char buf_send_[BUF_SIZE];
    
    uint32_t stack_len_;
    unsigned char packet_stack_[BUF_SIZE];

private:
    uint32_t seq_recv_;
    uint32_t seq_send_;

    uint32_t len_recv_;
    uint32_t len_send_;

// Variables to elegantly terminate TX & RX threads
private:
    std::shared_future<void> terminate_future_;
    std::promise<void>       terminate_promise_;

// TX & RX threads and their mutex
private:
    std::thread thread_tx_;
    std::thread thread_rx_;

    std::shared_ptr<std::mutex> mutex_tx_;
    std::shared_ptr<std::mutex> mutex_rx_;
};


#endif