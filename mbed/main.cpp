/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DigitalOut.h"
#include "cmsis_os2.h"
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "serial_comm_mbed.h"

#include <cstdint>

// Parameter settings
#define BAUD_RATE 230400
#define I2C_FREQ  400000 // 400 kHz

typedef union USHORT_UNION_{
    uint16_t ushort_;
    uint8_t  bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    uint8_t  bytes_[4];
} UINT_UNION;

typedef union FLOAT_UNION_{
    float float_;
    uint8_t bytes_[4];   
} FLOAT_UNION;

// Serial
SerialCommunicatorMbed serial(BAUD_RATE);

uint8_t send_buffer[256];
uint8_t recv_buffer[256];

DigitalOut recv_led(LED1);
void workISRUserContext_readSerial(){
    recv_led = true;
    serial.read_withChecksum(recv_buffer);
    recv_led = false;
};

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue equeue(128 * EVENTS_EVENT_SIZE);

// Timer to know how much time elapses.
Timer timer;


int main()
{
    // Timer starts.
    timer.start();
    uint64_t us_curr;
    USHORT_UNION tsec;
    UINT_UNION   tusec;

    // Start the event queue
    thread_poll.start(callback(&equeue, &EventQueue::dispatch_forever));
    serial.send_withoutChecksum("Starting event queue in context...",50);
    
    int cnt = 0;
    // Loop
    while (true) {
        std::chrono::microseconds tnow = timer.elapsed_time();       

        // Current time
        us_curr      = timer.elapsed_time().count();
        tsec.ushort_ = (uint16_t)(us_curr/1000000);
        tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);
        
        // Write data if IMU data received
        if(0) {
            serial.send_withChecksum(send_buffer, 24);
        }
    
        // Read data if data exists.
        if(serial.readable()){
            equeue.call(workISRUserContext_readSerial);
        }
                
    }
}
