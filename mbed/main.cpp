#include "mbed.h"
#include "platform/mbed_thread.h"

#include "serial_comm_mbed.h"
#include "PCA9685.h"

// Parameter settings
#define BAUD_RATE 460800
#define I2C_FREQ  400000 // 400 kHz

typedef union USHORT_UNION_{
    uint16_t ushort_;
    char  bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    char  bytes_[4];
} UINT_UNION;

typedef union FLOAT_UNION_{
    float float_;
    char bytes_[4];   
} FLOAT_UNION;

// Serial
SerialCommunicatorMbed serial(BAUD_RATE);

char send_buffer[256];
char recv_buffer[256];

USHORT_UNION pwm_tmp;
uint16_t pwm_values[8]; 
void ISR_readSerial(){
    int len_recv_message = 0;
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        len_recv_message = serial.getReceivedMessage(recv_buffer);
    }

    int idx = 0;
    for(int i = 0; i < 8; ++i){
        pwm_tmp.bytes_[0] = recv_buffer[idx++];
        pwm_tmp.bytes_[1] = recv_buffer[idx++];
        pwm_values[i] = pwm_tmp.ushort_;
    }

    // change the pwm values.
    // for(int i = 0; i < len_recv_message; ++i){
    //     send_buffer[i] = recv_buffer[i];
    // }
    // if(serial.writable() && len_recv_message > 0){
    //     int len_send = len_recv_message;
    //     serial.send_withChecksum(recv_buffer, len_send);
    // }
};


// PCA9685
#define I2C1_SDA PB_7
#define I2C1_SCL PB_6

PCA9685 pca9685_pwm(I2C1_SDA, I2C1_SCL);


void setMotorPWM(uint8_t n, uint16_t pwm_ushort){
    if(pwm_ushort > 4095) pwm_ushort = 4095;
    if(pwm_ushort < 0) pwm_ushort = 0;
    pca9685_pwm.setPWM(n, 0, pwm_ushort);
}

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue equeue(128 * EVENTS_EVENT_SIZE);

// Timer to know how much time elapses.
Timer timer;


int main() {
    // Timer starts.
    timer.start();
    uint64_t us_curr;
    USHORT_UNION tsec;
    UINT_UNION   tusec;

    // Start the event queue
    thread_poll.start(callback(&equeue, &EventQueue::dispatch_forever));
    serial.send_withoutChecksum("Starting event queue in context...",50);
    
    // initialize PWM
    pca9685_pwm.begin();
    // pca9685_pwm.setPrescale(64); //This value is decided for 10ms interval.
    pca9685_pwm.frequencyI2C(400000); // 400kHz
    pca9685_pwm.setPWMFreq(1000.0); // Hz

    // 

    
    // Loop    
    while (true) {

        std::chrono::microseconds tnow = timer.elapsed_time();       

        // Current time
        us_curr      = timer.elapsed_time().count();
        tsec.ushort_ = (uint16_t)(us_curr/1000000);
        tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);
        
        // Write data if IMU data received
        if(serial.writable()) {
        }
    
        // Read data if data exists.
        if(serial.readable()){
            for(int i = 0; i < 8; ++i){
                setMotorPWM(i,pwm_values[i]);
            }
            equeue.call(ISR_readSerial);
        }
    }
}
