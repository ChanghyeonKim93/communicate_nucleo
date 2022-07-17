#include "mbed.h"
#include "platform/mbed_thread.h"

#include "serial_comm_mbed.h"
#include "PCA9685.h"
#include "union_struct.h"
#include <chrono>

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Parameter settings
#define BAUD_RATE 921600
#define I2C_FREQ  400000 // 400 kHz

#define I2C1_SDA_PCA9685 PB_7
#define I2C1_SCL_PCA9685 PB_6

#define VOLTAGE_PIN A0

// Serial
uint8_t packet_send[256];
uint8_t packet_recv[256];

uint16_t pwm_values[8]; 

SerialCommunicatorMbed serial(BAUD_RATE);

// PCA9685
PCA9685 pca9685_pwm(I2C1_SDA_PCA9685, I2C1_SCL_PCA9685);

void setMotorPWM(uint8_t n, uint16_t pwm_ushort){
    if(pwm_ushort > 4095) pwm_ushort = 4095;
    if(pwm_ushort < 0) pwm_ushort = 0;
    pca9685_pwm.setPWM(n, 0, pwm_ushort);
};

void setMotorPWM_01234567(uint16_t pwm_ushort[8]){
    for(int i = 0; i < 8; ++i){
        if(pwm_ushort[i] < 0 ) pwm_ushort[i] = 0;
        if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    }
    pca9685_pwm.setPWM_01234567(pwm_ushort);
};

// Voltage reader                
FLOAT_UNION vol_float;
AnalogIn voltage_adc(VOLTAGE_PIN);



// Define ISR functions
void ISR_readSerial(){
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial.getReceivedMessage(packet_recv); 

        if( len_recv_message > 0 ) { // Successfully received the packet.
            USHORT_UNION pwm_tmp;
            int idx = 0;
            for(int i = 0; i < 8; ++i) {
                pwm_tmp.bytes_[0] = packet_recv[idx++];
                pwm_tmp.bytes_[1] = packet_recv[idx++];
                pwm_values[i]     = pwm_tmp.ushort_;
            }
            setMotorPWM_01234567(pwm_values);           
        }
    }
};



// Timer to know how much time elapses.
Timer timer;


int main() {
    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;

    uint64_t us_curr;
    USHORT_UNION tsec;
    UINT_UNION   tusec;

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    // Initialize PWM PCA9685 device
    pca9685_pwm.begin();
    // pca9685_pwm.setPrescale(64); //This value is decided for 10ms interval.
    pca9685_pwm.frequencyI2C(400000); // 400kHz
    pca9685_pwm.setPWMFreq(1000.0); // PWM frequency in Hertz

    // Loop    
    int cnt = 0 ;
    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send = time_curr-time_send_prev;
        // std::chrono::duration<int, std::micro> dt_recv = time_curr-time_recv_prev;

        if(dt_send.count() > 9999){ // 10 ms interval
            if(serial.writable()) {
                cnt = 0;
                vol_float.float_ = voltage_adc; // Read Analog voltage data (A0 pin, AnalogIn)
                serial.send_withChecksum(vol_float.bytes_, 4);
            }

            time_send_prev = time_curr;
        }
    
        // Read data if data exists.
        if(serial.readable()){
            ISR_readSerial();
            
        }
    }
    
    return 0;
}
