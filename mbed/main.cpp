#include "mbed.h"
#include "platform/mbed_thread.h"

#include "serial_comm_mbed.h"
#include "drone_motor_pwm.h"
#include "union_struct.h"
#include <chrono>

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Parameter settings
#define BAUD_RATE 921600
#define I2C_FREQ  400000 // 400 kHz

// Voltage pin
#define VOLTAGE_PIN PC_5

// Serial
uint8_t packet_send[256];
uint8_t packet_recv[256];

uint16_t pwm_values[8]; 

SerialCommunicatorMbed serial(BAUD_RATE);

// PCA9685
DroneMotorPwm motor_pwm;

void setMotorPWM(uint8_t n, uint16_t pwm_ushort){
    if(pwm_ushort > 4095) pwm_ushort = 4095;
    if(pwm_ushort < 0) pwm_ushort = 0;
    motor_pwm.setPWM(n, pwm_ushort);
};

void setMotorPWM_01234567(uint16_t pwm_ushort[8]){
    for(int i = 0; i < 8; ++i){
        if(pwm_ushort[i] < 0 ) pwm_ushort[i] = 0;
        if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    }
    motor_pwm.setPWM_all(pwm_ushort);
};

// Voltage reader                
USHORT_UNION vol_ushort;
AnalogIn voltage_adc(VOLTAGE_PIN);

// Define ISR functions
void ISR_readSerial(){
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial.getReceivedMessage(packet_recv); 

        if( len_recv_message == 16 ) { // Successfully received the packet.
            USHORT_UNION pwm_tmp;
            
            pwm_tmp.bytes_[0] = packet_recv[0]; pwm_tmp.bytes_[1] = packet_recv[1];
            pwm_values[0]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[2]; pwm_tmp.bytes_[1] = packet_recv[3];
            pwm_values[1]     = pwm_tmp.ushort_;
    
            pwm_tmp.bytes_[0] = packet_recv[4]; pwm_tmp.bytes_[1] = packet_recv[5];
            pwm_values[2]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[6]; pwm_tmp.bytes_[1] = packet_recv[7];
            pwm_values[3]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[8]; pwm_tmp.bytes_[1] = packet_recv[9];
            pwm_values[4]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[10]; pwm_tmp.bytes_[1] = packet_recv[11];
            pwm_values[5]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[12]; pwm_tmp.bytes_[1] = packet_recv[13];
            pwm_values[6]     = pwm_tmp.ushort_;

            pwm_tmp.bytes_[0] = packet_recv[14]; pwm_tmp.bytes_[1] = packet_recv[15];
            pwm_values[7]     = pwm_tmp.ushort_;

            setMotorPWM_01234567(pwm_values);           
        }
    }
};

void ISR_sendSerialVoltage(){
    vol_ushort.ushort_ = voltage_adc.read_u16(); // Read Analog voltage data (A0 pin, AnalogIn)
    // vol_ushort.ushort_ = 22222;
    serial.send_withChecksum(vol_ushort.bytes_, 2);
};


// Timer to know how much time elapses.
Timer timer;

int main() {
    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    // Initialize PWM PCA9685 device
    // pca9685_pwm.begin();
    // pca9685_pwm.frequencyI2C(400000); // 400kHz
    // pca9685_pwm.setPWMFreq(1000.0); // PWM frequency in Hertz

    // Loop    
    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send = time_curr-time_send_prev;

        if(dt_send.count() > 2499){ // 2.5 ms interval
            if(serial.writable()) 
                ISR_sendSerialVoltage();
            
            time_send_prev = time_curr;
        }
    
        // Read data if data exists.
        if(serial.readable())
            ISR_readSerial();            
        
    }
    
    return 0;
}
