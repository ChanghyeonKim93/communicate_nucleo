#include "mbed.h"
#include "platform/mbed_thread.h"
#include "serial_comm_mbed.h"

#include "PCA9685.h"

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue equeue(128 * EVENTS_EVENT_SIZE);

// Unions
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


// Parameter settings
#define BAUD_RATE 921600
#define I2C_FREQ  400000 // 400 kHz

#define I2C1_SDA_PCA9685 PB_7
#define I2C1_SCL_PCA9685 PB_6

#define VOLTAGE_PIN A0

// Serial
char send_buffer[256];
char recv_buffer[256];
uint16_t pwm_values[8]; 

SerialCommunicatorMbed serial(BAUD_RATE);
void ISR_readSerial(){
    int len_recv_message = 0;
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        len_recv_message = serial.getReceivedMessage(recv_buffer);
    }

    int idx = 0;
    USHORT_UNION pwm_tmp;
    for(int i = 0; i < 8; ++i){
        pwm_tmp.bytes_[0] = recv_buffer[idx++];
        pwm_tmp.bytes_[1] = recv_buffer[idx++];
        pwm_values[i]     = pwm_tmp.ushort_;
    }
};


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
    
    // initialize PWM PCA9685 device
    pca9685_pwm.begin();
    // pca9685_pwm.setPrescale(64); //This value is decided for 10ms interval.
    pca9685_pwm.frequencyI2C(400000); // 400kHz
    pca9685_pwm.setPWMFreq(1000.0); // PWM frequency in Hertz

    // Loop    
    int cnt = 0 ;
    while (true) {
        if(serial.writable()) {
            ++cnt;
            if(cnt > 500){
                cnt = 0;
                float voltage_now = voltage_adc;
                vol_float.float_  = voltage_now;

                char v[4] = {vol_float.bytes_[0], vol_float.bytes_[1], vol_float.bytes_[2], vol_float.bytes_[3]};

                serial.send_withChecksum(v, 4);
            }
        }
    
        // Read data if data exists.
        if(serial.readable()){
            setMotorPWM_01234567(pwm_values);
            
            equeue.call(ISR_readSerial);
            
        }
    }
}
