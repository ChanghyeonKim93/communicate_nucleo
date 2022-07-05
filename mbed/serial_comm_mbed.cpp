#include "serial_comm_mbed.h"

SerialCommunicatorMbed::SerialCommunicatorMbed(int baud_rate)
: pc(USBTX,USBRX),
 led_recv(LED1)
{
    //Serial baud rate
    pc.set_baud(baud_rate);
};

uint8_t SerialCommunicatorMbed::stringChecksum(const uint8_t* s, int idx_start, int idx_end) {
  uint8_t c = 0;
  for(int i = idx_start; i <= idx_end; ++i) c ^= s[i];
  return c;
};

void SerialCommunicatorMbed::send_withChecksum(const uint8_t* data, int len){
    uint8_t crc = stringChecksum(data, 0, len-1);

    send_buf_[0] = STX_;
    send_buf_[1] = (uint8_t)len;
    for(int i = 0; i < len; ++i) send_buf_[2+i] = data[i];
    send_buf_[len+2] = crc;
    send_buf_[len+3] = ETX_;

    pc.write(send_buf_, len+4);
};

void SerialCommunicatorMbed::send_withoutChecksum(const char* data, int len){
    pc.write(data, len);
};

int SerialCommunicatorMbed::read_withChecksum(uint8_t* msg){
    bool len_msg = 0;
    if(pc.readable()){
        int i = 0;
        led_recv = true;
        int len_read = pc.read((void*)read_buf_, READ_BUFFER_SIZE);
        if(len_read > 0){
            for(i = 0; i < len_read; ++i) {
                if(read_buf_[i] == ETX_) {
                    if(serial_stack_[0] == STX_) {
                        // CRC is calculated including len.
                        len_msg = (int)serial_stack_[1];
                        uint8_t check_sum = stringChecksum(serial_stack_, 2, len_msg + 1); 
                        if(serial_stack_[len_msg+2] == check_sum){
                            // successfully data received
                            ++seq;
                            for(int j = 0; j < len_msg; ++j) msg[j] = serial_stack_[j+2];
                        }
                    }
                    stack_len_ = 0;
                }
                else { // stack
                    serial_stack_[stack_len_++] = read_buf_[i];
                }
            }
        }
        led_recv = false;
    }
    return len_msg;
}; 

bool SerialCommunicatorMbed::writable(){
    return pc.writable();
};
bool SerialCommunicatorMbed::readable(){
    return pc.readable();
};
