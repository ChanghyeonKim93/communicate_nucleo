#include "serial_comm_mbed.h"

SerialCommunicatorMbed::SerialCommunicatorMbed(int baud_rate)
: buffered_serial_(USBTX,USBRX), recv_led_(LED1), recv_signal_(D12)
{
    //Serial baud rate
    buffered_serial_.set_baud(baud_rate);
    
    flagSTXFound = false;
    flagCTXFound = false;

    idx_packet_ = 0;

    cnt_read_success_ = 0;
    cnt_read_fail_    = 0;

    len_message_read_ = 0;
    len_message_send_ = 0;
};

char SerialCommunicatorMbed::stringChecksum(const char* s, int idx_start, int idx_end) {
  char c = 0;
  for(int i = idx_start; i <= idx_end; ++i) c ^= s[i];
  return c;
};

void SerialCommunicatorMbed::send_withChecksum(const char* data, int len){
    char crc = stringChecksum(data, 0, len-1);

    buf_send_[0] = STX_;
    buf_send_[1] = (char)len;
    for(int i = 0; i < len; ++i) buf_send_[2+i] = data[i];
    buf_send_[len+2] = CTX_;
    buf_send_[len+3] = crc;
    buf_send_[len+4] = ETX_;

    buffered_serial_.write(buf_send_, len+5);
};

void SerialCommunicatorMbed::send_withoutChecksum(const char* data, int len){
    buffered_serial_.write(data, len);
};


bool SerialCommunicatorMbed::tryToReadSerialBuffer(){
    bool packet_ready = false;
    if(buffered_serial_.readable()) { // data recved.
        uint32_t len_read = buffered_serial_.read((void*)buf_read_, BUF_SIZE);       
        if(len_read > 0) { // There is data.
            for(int i = 0; i < len_read; ++i) {
                char current_ascii = buf_read_[i];

                if(current_ascii == STX_){
                    flagSTXFound = true;
                    idx_packet_ = 0; // reset stack
                    buf_packet_[idx_packet_++] = current_ascii;
                }
                else if(flagSTXFound && current_ascii == CTX_){
                    flagCTXFound = true;
                    buf_packet_[idx_packet_++] = current_ascii;
                }
                else if(flagCTXFound && current_ascii == ETX_){ // 'ETX', data part : buf_packet_[1] ~ buf_packet_[]
                    flagCTXFound = false;
                    flagSTXFound = false;
                    buf_packet_[idx_packet_] = current_ascii;
                    idx_packet_ = 0;
                    
                    uint32_t len_message = (uint32_t)buf_packet_[1];
                    char crc_recv = buf_packet_[len_message + 3];
                    char crc_calc = stringChecksum(buf_packet_, 2, len_message + 1);
                    if(crc_calc == crc_recv) { // 'Checksum test' PASS, MESSAGE SUCCESSFULLY RECEIVED.
                        recv_signal_ = true;
                    
                        ++seq_recv_;
                        ++cnt_read_success_;

                        // fill the packet.
                        len_message_read_ = len_message;
                        for(int j = 0; j < len_message_read_; ++j) 
                            message_read_[j] = buf_packet_[j+2];
                        
                        packet_ready = true;
                        recv_signal_ = false;
                    }
                    else ++cnt_read_fail_;
                    // "______________________CRC FAILED: failed / seq : " << cnt_failed <<"/"<<seq_recv_<<"!\n\n";
                }
                else{
                    buf_packet_[idx_packet_++] = current_ascii;
                }    
            }
        }
    }

    return packet_ready;
};

int SerialCommunicatorMbed::getReceivedMessage(char* msg){
    for(int i = 0; i < len_message_read_; ++i){
        msg[i] = message_read_[i];
    }
    return len_message_read_;
};

bool SerialCommunicatorMbed::writable() { return buffered_serial_.writable(); };
bool SerialCommunicatorMbed::readable() { return buffered_serial_.readable(); };
