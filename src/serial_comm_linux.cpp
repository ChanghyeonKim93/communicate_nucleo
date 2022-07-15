#include "serial_comm_linux.h"
using namespace std::chrono_literals;

// #define VERBOSE

SerialCommunicatorLinux::SerialCommunicatorLinux(const std::string& portname, const int& baud_rate)
    : STX_('$'), CTX_('*'), ETX_('%'), seq_recv_(0), seq_send_(0), len_send_(0), len_recv_(0) 
{    
    // initialize mutex
    mutex_rx_ = std::make_shared<std::mutex>();
    mutex_tx_ = std::make_shared<std::mutex>();

    // initialize the portname     
    this->setPortName(portname);

    // Check whether this baud rate is valid
    this->setBaudRate(baud_rate);

    // In 'termios-baud.h',
    double bytes_per_second = (double)baud_rate/(10.0);
    double packet_per_second = bytes_per_second/27.0; // 3413. Hz
    double time_per_packet = 1./packet_per_second;

#ifdef VERBOSE
    std::cout << "bytes_per_second: "  << bytes_per_second << std::endl;
    std::cout << "packet_per_second: " << packet_per_second << std::endl;
    std::cout << "time_per_packet: "   << time_per_packet*1000.0 << " [ms]" << std::endl; // 0.29 ms
#endif

    // Try to open serial port.
    this->openSerialPort();
    
    // Run TX RX threads
    this->terminate_future_ = this->terminate_promise_.get_future();
    this->runThreadRX();
    this->runThreadTX();

    printf("[SerialComm] Portname is {%s}\n", portname_.c_str());
};

    // deconstructor
SerialCommunicatorLinux::~SerialCommunicatorLinux() {
    // Terminate signal .
    std::cout << "SerialCommunicatorLinux - terminate signal on...\n";
    this->terminate_promise_.set_value();

    // wait for TX & RX threads to terminate ...
    std::this_thread::sleep_for(1s);

    if(this->thread_rx_.joinable()) this->thread_rx_.join();
    std::cout << "thread RX join.";
    if(this->thread_tx_.joinable()) this->thread_tx_.join();
    std::cout << "thread TX join.";

    // Close the serial port.
    this->closeSerialPort();
};

bool SerialCommunicatorLinux::isReceiveReady(){

    bool res = false;
    mutex_rx_->lock();
    res = len_recv_ > 0;
    mutex_rx_->unlock();
    return res;
};

int SerialCommunicatorLinux::getMessage(char* buf){
    // return len > 0 when data ready.
    // else -1.
    int len = -1;

    mutex_rx_->lock();
    if(len_recv_ > 0){
        len = len_recv_;
        for(int i = 0; i < len; ++i) buf[i] = buf_recv_[i];
        len_recv_ = 0;
    }
    mutex_rx_->unlock();

    // len == -1 : error!
    // len > 0 success.
    return len;
};

bool SerialCommunicatorLinux::sendMessage(char* buf, int len){
    bool isOK = true;

    this->mutex_tx_->lock();
    // update message & length
    len_send_ = len; // Flag up!
    for(int i = 0; i < len; ++i) buf_send_[i] = buf[i];
    this->mutex_tx_->unlock();
    return isOK;
};

void SerialCommunicatorLinux::runThreadRX(){
    this->thread_rx_ = std::thread([&](){ processRX(this->terminate_future_); } );
};

void SerialCommunicatorLinux::runThreadTX(){
    this->thread_tx_ = std::thread([&](){ processTX(this->terminate_future_); } );
};


void SerialCommunicatorLinux::setBaudRate(int baud_rate){
    if(     baud_rate == 57600)   BAUD_RATE_ = B57600;
    else if(baud_rate == 115200)  BAUD_RATE_ = B115200;
    else if(baud_rate == 230400)  BAUD_RATE_ = B230400;
    else if(baud_rate == 460800)  BAUD_RATE_ = B460800;
    else if(baud_rate == 500000)  BAUD_RATE_ = B500000;
    else if(baud_rate == 576000)  BAUD_RATE_ = B576000;
    else if(baud_rate == 921600)  BAUD_RATE_ = B921600;
    else if(baud_rate == 1000000) BAUD_RATE_ = B1000000;
    else if(baud_rate == 1152000) BAUD_RATE_ = B1152000;
    else if(baud_rate == 1500000) BAUD_RATE_ = B1500000;
    else if(baud_rate == 2000000) BAUD_RATE_ = B2000000;
    else if(baud_rate == 2500000) BAUD_RATE_ = B2500000;
    else if(baud_rate == 3000000) BAUD_RATE_ = B3000000;
    else if(baud_rate == 3500000) BAUD_RATE_ = B3500000;
    else if(baud_rate == 4000000) BAUD_RATE_ = B4000000;
    else std::runtime_error("Unsupported Baudrate...");
};

void SerialCommunicatorLinux::setPortName(std::string portname){ portname_ = portname; };


void SerialCommunicatorLinux::copyReadToRecvBuffer(int len){
    mutex_rx_->lock();
    len_recv_ = len;
    for(int j = 0; j < len; ++j) buf_recv_[j] = serial_stack_[j+2];
    mutex_rx_->unlock();                          
};

void SerialCommunicatorLinux::send_withChecksum(const char* data, int len){
    char crc   = stringChecksum(data, 0, len-1);
    char len_c = (char)len;
    // 0 : STX
    // 1 : length
    // 2 ~ len+1 : message
    // len+2 : CTX
    // len+3 : crc
    // len+4 : ETX
    write(fd_, &STX_,  1);
    write(fd_, &len_c, 1);
    write(fd_, data,   len);
    write(fd_, &CTX_,  1);
    write(fd_, &crc,   1);
    write(fd_, &ETX_,  1);
    len_send_ = 0; // initialize.
};

void SerialCommunicatorLinux::openSerialPort(){
    std::cout << "Opening the serial port...\n";
    fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    /* clear struct for new port settings */
    memset(&this->term_setting_, 0, sizeof(this->term_setting_));
    
    /* Set baudrate, 8n1, no modem control, enable receiving characters. */
    term_setting_.c_cflag     = BAUD_RATE_ | CS8 | CLOCAL | CREAD;
    term_setting_.c_cflag     &= ~CSTOPB;
	term_setting_.c_iflag     = 0; /* No parity bit */
	term_setting_.c_oflag     = 0;			/* Enable raw data output. */
    term_setting_.c_lflag     = 0;
	term_setting_.c_cc[VTIME] = 0;	/* Do not use inter-character timer. */
	term_setting_.c_cc[VMIN]  = 0;	/* Block on reads until 0 character is received. */

    /* Clear the COM port buffers. */
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &term_setting_);

    //
    serial_struct serial;
    ioctl(fd_, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &serial);

    // poll preparation
    poll_events_.fd      = fd_;
    poll_events_.events  = POLLIN | POLLERR; // if received or if there is error.
    poll_events_.revents = 0;

    if (fd_ == -1) throw std::runtime_error("Error - no serial port '" + portname_ + "' is opened! Check the serial device.\n");
    else printf("{%s} is opened.\n",portname_.c_str());
};

void SerialCommunicatorLinux::closeSerialPort(){
    std::cout << "Close the serial port...";
    close(fd_);
};

void SerialCommunicatorLinux::processRX(std::shared_future<void> terminate_signal){
    // 'term_setting_' initialization.
    auto start = std::chrono::system_clock::now();

    bool flagSTXFound = false;
    bool flagCTXFound = false;

    stack_len_ = 0;
    int timeout_poll    = 500; // ms, -1: nonblock
    uint32_t cnt_failed = 0;
    // WHILE LOOP
    while(true) { // polling stage. (when some data is received.)
        auto t1 = std::chrono::system_clock::now();
        poll_state_ = poll((struct pollfd *)&poll_events_, 1, timeout_poll); // time out [ms]
        if (poll_state_ > 0) {
            if (poll_events_.revents & POLLIN) { // event is receiving data.
                int len_read = read(fd_, buf_read_, 128); // Read serial RX buffer
                
                if(len_read > 0) {
                    for(int i = 0; i < len_read; ++i) {
                        char current_ascii = buf_read_[i];

                        if(current_ascii == STX_){
                            flagSTXFound = true;
                            stack_len_ = 0; // reset stack
                            serial_stack_[stack_len_++] = current_ascii;
                        }
                        else if(flagSTXFound && current_ascii == CTX_){
                            flagCTXFound = true;
                            serial_stack_[stack_len_++] = current_ascii;
                        }
                        else if(flagCTXFound && current_ascii == ETX_){ // 'ETX', data part : serial_stack_[1] ~ serial_stack_[]
                            flagCTXFound = false;
                            flagSTXFound = false;

                            serial_stack_[stack_len_++] = current_ascii;
                            stack_len_ = 0;
                            
                            int len_message = (int)serial_stack_[1];
                            char crc_recv = serial_stack_[len_message + 3];
                            char crc_calc = stringChecksum(serial_stack_, 2, len_message + 1);
                            if(crc_calc == crc_recv) { // 'Checksum test' PASS, MESSAGE SUCCESSFULLY RECEIVED.
                                ++seq_recv_;
                                this->copyReadToRecvBuffer(len_message);
                            }
                            else {
                                ++cnt_failed;
                                std::cout << "______________________CRC FAILED: failed / seq : " << cnt_failed <<"/"<<seq_recv_<<"!\n\n";
                            }
                        }
                        else{
                            serial_stack_[stack_len_++] = current_ascii;
                        }    
                    }
                }
                else {
#ifdef VERBOSE
                    std::cout << "______________________LEN <= 0\n\n";
#endif
                }
            }
            if (poll_events_.revents & POLLERR) {// The connection is destructed.
                printf("[SerialComm] ERROR - There is communication error. program exit.\n");
                break;
            }
        }
        else if (poll_state_ == 0) { //timeout.
#ifdef VERBOSE
            printf("[SerialComm] RX - NO RX data. TIMEOUT-1000 [ms].\n");
#endif
        }
        else if(poll_state_ < 0) { // Error occurs.
            continue;
        }

        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(10));
        if (terminate_status == std::future_status::ready) break;
    } // END WHILE
    std::cout << "   PROCESS RX receives terminate signal.\n";
};


void SerialCommunicatorLinux::processTX(std::shared_future<void> terminate_signal){
    while(true){
        if(len_send_ > 0){
            mutex_tx_->lock();
            int len_tmp = len_send_;
            send_withChecksum(buf_send_, len_send_);
            std::cout << " TX Send:" << ++seq_send_ << ", length: " << len_tmp << std::endl;
            len_send_ = 0;
            mutex_tx_->unlock();
        }

        // std::this_thread::sleep_for(5ms);
        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(10));
        if (terminate_status == std::future_status::ready) {
            USHORT_UNION data_union;
            len_send_ = 16;
            for(int i = 0; i < len_send_; ++i) {
                data_union.ushort_ = (unsigned short)0;
                buf_send_[2*i]     = data_union.bytes_[0];
                buf_send_[2*i+1]   = data_union.bytes_[1];
            }
            send_withChecksum(buf_send_, len_send_);
            len_send_ = 0;
            std::cout << "send the last message... :\n";
            for(int i = 0 ; i < 16; ++i) std::cout << buf_send_[i] << " ";
            std::cout << std::endl;
            break;
        }
    }
    std::cout << "   PROCESS TX receives terminate signal.\n";
};

inline char SerialCommunicatorLinux::stringChecksum(const char* s, int idx_start, int idx_end) {
    char c = 0;
    for(int i = idx_start; i <= idx_end; ++i) c ^= s[i];
    return c;
};   

unsigned short SerialCommunicatorLinux::stringChecksumCRC16_CCITT(const char* s, int idx_start, int idx_end){
    return crc16_ccitt(s,idx_start, idx_end);
};
