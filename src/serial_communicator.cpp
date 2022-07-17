#include "serial_communicator.h"

SerialCommunicator::SerialCommunicator(const std::string& portname, const int& baud_rate)
    : STX_({DLE, STX}), ETX_({DLE, ETX}), 
    seq_recv_(0), seq_send_(0), 
    len_send_(0), len_recv_(0),
    len_packet_(0),
    io_service_(), timeout_(io_service_), flag_packet_ready_(false)
{    
    // initialize mutex
    mutex_rx_ = std::make_shared<std::mutex>();
    mutex_tx_ = std::make_shared<std::mutex>();

    // initialize the portname     
    this->setPortName(portname);

    // Check whether this baud rate is valid
    this->checkSupportedBaudRate(baud_rate);

    // Try to open serial port.
    this->openSerialPort();    
    
    // Run TX RX threads
    this->terminate_future_ = this->terminate_promise_.get_future();
    this->runThreadTX();
    this->runThreadRX();
    
    printf("SerialCommunicator - port {%s} is open.\n", portname_.c_str());
};

// deconstructor
SerialCommunicator::~SerialCommunicator() {
    // Terminate signal .
    std::cerr << "SerialCommunicator - terminate signal published...\n";
    this->terminate_promise_.set_value();

    // wait for TX & RX threads to terminate ...
    std::cerr << "                   - waiting 1 second to join TX / RX threads ...\n";
    std::this_thread::sleep_for(1s);

    if(this->thread_rx_.joinable()) this->thread_rx_.join();
    std::cerr << "                   -   RX thread joins successfully.\n";

    if(this->thread_tx_.joinable()) this->thread_tx_.join();
    std::cerr << "                   -   TX thread joins successfully.\n";

    // Close the serial port.
    this->closeSerialPort();
    std::cerr << "SerialCommunicator - terminated.\n";
};

void SerialCommunicator::setPortName(std::string portname){ portname_ = portname; };
void SerialCommunicator::setBaudRate(int baudrate) {
    checkSupportedBaudRate(baudrate);

    boost::asio::serial_port_base::baud_rate baud_rate_option2(baudrate);
    serial_->set_option(baud_rate_option2);
    boost::asio::serial_port_base::baud_rate baud_rate_option3;
    serial_->get_option(baud_rate_option3);
    std::cout << "SerialCommunicator - baudrate is changed from 115200 (default) to " << baud_rate_option3.value() << std::endl;
};

void SerialCommunicator::checkSupportedBaudRate(int baud_rate){
    if(   baud_rate == 57600
       || baud_rate == 115200
       || baud_rate == 230400
       || baud_rate == 460800
       || baud_rate == 500000
       || baud_rate == 576000
       || baud_rate == 921600
       || baud_rate == 1000000
       || baud_rate == 1152000
       || baud_rate == 1500000
       || baud_rate == 2000000
       || baud_rate == 2500000
       || baud_rate == 3000000
       || baud_rate == 3500000
       || baud_rate == 4000000)
    {
        // OK
        baud_rate_ = baud_rate;
    }
    else std::runtime_error("SerialCommunicator - Unsupported Baudrate...");
};


void SerialCommunicator::openSerialPort(){
    // Generate serial port.
    std::cout << "SerialCommunicator - opening the serial port...\n";
    serial_ = new boost::asio::serial_port(io_service_);

    // Try to open the serial port
    try{
        serial_->open(portname_);
    }
    catch (boost::system::system_error& error){
        std::cout << "SerialCommunicator - port [" << portname_.c_str() << "] cannot be opened. Error message:" << error.what() << std::endl;
        throw std::runtime_error(error.what());
    }

    // If serial port cannot be opened, 
    if ( !serial_->is_open() ) {
      std::cout << "SerialCommunicator - [" << portname_ <<"] is not opened. terminate the node\n";
      throw std::runtime_error("");
    }

    // Set serial port spec.
    // No flow control, 8bit / no parity / stop bit 1
    boost::asio::serial_port_base::baud_rate    baud_rate_option(baud_rate_);
    boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
    boost::asio::serial_port_base::parity       parity(boost::asio::serial_port_base::parity::none);
    boost::asio::serial_port_base::stop_bits    stop_bits(boost::asio::serial_port_base::stop_bits::one);

    serial_->set_option(baud_rate_option);
    serial_->set_option(flow_control);
    serial_->set_option(parity);
    serial_->set_option(stop_bits);
};

void SerialCommunicator::closeSerialPort(){
    serial_->close();
    std::cerr << "SerialCommunicator - portname [" << portname_ << "] is closed...\n";
};

void SerialCommunicator::runThreadRX(){
    this->thread_rx_ = std::thread([&](){ processRX(this->terminate_future_); } );
};

void SerialCommunicator::runThreadTX(){
    this->thread_tx_ = std::thread([&](){ processTX(this->terminate_future_); } );
};

void SerialCommunicator::processRX(std::shared_future<void> terminate_signal){
    // Initialize
    bool flagStacking = false;
    bool flagDLEFound = false;

    std::this_thread::sleep_for(2s);
    idx_stk_ = 0;
    timer::tic();
    while(true){
        // Try to read serial port
        int len_read = serial_->read_some(boost::asio::buffer(buf_recv_, BUF_SIZE));

        if( len_read > 0) { // There is data
            // std::cout << " len read : " <<len_read << std::endl;

            for(int i = 0; i < len_read; ++i) {



                if( flagStacking ) { // 현재 Packet stack 중...

                    if( flagDLEFound ) { // 1) DLE, DLE / 2) DLE, ETX

                        if( buf_recv_[i] == DLE ) { // 1) DLE, DLE --> 실제데이터가 DLE
                            flagDLEFound = false;

                            packet_stack_[idx_stk_] = buf_recv_[i];
                            ++idx_stk_;
                        }
                        else if( buf_recv_[i] == ETX ){ // 2) DLE, ETX
                            flagStacking = false;
                            flagDLEFound = false;
                            ++seq_recv_;

                            // Packet END. Copy the packet.
                            std::cout << "== " << timer::toc(0) << "ms, ETX found. seq: " << seq_recv_ << ", length: " << idx_stk_ << std::endl;
                            mutex_rx_->lock();
                            for(int j = 0; j < idx_stk_; ++j){
                                packet_[j] = packet_stack_[j];
                                std::cout << (int)packet_[j] << " ";
                            }
                            std::cout << std::endl;
                            
                            len_packet_        = idx_stk_;
                            flag_packet_ready_ = true;
                            mutex_rx_->unlock();

                            idx_stk_ = 0;                  
                        }
                    }
                    else { // 이전에 DLE가 발견되지 않았다.
                        if(buf_recv_[i] == DLE){ // DLE발견
                            flagDLEFound = true;
                        }
                        else { // 스택.
                            packet_stack_[idx_stk_] = buf_recv_[i];
                            ++idx_stk_; 
                            if(idx_stk_ >= BUF_SIZE){
                                flagStacking = false;
                                flagDLEFound = false;
                                std::cout << "WARNING ! - RX STACK OVER FLOW!\n" << std::endl;
                            }
                            // std::cout << (int)buf_recv_[i] << std::endl;
                        }
                    }

                }
                else { // 아직 STX를 발견하지 못함. (Stack 하지않음)
                    if(flagDLEFound){ // 이전에 DLE나옴.
                        if(buf_recv_[i] == STX) { // STX 찾음, 새로운 packet을 stack 시작함.
                            std::cout << "== " << " found STX!\n";
                            flagDLEFound       = false;

                            flagStacking       = true;
                            idx_stk_   = 0;

                            flag_packet_ready_ = false;
                        }
                    }
                    else { // 
                        std::cout << " not found STX. DLE found. \n";
                        if(buf_recv_[i] == DLE) flagDLEFound = true;
                    }
                }


            }
        }

        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(10));
        if (terminate_status == std::future_status::ready){
                    

            break;
        }
    }
    std::cerr << "SerialCommunicator - RX thread receives termination signal.\n";
};

void SerialCommunicator::processTX(std::shared_future<void> terminate_signal){
    while(true){
        // Do something


        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(10));
        if (terminate_status == std::future_status::ready){


            break;
        } 
    }
    std::cerr << "SerialCommunicator - TX thread receives termination signal.\n";
};

unsigned short SerialCommunicator::stringChecksumCRC16_CCITT(const char* s, int idx_start, int idx_end){
    return crc16_ccitt(s,idx_start, idx_end);
};
