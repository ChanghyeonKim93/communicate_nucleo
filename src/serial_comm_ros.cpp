#include "serial_comm_ros.h"

SerialCommROS::SerialCommROS(ros::NodeHandle& nh)
: nh_(nh), portname_("/dev/ttyACM0"), 
baudrate_(115200), loop_frequency_(200)
{
    ROS_INFO_STREAM("SerialCommROS - starts.");
    ROS_INFO_STREAM("Default  port name  : " << portname_);
    ROS_INFO_STREAM("Default  baud rate  : " << baudrate_);
    ROS_INFO_STREAM("Default  node rate  : " << loop_frequency_ << " Hz\n");

    // Get ROS parameters
    this->getParameters();
    
    ROS_INFO_STREAM("User-set port name  : " << portname_);
    ROS_INFO_STREAM("User-set baud rate  : " << baudrate_);
    ROS_INFO_STREAM("User-set node rate  : " << loop_frequency_ << " Hz\n");

    // construct serial communicator
    serial_communicator_ = std::make_shared<SerialCommunicator>(portname_, baudrate_);
    
    // subscriber
    sub_msg_to_send_ = nh_.subscribe<std_msgs::UInt16MultiArray>(topicname_msg_to_send_, 1, &SerialCommROS::callbackToSend, this);

    // publisher
    pub_msg_recv_ = nh_.advertise<std_msgs::Int8MultiArray>(topicname_msg_recv_,1);

    // run
    this->run();
};

SerialCommROS::~SerialCommROS(){
    ROS_INFO_STREAM("SerialCommROS - terminate.");
};

void SerialCommROS::getParameters(){

    if(!ros::param::has("~serial_port")) 
        throw std::runtime_error("SerialCommROS - no 'serial_port' is set. terminate program.\n");
    if(!ros::param::has("~baud_rate")) 
        throw std::runtime_error("SerialCommROS - no 'baud_rate' is set. terminate program.\n");
    if(!ros::param::has("~topicname_pwm"))      
        throw std::runtime_error("SerialCommROS - no 'topicname_pwm' is set. terminate program.\n");
    if(!ros::param::has("~topicname_from_nucleo"))  
        throw std::runtime_error("SerialCommROS - no 'topicname_from_nucleo' is set. terminate program.\n");
    if(!ros::param::has("~frequency"))  
        throw std::runtime_error("SerialCommROS - no 'frequency' is set. terminate program.\n");
    
    ros::param::get("~serial_port",           portname_);
    ros::param::get("~baud_rate",             baudrate_);
    ros::param::get("~topicname_pwm",         topicname_msg_to_send_);
    ros::param::get("~topicname_from_nucleo", topicname_msg_recv_);
    ros::param::get("~frequency",             loop_frequency_);

    ROS_INFO_STREAM("topicname PWM        : " << topicname_msg_to_send_);
    ROS_INFO_STREAM("topicname from nucleo: " << topicname_msg_recv_);

};

void SerialCommROS::run(){
    ROS_INFO_STREAM("SerialCommROS - rosnode runs at {" << loop_frequency_ <<"} Hz\n");
    ros::Rate rate(loop_frequency_);
    while(ros::ok()){
        if(isPacketReady()) {
            uint32_t len = getMessage(buf_recv_);
            if(len > 0){
                // publish the Received message from the Nucleo board.
                for(int i = 0; i < len; ++i) msg_recv_.data.push_back(buf_recv_[i]);
                USHORT_UNION voltage_ushort;
                voltage_ushort.bytes_[0] = buf_recv_[0];
                voltage_ushort.bytes_[1] = buf_recv_[1];

                float voltage_float= ((float)voltage_ushort.ushort_/65535.0f * 3.3f);
                std::cout << "vol ushort : " << voltage_ushort.ushort_ << std::endl;
                ROS_INFO_STREAM("VOLTAGE : " << voltage_float << " V");
                pub_msg_recv_.publish(msg_recv_);
                msg_recv_.data.clear();
            }
            else{
                ROS_WARN_STREAM("In 'getMessage()', it returns len == 0.  An empty packet might be received.");
            }

        }

        ros::spinOnce();
        rate.sleep();
    }
};

void SerialCommROS::callbackToSend(const std_msgs::UInt16MultiArray::ConstPtr& msg){
    int len = this->fill16bitsTo8bits(msg,buf_send_);
    sendMessage(buf_send_, len);
};  


bool SerialCommROS::isPacketReady(){
    return serial_communicator_->isPacketReady();
};

uint32_t SerialCommROS::getMessage(unsigned char* data){
    uint32_t len = 0;
    len = serial_communicator_->getPacket(data);
    return len;
};

void SerialCommROS::sendMessage(unsigned char* data, int len){
    serial_communicator_->sendPacket(data, len);
};

int SerialCommROS::fill16bitsTo8bits(const std_msgs::UInt16MultiArray::ConstPtr& msg, unsigned char* buf_send){
    int len = msg->data.size();
    
    USHORT_UNION data_union;
    for(int i = 0; i < len; ++i) {
        data_union.ushort_ = msg->data[i];
        buf_send_[2*i]     = data_union.bytes_[0];
        buf_send_[2*i+1]   = data_union.bytes_[1];
    }

    return len*2;      
};
