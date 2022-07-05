#include "serial_comm_ros.h"

SerialCommROS::SerialCommROS(ros::NodeHandle& nh)
: nh_(nh){
    ROS_INFO_STREAM("SerialCommROS - starts.");
    portname_ = "/dev/ttyACM0";
    baudrate_ = 460800;
    loop_frequency_ = 200;
    ROS_INFO_STREAM("Default PORTNANE:  " << portname_);
    ROS_INFO_STREAM("Default baud rate: " << baudrate_);
    ROS_INFO_STREAM("Default frequency: " << loop_frequency_);

    this->getParameters();
    
    ROS_INFO_STREAM("SerialCommROS - serial port name: " << portname_);
    ROS_INFO_STREAM("SerialCommROS - serial baud rate: " << baudrate_);
    serial_comm_linux_ = std::make_shared<SerialCommunicatorLinux>(portname_, baudrate_);
    
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

    ROS_INFO_STREAM("User PORTNANE        : " << portname_);
    ROS_INFO_STREAM("User baud rate       : " << baudrate_);
    ROS_INFO_STREAM("User frequency       : " << loop_frequency_);

    ROS_INFO_STREAM("topicname PWM        : " << topicname_msg_to_send_);
    ROS_INFO_STREAM("topicname from nucleo: " << topicname_msg_recv_);

};

void SerialCommROS::run(){
    ros::Rate rate(1000);
    while(ros::ok()){
        if(receiveDataReady()) {
            int len = getMessage(buf_recv_);

            // publish the Received message from the Nucleo board.
            for(int i = 0; i < len; ++i) msg_recv_.data.push_back(buf_recv_[i]);
            FLOAT_UNION voltage_float;
            for(int i = 0; i < 4; ++i) voltage_float.bytes_[i] = buf_recv_[i];

            std::cout << "VOLTAGE : " << voltage_float.float_ * 3.3f<< " V" << std::endl;
            pub_msg_recv_.publish(msg_recv_);
            msg_recv_.data.clear();
        }

        ros::spinOnce();
        rate.sleep();
    }
};

void SerialCommROS::callbackToSend(const std_msgs::UInt16MultiArray::ConstPtr& msg){
    // ROS_INFO_STREAM("'msg_to_send' recv.\n");
    int len = this->fill16bitsTo8bits(msg,buf_send_);
    sendMessage(buf_send_, len);
    // ROS_INFO_STREAM("'msg_to_send' is transmitted to the serial comm.\n");
};  


bool SerialCommROS::receiveDataReady(){
    return serial_comm_linux_->isReceiveReady();
};

int SerialCommROS::getMessage(char* data){
    int len = 0;
    len = serial_comm_linux_->getMessage(data);

    return len;
};

void SerialCommROS::sendMessage(char* data, int len){
    serial_comm_linux_->sendMessage(data, len);
};

int SerialCommROS::fill16bitsTo8bits(const std_msgs::UInt16MultiArray::ConstPtr& msg, char* buf_send){
    int len = msg->data.size();
    
    USHORT_UNION data_union;
    for(int i = 0; i < len; ++i) {
        data_union.ushort_ = msg->data[i];
        buf_send_[2*i]   = data_union.bytes_[0];
        buf_send_[2*i+1] = data_union.bytes_[1];
    }

    return len*2;      
};
