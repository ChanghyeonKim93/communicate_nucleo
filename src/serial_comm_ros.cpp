#include "serial_comm_ros.h"

SerialCommROS::SerialCommROS(ros::NodeHandle& nh)
: nh_(nh){
    ROS_INFO_STREAM("SerialCommROS - starts.");

    portname_ = "/dev/ttyACM0";
    baudrate_ = 230400;
    ROS_INFO_STREAM("SerialCommROS - serial port name: " << portname_);
    ROS_INFO_STREAM("SerialCommROS - serial baud rate: " << baudrate_);
    serial_comm_linux_ = std::make_shared<SerialCommunicatorLinux>(portname_, baudrate_);
    
    // subscriber
    this->topicname_msg_to_send_ = "/msg_to_nucleo";
    sub_msg_to_send_ = nh_.subscribe<std_msgs::Int8MultiArray>(topicname_msg_to_send_, 1, &SerialCommROS::callbackToSend, this);

    // publisher
    this->topicname_msg_recv_ = "/msg_recv";
    pub_msg_recv_ = nh_.advertise<std_msgs::Int8MultiArray>(topicname_msg_recv_,1);

    // run
    this->run();
};

SerialCommROS::~SerialCommROS(){
    ROS_INFO_STREAM("SerialCommROS - terminate.");
};

void SerialCommROS::run(){
    ros::Rate rate(500);
    while(ros::ok()){
        if(receiveDataReady()) {
            int len = getMessage(buf_recv_);
            // ROS_INFO_STREAM("SerialCommROS - msg recv-len:" << len);

            // publish
            for(int i = 0; i < len; ++i) msg_recv_.data.push_back(buf_recv_[i]);
            pub_msg_recv_.publish(msg_recv_);

            msg_recv_.data.resize(0);
        }

        ros::spinOnce();
        rate.sleep();
    }
};

void SerialCommROS::callbackToSend(const std_msgs::Int8MultiArray::ConstPtr& msg){
    // ROS_INFO_STREAM("'msg_to_send' recv.\n");

    int len = msg->data.size();
    for(int i = 0; i < len; ++i) buf_send_[i] = msg->data[i];
    
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