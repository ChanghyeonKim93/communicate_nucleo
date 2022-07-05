#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>

#include "serial_comm_ros.h"

int main(int argc, char **argv) {
    // ros::init(argc, argv, "hce_gcs", ros::init_options::NoSigintHandler);
    // SignalHandle::initSignalHandler();
    ros::init(argc, argv, "test_publisher");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("test_publisher - STARTS.");
	
    ros::Publisher pub = nh.advertise<std_msgs::Int8MultiArray>("/msg_to_nucleo",1);

	try{
		if(ros::ok()){
            ros::Rate rate(50);
            while(ros::ok()){
                std_msgs::Int8MultiArray msg;
                msg.data.push_back(1);
                msg.data.push_back(3);
                msg.data.push_back(5);
                msg.data.push_back(7);
                pub.publish(msg);
                ros::spinOnce();
                rate.sleep();
            }
		}
		else{
			throw std::runtime_error("ros not ok");
		}
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("test_publisher - TERMINATED.");
	return 0;
}