#include <iostream>
#include <string>

#include <ros/ros.h>

#include "serial_comm_ros.h"

int main(int argc, char **argv) {
    // ros::init(argc, argv, "hce_gcs", ros::init_options::NoSigintHandler);
    // SignalHandle::initSignalHandler();
    ros::init(argc, argv, "serial_node");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("serial_node - STARTS.");
	
	try{
		if(ros::ok()){
			std::shared_ptr<SerialCommROS> serial_ros;
			serial_ros = std::make_shared<SerialCommROS>(nh);
		}
		else{
			throw std::runtime_error("ros not ok");
		}
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("serial_node - TERMINATED.");
	return 0;
}