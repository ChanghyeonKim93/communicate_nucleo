#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_publisher");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("test_publisher - STARTS.");
	
    ros::Publisher pub = nh.advertise<std_msgs::UInt16MultiArray>("/serial/pwm",1);

	try{
        uint16_t pwm_values[100] = {0,};
        uint16_t a = 0;
        for(int jj = 0; jj < 100; ++jj){
            pwm_values[jj] = a; 
            a += 40;
        }
        uint8_t cnt = 0;
		if(ros::ok()){
            ros::Rate rate(400);
            while(ros::ok()){
                std_msgs::UInt16MultiArray msg;
                if(cnt==100) cnt=0;
                for(int i = 0; i < 8; ++i){
                    msg.data.push_back(pwm_values[cnt]);
                }
                ++cnt;
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