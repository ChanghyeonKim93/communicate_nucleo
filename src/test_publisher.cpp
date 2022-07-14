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
        uint16_t pwm_values[8] = {0,400,800,1200,1200,800,400,0};
        uint16_t pwm_step[8]   = {10,10,10,10,10,10,10,10};

        uint16_t MIN_PWM = 0;
        uint16_t MAX_PWM = 4095;// 12 bits

        uint8_t cnt = 0;
		if(ros::ok()){
            ros::Rate rate(800);
            while(ros::ok()){
                std_msgs::UInt16MultiArray msg;
                for(int i = 0; i < 8; ++i){
                    msg.data.push_back(pwm_values[i]);
                    pwm_values[i] += pwm_step[i];
                    if(pwm_values[i] > 4095){
                        pwm_values[i] = 4095;
                        pwm_step[i] = -pwm_step[i];
                    }
                    if(pwm_values[i] < 0){
                        pwm_values[i] = 0;
                        pwm_step[i] = -pwm_step[i];
                    }
                }
                
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