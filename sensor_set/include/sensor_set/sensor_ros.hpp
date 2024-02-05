#pragma once

// STL
#include <iostream>
#include <mutex>
#include <string>



// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <thread>
#include "sensor_set/imu_lib.hpp"
class Sensor_Node{
    public:
        
        Sensor_Node(ros::NodeHandle& nh);
        ~Sensor_Node();

        ICM20948 imu1;
        ICM20948 imu2;
        int8_t num1;
        int8_t num2;
        int8_t index1;
    private:
        void updateimu1data(const ros::TimerEvent&);
        void updateimu2data(const ros::TimerEvent&);
        ros::NodeHandle nh_;

        ros::Publisher imu1Pub_;
        ros::Publisher mag1Pub_;
        ros::Publisher imu2Pub_;
        ros::Publisher mag2Pub_;
        ros::Timer imu1Timer_;
        ros::Timer imu2Timer_;
        ros::Time publish_time_;
        sensor_msgs::Imu imu1_msg;
        sensor_msgs::MagneticField mag1_msg;
        sensor_msgs::Imu imu2_msg;
        sensor_msgs::MagneticField mag2_msg;
};