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
        int8_t num;
    private:
        void updateimudata(const ros::TimerEvent&);
        void updatemagdata(const ros::TimerEvent&);
        ros::NodeHandle nh_;

        ros::Publisher imu1Pub_;
        ros::Publisher mag1Pub_;
        ros::Timer magTimer_;
        ros::Timer imuTimer_;
        sensor_msgs::Imu imu1_msg;
        sensor_msgs::MagneticField mag1_msg;
};