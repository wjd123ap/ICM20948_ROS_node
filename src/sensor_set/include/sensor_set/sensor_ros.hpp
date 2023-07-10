#pragma once

// STL
#include <iostream>
#include <mutex>
#include <string>
// Eigen
#include <Eigen/Dense>


// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sensor_set/imu_lib.hpp"
class Sensor_Node{
    public:
        
        Sensor_Node(ros::NodeHandle& nh);
        ~Sensor_Node();

        using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using RowMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        IMU_QUATERNION_DATA stQuaternion;
	    IMU_ST_SENSOR_DATA stGyroRawData;
	    IMU_ST_SENSOR_DATA stAccelRawData;
        ICM20948 imu1;
        ICM20948 imu2;
    private:
        void updateimudata(const ros::TimerEvent&);
        ros::NodeHandle nh_;

        ros::Publisher imu1Pub_;
        ros::Publisher imu2Pub_;
        ros::Timer imuTimer_;
        sensor_msgs::Imu imu1_msg;
        sensor_msgs::Imu imu2_msg;
        
};