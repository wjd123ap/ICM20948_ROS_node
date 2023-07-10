#include <cmath>
#include "sensor_set/sensor_ros.hpp"
Sensor_Node::Sensor_Node(ros::NodeHandle& nh){
    nh_=nh;

    imu1Pub_=nh_.advertise<sensor_msgs::Imu>("imu1",10);
    imu2Pub_=nh_.advertise<sensor_msgs::Imu>("imu2",10);
    int sampling_rate=200;
    imu1.I2Cinit(7);
    imu2.I2Cinit(1);

    if (sampling_rate > 0) {
        double duration = 1.0 / (sampling_rate + 0.00001);
        imuTimer_ = nh_.createTimer(ros::Duration(duration), &Sensor_Node::updateimudata, this, false, true);
    }

    
    
}

Sensor_Node::~Sensor_Node(){
    imu1.Close();
    imu2.Close();
}

void Sensor_Node::updateimudata(const ros::TimerEvent&) {
    // To be changed
    imu1.imuDataGet(stQuaternion,stGyroRawData,stAccelRawData);
    
    imu1_msg.header.stamp = ros::Time::now();
    imu1_msg.header.frame_id = "imu_link";
    imu1_msg.angular_velocity.x=stGyroRawData.s16X/GYRO_SCALE_FACTOR;
    imu1_msg.angular_velocity.y=stGyroRawData.s16Y/GYRO_SCALE_FACTOR;
    imu1_msg.angular_velocity.z=stGyroRawData.s16Z/GYRO_SCALE_FACTOR;
    imu1_msg.linear_acceleration.x=stAccelRawData.s16X/ACCEL_SCALE_FACTOR;
    imu1_msg.linear_acceleration.y=stAccelRawData.s16Y/ACCEL_SCALE_FACTOR;
    imu1_msg.linear_acceleration.z=stAccelRawData.s16Z/ACCEL_SCALE_FACTOR;
    imu1_msg.orientation.w=stQuaternion.q0;
    imu1_msg.orientation.x=stQuaternion.q1;
    imu1_msg.orientation.y=stQuaternion.q2;
    imu1_msg.orientation.z=stQuaternion.q3;
    imu1Pub_.publish(imu1_msg);
    imu2.imuDataGet(stQuaternion,stGyroRawData,stAccelRawData);
    imu2_msg.header.stamp = ros::Time::now();
    imu2_msg.header.frame_id = "imu_link";
    imu2_msg.angular_velocity.x=stGyroRawData.s16X/GYRO_SCALE_FACTOR;
    imu2_msg.angular_velocity.y=stGyroRawData.s16Y/GYRO_SCALE_FACTOR;
    imu2_msg.angular_velocity.z=stGyroRawData.s16Z/GYRO_SCALE_FACTOR;
    imu2_msg.linear_acceleration.x=stAccelRawData.s16X/ACCEL_SCALE_FACTOR;
    imu2_msg.linear_acceleration.y=stAccelRawData.s16Y/ACCEL_SCALE_FACTOR;
    imu2_msg.linear_acceleration.z=stAccelRawData.s16Z/ACCEL_SCALE_FACTOR;
    imu2_msg.orientation.w=stQuaternion.q0;
    imu2_msg.orientation.x=stQuaternion.q1;
    imu2_msg.orientation.y=stQuaternion.q2;
    imu2_msg.orientation.z=stQuaternion.q3;
    imu2Pub_.publish(imu2_msg);
}