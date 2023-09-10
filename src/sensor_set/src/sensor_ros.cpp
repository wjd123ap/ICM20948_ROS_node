#include <cmath>
#include "sensor_set/sensor_ros.hpp"

using std::thread;
Sensor_Node::Sensor_Node(ros::NodeHandle& nh){
    nh_=nh;
    
    imu1Pub_=nh_.advertise<sensor_msgs::Imu>("imu1",10);
    mag1Pub_=nh_.advertise<sensor_msgs::MagneticField>("mag1",10);
    float sampling_rate=1100/4;
    imu1.halfT=1.0/(sampling_rate*2);
    
    imu1.I2Cinit(7);
    num=0;
    if (sampling_rate > 0) {
        double duration = 1.0 / (sampling_rate);
        imuTimer_ = nh_.createTimer(ros::Duration(duration), &Sensor_Node::updateimudata, this, false, true);
    }
    if (sampling_rate > 0) {
        double duration = 1.0 / (sampling_rate);
        imuTimer_ = nh_.createTimer(ros::Duration(duration), &Sensor_Node::updateimudata, this, false, true);
    }
}

Sensor_Node::~Sensor_Node(){
    imu1.Close();

}

void Sensor_Node::updateimudata(const ros::TimerEvent&) {
    // To be changed



    imu1.stMagRawData.s16X=0;
    imu1.stMagRawData.s16Y=0;
    imu1.stMagRawData.s16Z=0;
   
    if(num==0x03){
        imu1.MagRead(imu1.stMagRawData.s16X,imu1.stMagRawData.s16Y,imu1.stMagRawData.s16Z);
        mag1_msg.header.stamp= ros::Time::now();
        mag1_msg.header.frame_id = "imu_link";
        mag1_msg.magnetic_field.x =imu1.stMagRawData.s16X*0.15;
        mag1_msg.magnetic_field.y =imu1.stMagRawData.s16Y*0.15;
        mag1_msg.magnetic_field.z =imu1.stMagRawData.s16Z*0.15;
        mag1Pub_.publish(mag1_msg);
    }
    ++num;
    num&= 0x03;
    imu1.imuDataGet();

    imu1_msg.header.stamp = ros::Time::now();
    imu1_msg.header.frame_id = "imu_link";
    imu1_msg.angular_velocity.x=imu1.stGyroRawData.s16X/GYRO_SCALE_FACTOR;
    imu1_msg.angular_velocity.y=imu1.stGyroRawData.s16Y/GYRO_SCALE_FACTOR;
    imu1_msg.angular_velocity.z=imu1.stGyroRawData.s16Z/GYRO_SCALE_FACTOR;
    imu1_msg.linear_acceleration.x = imu1.stAccelRawData.s16X/ACCEL_SCALE_FACTOR;
    imu1_msg.linear_acceleration.y = imu1.stAccelRawData.s16Y/ACCEL_SCALE_FACTOR;
    imu1_msg.linear_acceleration.z = imu1.stAccelRawData.s16Z/ACCEL_SCALE_FACTOR;
    imu1_msg.orientation.w = imu1.stQuaternion.q0;
    imu1_msg.orientation.x = imu1.stQuaternion.q1;
    imu1_msg.orientation.y = imu1.stQuaternion.q2;
    imu1_msg.orientation.z = imu1.stQuaternion.q3;


    imu1Pub_.publish(imu1_msg);

}
