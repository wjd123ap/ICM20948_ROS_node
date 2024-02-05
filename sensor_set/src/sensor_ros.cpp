#include <cmath>
#include "sensor_set/sensor_ros.hpp"

using std::thread;
Sensor_Node::Sensor_Node(ros::NodeHandle& nh){
    nh_=nh;
    
    imu1Pub_=nh_.advertise<sensor_msgs::Imu>("imu1",10);
    mag1Pub_=nh_.advertise<sensor_msgs::MagneticField>("mag1",10);
    // imu2Pub_=nh_.advertise<sensor_msgs::Imu>("imu2",10);
    // mag2Pub_=nh_.advertise<sensor_msgs::MagneticField>("mag2",10);
    int sampling_rate=1125;
    imu1.halfT=1.0/(sampling_rate*2);
    // imu2.halfT=1.0/(sampling_rate*2);
    imu1.I2Cinit(7,REG_VAL_BIT_GYRO_FS_1000DPS,REG_VAL_BIT_ACCEL_FS_4g);
    // imu2.I2Cinit(1,REG_VAL_BIT_GYRO_FS_1000DPS);
    // imu1.gstGyroOffset= {-22,10,5};
    // imu1.stMagoffsets = {-12,-151,-61};
    // imu1.stMagscales = {1.060552, 0.92229, 1.02792};
    // imu1.stAcceloffsets = {-23,-116,-53};//-10,-203,-2
    // imu1.stAccelmatrix ={1.004962, 0.001690 ,-0.002612,
    //                     0.001690, 1.007423 ,-0.000780,
    //                     -0.002612, -0.000780, 0.997988};
    imu1.gstGyroOffset = {0x46, 0x20, 0x0e};
    imu1.stMagoffsets = {-27, 64, -12};
    imu1.stMagscales = {1.0863746, 0.9525333, 0.9711799};
    imu1.stAcceloffsets ={63,-106,133};

    imu1.stAccelmatrix ={1.002437, -0.000545, 0.000216,
                        -0.000545, 1.000011, 0.000926,
                        0.000216, 0.000926, 0.995565};
    imu1.GyroOffsetConfig();
    num1=0;
    num2=0;
    index1=0;
    if (sampling_rate > 0) {
        double duration = 1.0 / (sampling_rate);
        imu1Timer_ = nh_.createTimer(ros::Duration(duration), &Sensor_Node::updateimu1data, this, false, true);
    }
    // if (sampling_rate > 0) {
    //     double duration = 1.0 / (sampling_rate);
    //     imu2Timer_ = nh_.createTimer(ros::Duration(duration), &Sensor_Node::updateimu2data, this, false, true);
    // }
}

Sensor_Node::~Sensor_Node(){
    imu1.Close();
    // imu2.Close();
}

void Sensor_Node::updateimu1data(const ros::TimerEvent&) {
    // To be changed




    

    if(num1==0x00){
        ros::Time now=ros::Time::now();
        imu1.imuDataGet(false);
        imu1.gyro_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stGyroRawData.s16X;
        imu1.gyro_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Y;
        imu1.gyro_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Z;
        imu1.accel_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stAccelRawData.s16X;
        imu1.accel_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Y;
        imu1.accel_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Z;
        

        imu1_msg.header.stamp = publish_time_;
        imu1_msg.header.frame_id = "imu1";
        imu1_msg.angular_velocity.x=imu1.CalAvgValue(imu1.gyro_stAvgBuf[0].s16AvgBuffer)/imu1.GYRO_SCALE_FACTOR;
        imu1_msg.angular_velocity.x=imu1_msg.angular_velocity.x*RAD_CONSTANT;
        imu1_msg.angular_velocity.y=imu1.CalAvgValue(imu1.gyro_stAvgBuf[1].s16AvgBuffer)/imu1.GYRO_SCALE_FACTOR;
        imu1_msg.angular_velocity.y=imu1_msg.angular_velocity.y*RAD_CONSTANT;
        imu1_msg.angular_velocity.z=imu1.CalAvgValue(imu1.gyro_stAvgBuf[2].s16AvgBuffer)/imu1.GYRO_SCALE_FACTOR;
        imu1_msg.angular_velocity.z=imu1_msg.angular_velocity.z*RAD_CONSTANT;
        imu1_msg.linear_acceleration.x = imu1.CalAvgValue(imu1.accel_stAvgBuf[0].s16AvgBuffer)/imu1.ACCEL_SCALE_FACTOR;
        imu1_msg.linear_acceleration.x = imu1_msg.linear_acceleration.x*G_CONSTANT;
        imu1_msg.linear_acceleration.y = imu1.CalAvgValue(imu1.accel_stAvgBuf[1].s16AvgBuffer)/imu1.ACCEL_SCALE_FACTOR;
        imu1_msg.linear_acceleration.y = imu1_msg.linear_acceleration.y*G_CONSTANT;
        imu1_msg.linear_acceleration.z = imu1.CalAvgValue(imu1.accel_stAvgBuf[2].s16AvgBuffer)/imu1.ACCEL_SCALE_FACTOR;
        imu1_msg.linear_acceleration.z = imu1_msg.linear_acceleration.z*G_CONSTANT;
        imu1_msg.orientation.w = imu1.stQuaternion.q0;
        imu1_msg.orientation.x = imu1.stQuaternion.q1;
        imu1_msg.orientation.y = imu1.stQuaternion.q2;
        imu1_msg.orientation.z = imu1.stQuaternion.q3;
        imu1Pub_.publish(imu1_msg);
        publish_time_=now;


    }
    else{
        ros::Time start=ros::Time::now();
        imu1.imuDataGet(false);
        ros::Time end=ros::Time::now();
        // ROS_INFO("processs:%f",(end-start).toSec());
        imu1.gyro_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stGyroRawData.s16X;
        imu1.gyro_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Y;
        imu1.gyro_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Z;
        imu1.accel_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stAccelRawData.s16X;
        imu1.accel_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Y;
        imu1.accel_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Z;
    }
    ++num1;
    ++index1;
    if (num1==0x04)
    {num1=0x00;
    }
    if (index1==0x07)
    {index1=0x00;
    }



    // ROS_INFO("processs:%f",(imu1_msg.header.stamp-start).toSec());

    // mag1_msg.header.stamp= imu1_msg.header.stamp;
    // mag1_msg.header.frame_id = "map";
    // mag1_msg.magnetic_field.x =imu1.stMagRawData.s16X;//*0.15;
    // mag1_msg.magnetic_field.y =imu1.stMagRawData.s16Y;//*0.15;
    // mag1_msg.magnetic_field.z =imu1.stMagRawData.s16Z;//*0.15;



    // mag1Pub_.publish(mag1_msg);
}
// void Sensor_Node::updateimu2data(const ros::TimerEvent&) {
//     // To be changed



//     imu2.stMagRawData.s16X=0;
//     imu2.stMagRawData.s16Y=0;
//     imu2.stMagRawData.s16Z=0;
   
//     if(num2==0x00){


//         imu2.imuDataGet(true);

       


//     }
//     else{

//         imu2.imuDataGet(false);

//     }
//     ++num2;
//     if (num2==0x04)
//     {num2=0x00;
//     }
//     imu2_msg.header.stamp = ros::Time::now();
//     imu2_msg.header.frame_id = "map";
//     imu2_msg.angular_velocity.x=imu2.stGyroRawData.s16X/imu2.GYRO_SCALE_FACTOR;
//     imu2_msg.angular_velocity.y=imu2.stGyroRawData.s16Y/imu2.GYRO_SCALE_FACTOR;
//     imu2_msg.angular_velocity.z=imu2.stGyroRawData.s16Z/imu2.GYRO_SCALE_FACTOR;
//     imu2_msg.linear_acceleration.x = imu2.stAccelRawData.s16X/ACCEL_SCALE_FACTOR;
//     imu2_msg.linear_acceleration.y = imu2.stAccelRawData.s16Y/ACCEL_SCALE_FACTOR;
//     imu2_msg.linear_acceleration.z = imu2.stAccelRawData.s16Z/ACCEL_SCALE_FACTOR;
//     imu2_msg.orientation.w = imu2.stQuaternion.q0;
//     imu2_msg.orientation.x = imu2.stQuaternion.q1;
//     imu2_msg.orientation.y = imu2.stQuaternion.q2;
//     imu2_msg.orientation.z = imu2.stQuaternion.q3;
//     mag2_msg.header.stamp= imu2_msg.header.stamp;
//     mag2_msg.header.frame_id = "map";
//     mag2_msg.magnetic_field.x =imu2.stMagRawData.s16X;//*0.15;
//     mag2_msg.magnetic_field.y =imu2.stMagRawData.s16Y;//*0.15;
//     mag2_msg.magnetic_field.z =imu2.stMagRawData.s16Z;//*0.15;
//     mag2Pub_.publish(mag2_msg);
//     imu2Pub_.publish(imu2_msg);

// }