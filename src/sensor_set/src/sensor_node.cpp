#include <ros/ros.h>
#include "sensor_set/sensor_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensorNode");
  ros::NodeHandle nh("~");
 

  Sensor_Node sensorNode(nh);


  // Spin
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}