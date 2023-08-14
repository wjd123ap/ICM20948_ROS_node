# ICM20948_ROS_node 
ros sensor_node icm20948 i2c
## comand block
```
$ sudo usermod -aG i2c $USER
$ git clone https://github.com/wjd123ap/ICM20948_ROS_node.git
$ cd ./ICM20948_ROS_node 
$ catkin_make
$ source ./devel/setup.bash
$ roslaunch sensor_set sensor_set.launch
```
