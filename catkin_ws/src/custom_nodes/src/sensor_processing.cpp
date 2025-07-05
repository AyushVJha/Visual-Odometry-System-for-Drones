#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_processing");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // Process sensor data here
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
