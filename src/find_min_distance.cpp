#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "laserscan/Laser.h"

laserscan::Laser pos;
void scanCallback(const sensor_msgs::LaserScan laser)
{
  float min_distance;
  int angle;

  min_distance =20.0;
  angle = 0;
  for(int i=0; i<laser.ranges.size(); i++)
  {
    if(laser.ranges[i]<min_distance)
    {
      min_distance = laser.ranges[i];
      angle=i;
    }
  }
  pos.min_distance = min_distance*100;
  pos.angle = angle*0.25 - 45;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_min_distance");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
  ros::Publisher pub = n.advertise<laserscan::Laser>("/laser_send", 1000);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
