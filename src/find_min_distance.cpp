#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "laserscan/Laser.h"
#include <math.h>

laserscan::Laser pos;
void scanCallback(const sensor_msgs::LaserScan laser)
{
  float average_min_distance=0.0;
  int average_min_angle=0;
  int angle=0;
  int count=0;
  float min_dist[10]={20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0};
  int min_angle[10];

  for(int i=0; i<laser.ranges.size(); i++)
  {
    if(laser.ranges[i]<min_dist[0])
    {
      min_dist[0] = laser.ranges[i];
      min_angle[0]=i;
    }
  }
  for(int j=1; j<10; j++)
  {
    for(int i=0; i<laser.ranges.size(); i++)
    {
      if(laser.ranges[i]<min_dist[j] && laser.ranges[i]>min_dist[j-1])
      {
        min_dist[j] = laser.ranges[i];
        min_angle[j]=i;
      }
    }
  }
  for(int k=0; k<10; k++)
  {
    angle = angle + min_angle[k];
  }
  angle = angle/10;

  for(int l=0; l<10; l++)
  {
    if(abs(min_angle[l]-angle)<40)
    {
    	average_min_distance += min_dist[l];
    	average_min_angle += min_angle[l];
    	count++;
    }
  }
  
  pos.min_distance = average_min_distance/count*100;
  pos.angle = average_min_angle/count*0.25 - 45;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_min_distance");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
  ros::Publisher pub = n.advertise<laserscan::Laser>("/laser_send", 10);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
