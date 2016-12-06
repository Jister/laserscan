#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "laser_geometry/laser_geometry.h"
#include "laserscan/Laser.h"
#include "Eigen/Dense"
#include <math.h>
#include <vector>

using namespace std;
using namespace Eigen;

class ScanProcess
{
public: 
	ScanProcess();
private:
	ros::NodeHandle n;
	ros::Subscriber scan_sub;
	ros::Publisher obstacle_pub;
	ros::Publisher cloud_pub;
	laserscan::Laser pos;
	laser_geometry::LaserProjection projector;
	sensor_msgs::PointCloud cloud;
	

	void scanCallback(const sensor_msgs::LaserScan scan);
};

ScanProcess::ScanProcess()
{
	scan_sub = n.subscribe("/scan_projected", 1, &ScanProcess::scanCallback, this);
	//pub = n.advertise<laserscan::Laser>("/laser_send", 1);
	obstacle_pub = n.advertise<geometry_msgs::PointStamped>("/obstacle_position", 1);
	cloud_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
}

void ScanProcess::scanCallback(const sensor_msgs::LaserScan scan)
{
	//convert scan to cloud data
	projector.projectLaser(scan, cloud);

	int flag[cloud.points.size()];
	memset(flag,0,sizeof(flag));

	int cluster_num = 0;
	float min_distance = 30;
	geometry_msgs::PointStamped obstacle;
	sensor_msgs::PointCloud cloud_cluster = cloud;

	
	for(int i=0; i<cloud.points.size(); i++)
	{ 
		for(int j=i; j<cloud.points.size(); j++)
		{
			if(sqrt((cloud.points[i].x - cloud.points[j].x)*(cloud.points[i].x - cloud.points[j].x) + (cloud.points[i].y - cloud.points[j].y)*(cloud.points[i].y - cloud.points[j].y)) < 0.2)
			{
				if(flag[j] == 0 && flag[i] == 0)
				{
					cluster_num++;
					flag[j] = cluster_num;
				}else if(flag[j] == 0 && flag[i] != 0)
				{
					flag[j] = flag[i];
				}	
			}
		}
	}
	ROS_INFO("cluster number:%d",cluster_num);

	vector<geometry_msgs::Point32> cluster[cluster_num];
	for(int i=0; i<cloud.points.size(); i++)
	{
		cluster[flag[i]-1].push_back(cloud.points[i]);
		cloud_cluster.channels[0].values[i] = flag[i]*5;
	}
	cloud_cluster.channels[0].name = "intensity";
	cloud_pub.publish(cloud_cluster);

	
	for(int i=0; i<cluster_num; i++)
	{
		float x = 0;
		float y = 0;
		float distance = 0;
		for(int j=0; j<cluster[i].size(); j++)
		{
			x = x + cluster[i][j].x;
			y = y + cluster[i][j].y;
		}
		x = x / cluster[i].size();
		y = y / cluster[i].size();
		distance = sqrt(x * x + y * y);
		//ROS_INFO("x:%f  y:%f  distance:%f", x, y, distance);

		if(distance < min_distance && distance > 0.5)
		{
			min_distance = distance;
			obstacle.header.stamp = ros::Time::now();
			obstacle.header.frame_id = "laser";
			obstacle.point.x = x;
			obstacle.point.y = y;
			obstacle.point.z = 0;
		}
	}
	ROS_INFO("min_distance:%f", min_distance);
	ROS_INFO("angle:%f", atan2(obstacle.point.y,obstacle.point.x)/3.14*180);
	obstacle_pub.publish(obstacle);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_min_distance");
	ScanProcess ScanProcess;
	ros::spin();
}