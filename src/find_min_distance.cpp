#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "laser_geometry/laser_geometry.h"
#include "laserscan/Laser.h"
#include "mavros_extras/LaserDistance.h"
#include "Eigen/Dense"
#include <math.h>
#include <vector>

#define Pi 3.141592653

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
	ros::Publisher pub;

	laser_geometry::LaserProjection projector;
	sensor_msgs::PointCloud cloud;

	geometry_msgs::PointStamped obstacle;
	geometry_msgs::PointStamped obstacle_prev;
	
	void scanCallback(const sensor_msgs::LaserScan scan);
};

ScanProcess::ScanProcess()
{
	scan_sub = n.subscribe("/scan_projected", 1, &ScanProcess::scanCallback, this);
	pub = n.advertise<mavros_extras::LaserDistance>("/laser_send", 1);
	obstacle_pub = n.advertise<geometry_msgs::PointStamped>("/obstacle_position", 1);
	cloud_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
}

void ScanProcess::scanCallback(const sensor_msgs::LaserScan scan)
{
	//convert scan to cloud data
	projector.projectLaser(scan, cloud);

	int cluster_num = 0;
	float min_distance = 30;
	bool disturb = false;

	obstacle_prev = obstacle;

	sensor_msgs::PointCloud cloud_cluster = cloud;

	vector<geometry_msgs::Point32> cluster[scan.ranges.size()];
	
	for(int i=0; i<cloud.points.size(); i++)
	{ 
		if(i == 0)
		{
			cluster[cluster_num].push_back(cloud.points[0]);
			cloud_cluster.channels[0].values[0] = cluster_num*5;
		}else
		{
			if(sqrt((cloud.points[i].x - cloud.points[i-1].x)*(cloud.points[i].x - cloud.points[i-1].x) + (cloud.points[i].y - cloud.points[i-1].y)*(cloud.points[i].y - cloud.points[i-1].y)) < 0.1)
			{
				cluster[cluster_num].push_back(cloud.points[i]);
				cloud_cluster.channels[0].values[i] = cluster_num*5;
			}else
			{
				cluster_num++;
				cluster[cluster_num].push_back(cloud.points[i]);
				cloud_cluster.channels[0].values[i] = cluster_num*5;
			}
		}
	}
	//ROS_INFO("cluster number:%d",cluster_num+1);

	cloud_cluster.channels[0].name = "intensity";
	cloud_pub.publish(cloud_cluster);

	
	for(int i=0; i<=cluster_num; i++)
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

		if(distance < min_distance && distance > 0.5 && cluster[i].size() > 1)
		{
			min_distance = distance;
			obstacle.header.stamp = ros::Time::now();
			obstacle.header.frame_id = "laser";
			obstacle.point.x = x;
			obstacle.point.y = y;
			obstacle.point.z = 0;
		}
	}

	//ROS_INFO("min_distance:%f", min_distance);
	mavros_extras::LaserDistance obstacle_pos;
	obstacle_pos.min_distance = min_distance;
	obstacle_pos.angle = atan2(obstacle.point.y, obstacle.point.x);
	if(obstacle_pos.angle < 0) obstacle_pos.angle = obstacle_pos.angle + 2 * Pi;
	
	if(sqrt((obstacle.point.x-obstacle_prev.point.x)*(obstacle.point.x-obstacle_prev.point.x)+
			(obstacle.point.y-obstacle_prev.point.y)*(obstacle.point.y-obstacle_prev.point.y))>2.0)
	{
		disturb = true;
	}
	if(!disturb)
	{
		pub.publish(obstacle_pos);
		obstacle_pub.publish(obstacle);
	}
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_min_distance");
	ScanProcess ScanProcess;
	ros::spin();
}
