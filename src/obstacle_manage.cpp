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

class ObstacleProcess
{
public: 
	ObstacleProcess();
private:
	ros::NodeHandle n;
	ros::Subscriber obstacle_sub;
	ros::Publisher obstacle_cloud_pub;
	
	void obstacleCallback(const geometry_msgs::PointStamped obstacle);
};

ObstacleProcess::ObstacleProcess()
{
	obstacle_sub = n.subscribe("/obstacle_position", 1, &ObstacleProcess::obstacleCallback, this);
	obstacle_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/obstacle_cloud", 1);
}

void ObstacleProcess::obstacleCallback(const geometry_msgs::PointStamped obstacle)
{

	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_manage");
	ObstacleProcess ObstacleProcess;
	ros::spin();
}
