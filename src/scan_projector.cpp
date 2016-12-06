#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Eigen/Dense"
#include <math.h>
#include <vector>

using namespace std;
using namespace Eigen;

class Projector
{
public: 
	Projector();
private:
	ros::NodeHandle n;
	ros::Subscriber scan_sub;
	ros::Subscriber imu_sub;
	ros::Publisher scan_pub;
	 
	bool initialized;
	vector<double> a_sin_;
    vector<double> a_cos_;

    double roll;
    double pitch;
    double yaw;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
	void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};

Projector::Projector()
{
	scan_sub = n.subscribe("/scan", 10, &Projector::scanCallback, this);
	imu_sub = n.subscribe("/mavros/imu/data", 10, &Projector::imuCallback, this);
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan_projected", 10);
	initialized = false;
	roll = 0;
	pitch = 0;
	yaw = 0;
}

void Projector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if(!initialized)
	{
		createCache(scan);
		ROS_INFO("initialized");
		initialized = true;
	}

	sensor_msgs::LaserScan scan_projected = *scan;
	scan_projected.header.frame_id = "laser_projection";

	for (unsigned int i = 0; i < scan->ranges.size(); i++)
	{
		double r = scan->ranges[i];
		scan_projected.ranges[i] = r * sqrt(a_cos_[i]*a_cos_[i]*cos(pitch)*cos(pitch)  + a_sin_[i]*a_sin_[i]*cos(roll)*cos(roll));
	}
	scan_pub.publish(scan_projected);
}

void Projector::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	tf::Quaternion q;
	tf::quaternionMsgToTF(imu_msg->orientation, q);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	ROS_INFO("roll:%f",roll);
	ROS_INFO("pitch:%f",pitch);
	ROS_INFO("yaw:%f",yaw);
}


void Projector::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_projector");
	Projector Projector;
	ros::spin();
}