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
	ros::Publisher cloud_pub;
	ros::Publisher cloud_project_pub;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform base_to_laser; // static, cached
	tf::Transform ortho_to_laser; // computed from b2l, w2b, w2o
	bool initialized;
	vector<double> a_sin_;
    vector<double> a_cos_;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
	void getOrthoTf(const tf::Transform& world_to_base, tf::Transform& world_to_ortho);
	bool getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};

Projector::Projector()
{
	scan_sub = n.subscribe("/scan", 1, &Projector::scanCallback, this);
	cloud_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
	cloud_project_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud_project", 1);
	initialized = false;
}

void Projector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if(!initialized)
	{
		initialized = getBaseToLaserTf(scan);
		if (initialized) 
			createCache(scan);
		else return;
	}

	projector.projectLaser(*scan, cloud);
	cloud_pub.publish(cloud);

	sensor_msgs::PointCloud cloud_project = cloud;
	cloud_project.header.frame_id = "/base_ortho";

	for (int i = 0; i < scan->ranges.size(); i++)
	{
		double r = scan->ranges[i];
		tf::Vector3 p(r * a_cos_[i], r * a_sin_[i], 0.0);
		p = ortho_to_laser * p;

		geometry_msgs::Point32 point;
		point.x = p.getX();
		point.y = p.getY();
		point.z = 0.0;

		cloud_project.points[i] = point;
	}
	cloud_project_pub.publish(cloud_project);
}

void Projector::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	tf::Transform world_to_base;
	world_to_base.setIdentity();

	tf::Quaternion q;
	tf::quaternionMsgToTF(imu_msg->orientation, q);
	world_to_base.setRotation(q);

	// calculate world to ortho frame transform
	tf::Transform world_to_ortho;
	getOrthoTf(world_to_base, world_to_ortho);

	//tf::StampedTransform world_to_ortho_tf(world_to_ortho, imu_msg->header.stamp, "/world", "/base_ortho");
	//tf_broadcaster.sendTransform(world_to_ortho_tf);

	// calculate ortho to laser tf, and save it for when scans arrive
	ortho_to_laser = world_to_ortho.inverse() * world_to_base * base_to_laser;
}

void Projector::getOrthoTf(const tf::Transform& world_to_base, tf::Transform& world_to_ortho)
{
	const tf::Vector3&    w2b_o = world_to_base.getOrigin();
	const tf::Quaternion& w2b_q = world_to_base.getRotation();

	tf::Vector3 wto_o(w2b_o.getX(), w2b_o.getY(), 0.0);
	tf::Quaternion wto_q = tf::createQuaternionFromYaw(tf::getYaw(w2b_q));

	world_to_ortho.setOrigin(wto_o);
	world_to_ortho.setRotation(wto_q);
} 

bool Projector::getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	tf::StampedTransform base_to_laser_tf;
	try
	{
		tf_listener.waitForTransform("/base_link", scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(1.0));
		tf_listener.lookupTransform ("/base_link", scan_msg->header.frame_id, scan_msg->header.stamp, base_to_laser_tf);
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("LaserOrthoProjector: Could not get initial laser transform(%s)", ex.what());
		return false;
	}
	base_to_laser = base_to_laser_tf;

	return true;
}

void Projector::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
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