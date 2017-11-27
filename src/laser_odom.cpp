#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

const double g_constant = 9.8;

class LaserOdometry
{
public: 
	LaserOdometry(ros::NodeHandle n);
	~LaserOdometry();
private:
	ros::NodeHandle nh;
	ros::Subscriber scan_sub;
	ros::Subscriber imu_sub;
	ros::Publisher odom_pub;

	bool initialized;
	bool use_imu;
	bool data_valid;

	double roll;
	double pitch;
	double yaw;

	double a_cos_;
	double a_sin_;

	Vector3f acc_b;
	Vector3f acc_w;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_previous;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_current;

	bool is_scan_valid(const sensor_msgs::LaserScan::ConstPtr& scan);

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg, LDP& ldp);
	void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg, double& theta);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

};

LaserOdometry::LaserOdometry(ros::NodeHandle n):
	nh(n),
	initialized(false),
	use_imu(false)
{
	scan_sub = nh.subscribe("scan", 1, &LaserOdometry::scanCallback, this);
	imu_sub = nh.subscribe("/mavros/imu/data", 1, &LaserOdometry::imuCallback, this);
	odom_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_projected", 1);

	if(!nh.getParam ("use_imu", use_imu))
	{
		use_imu = false;
	}

}

LaserOdometry::~LaserOdometry()
{
}

void LaserOdometry::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if(!initialized)
	{
		initialized = true;
	}else
	{
		createCache(scan, yaw);
		

	}
}

void LaserOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	tf::Quaternion q;
	tf::quaternionMsgToTF(imu_msg->orientation, q);
	tf::Matrix3x3 R(q);
	R.getRPY(roll, pitch, yaw);

	tf::Vector3 acc_body(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
	tf::Vector3 acc_world;
	
	acc_world = R * acc_body;

	acc_b(0) = imu_msg->linear_acceleration.x;
	acc_b(1) = imu_msg->linear_acceleration.y;
	acc_b(2) = imu_msg->linear_acceleration.z;
	acc_w(0) = acc_world.getX();
	acc_w(1) = acc_world.getY();
	acc_w(2) = acc_world.getZ();
}

void LaserOdometry::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg, LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserOdometry::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg, double& theta)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment + theta;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_odom");
	ros::NodeHandle n;
	LaserOdometry test(n);
	ros::spin();
}