#include "ros/ros.h"

#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h> 
#include <pcl/registration/icp.h> //ICP(iterative closest point)

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

	Vector3f acc_b;
	Vector3f acc_w;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_previous;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_current;

	bool is_scan_valid(const sensor_msgs::LaserScan &scan);

	void scanCallback(const sensor_msgs::LaserScan& scan);
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

void LaserOdometry::scanCallback(const sensor_msgs::LaserScan& scan)
{
	laser_geometry::LaserProjection projector;
	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud2 cloud2;
	//convert LaserScan to PointCloud
	projector.projectLaser(scan, cloud);
	//convert PointCloud to PointCloud2
	sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

	if(!initialized)
	{
		pcl::fromROSMsg(cloud2, *pcl_cloud_previous);
		initialized = true;
	}else
	{
		pcl::fromROSMsg(cloud2, *pcl_cloud_current);
		//*********************************
		// ICP
		//*********************************
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
		icp.setInputSource(pcl_cloud_previous); //set the input cloud
		icp.setInputTarget(pcl_cloud_current); //set the output cloud
		pcl::PointCloud<pcl::PointXYZ> final; //result
		//进行配准，结果存储在Final中
		icp.align(final); 
		//输出ICP配准的信息（是否收敛，拟合度）
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
		//输出最终的变换矩阵（4x4）
		std::cout << icp.getFinalTransformation() << std::endl;

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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_odom");
	ros::NodeHandle n;
	LaserOdometry test(n);
	ros::spin();
}