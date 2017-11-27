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

	sm_params input_;
	sm_result output_;
	LDP prev_ldp_scan_;

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

void LaserOdometry::processScan(LDP& curr_ldp_scan, const ros::Time& time)
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

  double dt = (time - last_icp_time_).toSec();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the fixed frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf::Transform corr_ch;

  if (output_.valid)
  {

    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;

    // **** publish

    if (publish_pose_)
    {
      // unstamped Pose2D message
      geometry_msgs::Pose2D::Ptr pose_msg;
      pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
      pose_msg->x = f2b_.getOrigin().getX();
      pose_msg->y = f2b_.getOrigin().getY();
      pose_msg->theta = tf::getYaw(f2b_.getRotation());
      pose_publisher_.publish(pose_msg);
    }
    if (publish_pose_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
      pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();

      pose_stamped_msg->header.stamp    = time;
      pose_stamped_msg->header.frame_id = fixed_frame_;

      tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);

      pose_stamped_publisher_.publish(pose_stamped_msg);
    }
    if (publish_pose_with_covariance_)
    {
      // unstamped PoseWithCovariance message
      geometry_msgs::PoseWithCovariance::Ptr pose_with_covariance_msg;
      pose_with_covariance_msg = boost::make_shared<geometry_msgs::PoseWithCovariance>();
      tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_publisher_.publish(pose_with_covariance_msg);
    }
    if (publish_pose_with_covariance_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
      pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

      pose_with_covariance_stamped_msg->header.stamp    = time;
      pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

      tf::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_stamped_publisher_.publish(pose_with_covariance_stamped_msg);
    }

    if (publish_tf_)
    {
      tf::StampedTransform transform_msg (f2b_, time, fixed_frame_, base_frame_);
      tf_broadcaster_.sendTransform (transform_msg);
    }
  }
  else
  {
    corr_ch.setIdentity();
    ROS_WARN("Error in scan matching");
  }

  // **** swap old and new

  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
  }
  else
  {
    ld_free(curr_ldp_scan);
  }

  last_icp_time_ = time;

  // **** statistics

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
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