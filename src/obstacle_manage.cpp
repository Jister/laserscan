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

#define Pi 		3.141592653
#define REGION	5.0

using namespace std;
using namespace Eigen;

class ObstacleProcess
{
public: 
	ObstacleProcess();
private:
	ros::NodeHandle n;
	ros::Subscriber obstacle_sub;
	ros::Subscriber localposition_sub;
	ros::Publisher obstacle_cloud_pub;

	vector<Vector2f> obstacle_set;
	Vector2f local_pos;
	double yaw;

	bool initialized;
	bool local_pos_update;

	int update_count;
	
	void obstacle_Callback(const geometry_msgs::PointStamped msg);
	void local_position_Callback(const geometry_msgs::PoseStamped &msg);
	rotate_2D(double yaw,  const Vector2f& input,  Vector2f& output);
};

ObstacleProcess::ObstacleProcess()
{
	obstacle_sub = n.subscribe("/obstacle_position", 1, &ObstacleProcess::obstacle_Callback, this);
	localposition_sub = nh.subscribe("/mavros/local_position/local", 1, ObstacleProcess::local_position_Callback);
	obstacle_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/obstacle_cloud", 1);
	initialized = false;
	local_pos_update = false;
	update_count = 0;
}

void ObstacleProcess::obstacle_Callback(const geometry_msgs::PointStamped msg)
{
	Vector2f obstacle_r;
	Vector2f obstacle_local_r;
	Vector2f obstacle_local;
	obstacle_r(0) = msg.point.x;
	obstacle_r(1) = msg.point.y;
	rotate_2D(yaw,obstacle_r,obstacle_local_r);
	obstacle_local = local_pos + obstacle_local_r;

	if(!initialized)
	{
		if(obstacle_r.norm() < REGION)
		{
			obstacle_set.push_back(obstacle_local);
			initialized = true;
		}
	}else
	{
		bool new_obstacle = false;

		//decide whether the obstacle is the same as the point in the set
		for(int i = 0; i < obstacle_set.size(); i++)
		{
			if((obstacle_local - obstacle_set[i]).norm() < 0.5)
			{
				continue;
			}else
			{
				if(obstacle_r.norm() < REGION)
				{
					new_obstacle = true;					
				}
			}
		}

		//push the new obstacle to the set
		if(new_obstacle)
		{
			obstacle_set.push_back(obstacle_local);
		}

		//decide whether the vehicle is still in region of old obstacle 
		for(vector<Vector2f>::iterator it = obstacle_set.begin(); it != obstacle_set.end(); )
		{
			if(( *it - local_pos).norm() > REGION)
			{
				it = obstacle_set.erase(it);
			}else
			{
				it++;
			}
		}
	}

	if(obstacle_set.size() > 0)
	{
		sensor_msgs::PointCloud obstacle_msg;
		obstacle_msg.header.stamp = msg.header.stamp;
		obstacle_msg.header.frame_id = "world";

		obstacle_msg.points.resize(obstacle_set.size());
		for(int i = 0; i < obstacle_set.size(); i++)
		{
			obstacle_msg.points[i].x = obstacle_set[i][0];
			obstacle_msg.points[i].y = obstacle_set[i][1];
			obstacle_msg.points[i].z = 0;
		}

		obstacle_cloud_pub.publish(obstacle_msg);
	}else
	{
		sensor_msgs::PointCloud obstacle_msg;
		obstacle_msg.header.stamp = msg.header.stamp;
		obstacle_msg.header.frame_id = "world";

		obstacle_msg.points.resize(1);
		obstacle_msg.points[i].x = 0;
		obstacle_msg.points[i].y = 0;
		obstacle_msg.points[i].z = 0;

		obstacle_cloud_pub.publish(obstacle_msg);
	}

	update_count++;
	if(update_count > 50)
	{
		update_count = 0;
		local_pos_update = false;
	}
	
}

void ObstacleProcess::local_position_Callback(const geometry_msgs::PoseStamped &msg)
{
	local_pos(0) = msg.pose.position.x;
	local_pos(1) = msg.pose.position.y;
	yaw = tf::getYaw(msg.pose.orientation);

	local_pos_update = true;
}

//rotate function from body to local
void ObstacleProcess::rotate_2D(double yaw,  const Vector2f& input,  Vector2f& output)
{
	double sy = sinf(yaw);
	double cy = cosf(yaw);

	Matrix2f R;
	R(0,0) = cy;
	R(0,1) = -sy;
	R(1,0) = sy;
	R(1,1) = cy;

	output = R * input;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_manage");
	ObstacleProcess ObstacleProcess;
	ros::spin();
}
