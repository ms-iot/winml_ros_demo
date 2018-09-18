#define _USE_MATH_DEFINES
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1 // The C++ Standard doesn't provide equivalent non-deprecated functionality yet.

#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

float targetYaw = 0.0f;
bool haveLidar = false;
std::vector<float> lidarIntensities;

// Lidar data starts at Zero straight ahead, 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	lidarIntensities = msg->intensities;
	haveLidar = true;
}

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	if (!haveLidar && lidarIntensities.size() < 360)
		return;

	float closestMarkDistance = FLT_MAX;

	for (auto &marker : msg->markers)
	{
		// iterate over each marker. Compre with lidarIntensities.
		// Each marker's x & y is relative to the camera frame. However,
		// the camera frame zero starts at -45 degress, and goes to 45 degress.
		// Just looking at the X, find the intensity for it. Compare with the closest mark.

		int lidarIndex = 360 * (marker.pose.position.x / 640.0f) - 45;

		float distance = lidarIntensities[lidarIndex];
		if (distance < closestMarkDistance)
		{
			float closestYawDegree = (float)lidarIndex;
			closestMarkDistance = lidarIntensities[lidarIndex];
			targetYaw = (float)angles::from_degrees(closestYawDegree);
		}
	}
}

const float kTolerance = 0.001f;
const float kAngleScale = 5.0f;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ml_point_to_closest");

	ros::Rate rate(30);
	ros::NodeHandle n;
	ros::Subscriber lidarSub = n.subscribe("scan", 500, scanCallback);
	ros::Subscriber markerSub = n.subscribe("tracked_objects", 500, markerCallback);
    ros::Publisher velocityPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	float currentYaw = 0.0f;
    while (ros::ok())
    {
		geometry_msgs::Twist vel_msg;
		float yaw_error = angles::normalize_angle(currentYaw - targetYaw);

		if (fabs(yaw_error) > kTolerance)
		{
			vel_msg.angular.z = kAngleScale * yaw_error;
			velocityPub.publish(vel_msg);
		}

		rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
