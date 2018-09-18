#define _USE_MATH_DEFINES
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msg/Pose.h>
#include <geometry_msgs/Twist.h>

float targetYaw = 0.0f;
bool haveLidar = false;
std::vector<float> lidarIntensities;

// Lidar data starts at Zero straight ahead, 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	lidarIntensities = msg.intensities;
	haveLidar = true;
}

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	if (!haveLidar && lidarIntensities.size < 360)
		return;

	float closestYawDegree = 0;
	float closestMarkDistance = FLT_MAX;
	visualization_msgs::Marker* closestMark = nullptr;

	for (auto &marker : msg.markers)
	{
		// iterate over each marker. Compre with lidarIntensities.
		// Each marker's x & y is relative to the camera frame. However,
		// the camera frame zero starts at -45 degress, and goes to 45 degress.
		// Just looking at the X, find the intensity for it. Compare with the closest mark.

		int lidarIndex = 360 * (marker.pose.x / 640.0f) - 45;

		float distance = lidarIntensities[lidarIndex];
		if (distance < closestMarkDistance)
		{
			closestYawDegree = (float)lidarIndex;
			closestMarkDistance = lidarIntensities[lidarIndex];
			closestMark = maker;
		}
	}

	if (closestMark != nullptr)
	{
		targetYaw = angles::from_degrees(closestYawDegree);
	}
}

const float kTolerance = 0.001;
const float kAngleScale = 5.0;

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
		ros::Twist vel_msg;
		float yaw_error = angles::normalize_angle(currentYaw - targetYawDegree);

		if (fabs(yaw_error) > kTolerance)
		{
			vel_msg.angular = kAngleScale * yaw_error;
			velocityPub.publish(vel_msg);
		}

		rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
