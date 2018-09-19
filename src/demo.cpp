#define _USE_MATH_DEFINES
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1 // The C++ Standard doesn't provide equivalent non-deprecated functionality yet.

#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

float targetYaw = 0.0f;

bool haveLidar = false;
std::vector<float> lidarRanges;
const float kFOV = 70.0f;  // move to param
const float kWidth = 410;
const float kMinDistance = 0.1f;
const float kMaxDistance = 1.0f;

bool useLidarOnly = true;

// Lidar data starts at Zero straight ahead
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lidarRanges = msg->ranges;
	haveLidar = true;
	if (useLidarOnly)
	{
		float degreePerIndex = 360.0f / lidarRanges.size();

		// Zero is forward. Start from FOV loop around
		int swingIndicies = (int)(kFOV / degreePerIndex);
		int startingIndex = -swingIndicies / 2;

		targetYaw = 0.0f;

		float closestMarkDistance = FLT_MAX;
		for (int i = startingIndex; i < swingIndicies; i++)
		{
			size_t index = i < 0 ? lidarRanges.size() + i : i;
			float distance = lidarRanges[index];
			if (distance > kMinDistance &&
				distance < kMaxDistance &&
				distance < closestMarkDistance)
			{
				closestMarkDistance = distance;
				targetYaw = (float)angles::from_degrees(i * degreePerIndex);
			}
		}
	}
}

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	if (useLidarOnly || !haveLidar)
		return;

	float closestMarkDistance = FLT_MAX;

	for (auto &marker : msg->markers)
	{
		// iterate over each marker. Compre with lidarIntensities.
		// Each marker's x & y is relative to the camera frame. However,
		// the camera frame goes from -kFOV/2 degress, and goes to kFOV/2 degrees.
		// Just looking at the X, find the intensity for it. Compare with the closest mark.

		float lidarDegree = angles::to_degrees(angles::normalize_angle_positive(angles::from_degrees(kFOV * (marker.pose.position.x / kWidth) - 45.0f)));

		int index = (int)((lidarDegree / 360.0f) * lidarRanges.size());

		float distance = lidarRanges[index];
		if (distance < closestMarkDistance)
		{
			closestMarkDistance = lidarRanges[index];
			targetYaw = (float)angles::from_degrees(lidarDegree);
		}
	}
}

const float kTolerance = 0.10f;
const float kAngleScale = 0.3f;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ml_point_to_closest");

	ros::NodeHandle n;
	ros::Subscriber lidarSub = n.subscribe("scan", 500, scanCallback);
	ros::Subscriber markerSub = n.subscribe("tracked_objects", 500, markerCallback);
    ros::Publisher velocityPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	ros::Rate rate(30);
    while (ros::ok())
    {
		geometry_msgs::Twist vel_msg;

		if (fabs(targetYaw) > kTolerance)
		{
			vel_msg.angular.z = kAngleScale * targetYaw;
			velocityPub.publish(vel_msg);
		}
		else
		{
			vel_msg.angular.z = 0.0f;
			velocityPub.publish(vel_msg);
		}

		rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
