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
const float kCameraPixelWidth = 416.0f;
const float kDegreePerCameraIndex = kFOV / kCameraPixelWidth;
const float kMinDistance = 1.0f;
const float kMaxDistance = 3.0f;

ros::Time g_lastSeen;
bool useLidarOnly = false;
ros::Duration g_lastSeenTimeout(2);

// Lidar data starts at Zero straight ahead
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lidarRanges = msg->ranges;
	haveLidar = true;
	if (useLidarOnly)
	{
		float degreePerIndex = 360.0f / lidarRanges.size();

		// Zero is forward. Start from FOV loop around
		int swingIndices = (int)(kFOV / degreePerIndex);
		int startingIndex = -swingIndices / 2;

		targetYaw = 0.0f;

		float closestMarkDistance = FLT_MAX;
		for (int degreeIndex = startingIndex; degreeIndex < swingIndices; degreeIndex++)
		{
			size_t index = degreeIndex < 0 ? lidarRanges.size() + degreeIndex : degreeIndex;
			float distance = lidarRanges[index];
			if (distance > kMinDistance &&
				distance < kMaxDistance &&
				distance < closestMarkDistance)
			{
				closestMarkDistance = distance;
				targetYaw = (float)angles::from_degrees(degreeIndex * degreePerIndex);
			}
		}
	}
}

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	if (useLidarOnly || !haveLidar)
		return;

	float closestMarkerDistance = FLT_MAX;
    float degreePerLidarIndex = 360.0f / lidarRanges.size();

    targetYaw = 0.0f;

	for (auto &marker : msg->markers)
	{
        // Map the centroid position reported by the marker to Lidar index
        float percentInImage = marker.pose.position.x / kCameraPixelWidth;

        // Zero is forward for the Lidar. Camera is centered around zero.
        int swingIndices = (int)(kFOV / degreePerLidarIndex);
        int degreeIndex = (int)(percentInImage * swingIndices - swingIndices / 2);

        size_t index = degreeIndex < 0 ? lidarRanges.size() + degreeIndex : degreeIndex;
        float distance = lidarRanges[index];
        
        if (distance > kMinDistance &&
            distance < kMaxDistance &&
            distance < closestMarkerDistance)
        {
            closestMarkerDistance = distance;
			targetYaw = (float)angles::from_degrees(- degreeIndex * degreePerLidarIndex);
            g_lastSeen = ros::Time::now();
        }
	}

    ROS_INFO("Turning Towards %f", targetYaw);
}

const float kTolerance = 0.05f;
const float kAngleScale = 0.3f;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ml_point_to_closest");

	ros::NodeHandle n;
	ros::Subscriber lidarSub = n.subscribe("scan", 500, scanCallback);
	ros::Subscriber markerSub = n.subscribe("tracked_objects", 500, markerCallback);
    ros::Publisher velocityPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    g_lastSeen = ros::Time::now();

	ros::Rate rate(30);
    while (ros::ok())
    {
		geometry_msgs::Twist vel_msg;
        if (ros::Time::now() - g_lastSeen > g_lastSeenTimeout ||
            fabs(targetYaw) < kTolerance)
        {
            vel_msg.angular.z = 0.0f;
            velocityPub.publish(vel_msg);
        }
        else
		{
			vel_msg.angular.z = kAngleScale * targetYaw;
			velocityPub.publish(vel_msg);
		}

		rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
