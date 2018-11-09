#include <ros/ros.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// current heading in degrees
float yaw = 0.0f;
float yawRad = 0.0f;

const float kFOV = 70.0f;
const float kWidth = 410.0f;
const float kMarker1 = 15.0f;
const float kMarker2 = 35.0f;
const int kMarkerWidth = 10;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::Quaternion tfQ;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, tfQ);

	yawRad = (float)tfQ.getAngle();
    yaw = (float)angles::to_degrees(yawRad);
}

void makeMarkerAt(visualization_msgs::Marker& msg, float x, float y, float z, int id)
{
	msg.header.frame_id = std::string("base_scan");
	msg.header.stamp = ros::Time::now();
	msg.ns = "Tracked";
	msg.id = id;
	msg.type = visualization_msgs::Marker::CUBE;
	msg.scale.x = 0.10;
	msg.scale.y = 0.10;
	msg.scale.z = .01;
	msg.color.r = 0.0f;
	msg.color.g = 1.0f;
	msg.color.b = 0.0f;
	msg.color.a = 0.5f;
	msg.lifetime = ros::Duration();
	msg.pose.position.z = z;
	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 1.0;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 1.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_laser_publisher");
    ros::NodeHandle n;
    sensor_msgs::LaserScan scan;
    ros::Time lastScanTime = ros::Time::now();
    ros::Rate publishTime(10);

    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::Publisher detect_pub = n.advertise<visualization_msgs::MarkerArray>("tracked_objects", 1);

	ros::Subscriber odomSub = n.subscribe("odom", 500, odomCallback);

	bool added = false;

    while (ros::ok())
    {
        ros::Time currentTime = ros::Time::now();
        ros::Duration increment = lastScanTime - currentTime;

        scan.header.frame_id = std::string("base_scan");
        scan.angle_increment = (float)(M_PI / 180.0);
        scan.angle_min = 0.0f;
        scan.angle_max = 2.0f*M_PI - scan.angle_increment;
        scan.range_min = 0.12f;
        scan.range_max = 3.5f;
        scan.time_increment = (float)increment.toSec();
        scan.header.stamp = ros::Time::now();
        scan.ranges.resize(360);
        scan.intensities.resize(360);

		float imageDegStart = 360.0f - kFOV / 2.0f;
		float imageDegEnd = kFOV / 2.0f;

        for (int index = 0; index < 360; index++)
        {
            if (index > yaw + imageDegStart + kMarker1 - kMarkerWidth / 2 && index < yaw + imageDegStart + kMarker1 + kMarkerWidth /2)
            {
                scan.ranges[index] = scan.range_min * 2.0f;
				scan.intensities[index] = scan.range_min * 2.0f;
			}
			else if (index > yaw + imageDegStart + kMarker2 - kMarkerWidth / 2 && index < yaw + imageDegStart + kMarker2 + kMarkerWidth / 2)
			{
				scan.ranges[index] = scan.range_min * 2.5f;
				scan.intensities[index] = scan.range_min * 2.5f;
			}
			else
            {
                scan.ranges[index] = scan.range_max / 2.0f;
				scan.intensities[index] = 1.0f;
			}
        }

        lastScanTime = currentTime;
        laser_pub.publish(scan);

        visualization_msgs::MarkerArray msgArray;

		visualization_msgs::Marker msg;
		makeMarkerAt(msg, kMarker1, 0, 0, 0);
		msg.action = added ? visualization_msgs::Marker::MODIFY : visualization_msgs::Marker::ADD;
		msgArray.markers.push_back(msg);

		makeMarkerAt(msg, kMarker2, 0, 0, 1);
		msg.action = added ? visualization_msgs::Marker::MODIFY : visualization_msgs::Marker::ADD;
		msgArray.markers.push_back(msg);

		detect_pub.publish(msgArray);

		added = true;

        ros::spinOnce();
        publishTime.sleep();
    }
    return 0;
}
