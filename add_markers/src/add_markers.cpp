#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>

static const float pickup[3] = {3.0, 5.0, 1.0};
static const float dropoff[3] = {-1.0, 0.0, 1.0};
static float eps[2] = {0.4, 0.03};

static const uint32_t shape = visualization_msgs::Marker::CUBE;

// States
static bool at_pickup = false;
static bool at_dropoff = false;

static bool reached(const float target[3], const nav_msgs::Odometry::ConstPtr &msg)
{
    return std::abs(target[0] - msg->pose.pose.position.x) < eps[0] && std::abs(target[1] - msg->pose.pose.position.y) < eps[0] && std::abs(target[2] - msg->pose.pose.orientation.w) < eps[1];
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    at_pickup = reached(pickup, msg);
    at_dropoff = reached(dropoff, msg);
}

static visualization_msgs::Marker createMarker(const float target[3], const uint8_t action)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = target[0];
    marker.pose.position.y = target[1];
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = target[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

int main(int argc, char **argv)
{
    ROS_INFO("Main");
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

    while (ros::ok())
    {
        bool pickup_done = false;
        bool dropoff_done = false;

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        ROS_INFO("Publishing pick-up marker");
        marker_pub.publish(createMarker(pickup, visualization_msgs::Marker::ADD));

        while (!at_pickup)
            ros::spinOnce();
        if (!pickup_done)
        {
            ROS_INFO("Publishing pick-up marker removal");
            marker_pub.publish(createMarker(pickup, visualization_msgs::Marker::DELETE));
            pickup_done = true;
        }

        while (!at_dropoff)
            ros::spinOnce();
        if (!dropoff_done)
        {
            ROS_INFO("Publishing marker to be displayed");
            marker_pub.publish(createMarker(dropoff, visualization_msgs::Marker::ADD));
            dropoff_done = true;
            ros::Duration(10.0).sleep();
        }
    }
    return 0;
}