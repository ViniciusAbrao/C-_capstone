#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

visualization_msgs::Marker marker;
visualization_msgs::Marker dropoffMARKER;
ros::Publisher marker_pub;
geometry_msgs::Pose pickup;
geometry_msgs::Pose dropoff;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pickup.position.x = -6;
    pickup.position.y = -3;
    dropoff.position.x = 2;
    dropoff.position.y = -2;

    // Set our initial shape type to be a sphere 
    uint32_t shape = visualization_msgs::Marker::SPHERE;


    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();


    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = shape;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker.pose.position.x = pickup.position.x;
    marker.pose.position.y = pickup.position.y;

    dropoffMARKER = marker;
    dropoffMARKER.id = 1;
    dropoffMARKER.pose.position.x = dropoff.position.x;
    dropoffMARKER.pose.position.y = dropoff.position.y;

    sleep(5);
    ROS_INFO("pick up");
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    
    sleep(5);
    ROS_INFO("delete");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    
    sleep(5);
    ROS_INFO("drop off");
    dropoffMARKER.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(dropoffMARKER);


}
