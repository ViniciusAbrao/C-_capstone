#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Pose pickup;
geometry_msgs::Pose dropoff;
bool preparing = true;
visualization_msgs::Marker marker;
visualization_msgs::Marker dropoffMARKER;
ros::Publisher marker_pub;
int count_pick=1;
int count_drop=1;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    const bool picking_up = (abs(pickup.position.x - msg->pose.pose.position.x) + abs(pickup.position.y - msg->pose.pose.position.y)) < 1;

    const bool dropping_off = (abs(dropoff.position.x - msg->pose.pose.position.x) + abs(dropoff.position.y - msg->pose.pose.position.y)) < 1;

    if(preparing){
        marker_pub.publish(marker);
        ROS_INFO("preparing");
     }

    if(picking_up){
        preparing = false;
        if (count_pick==1){
          sleep(5);
          count_pick=2;
        }
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("picking up");
    } else if(dropping_off){
        if (count_drop==1){
          sleep(5);
          count_drop=2;
        }
        dropoffMARKER.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(dropoffMARKER);
        ROS_INFO("dropping off");
    }
}




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
    ros::Subscriber marker_sub = n.subscribe("/odom",1000,odomCallback);
    // Set our initial shape type to be a sphere 
    uint32_t shape = visualization_msgs::Marker::SPHERE;


    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();


    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;
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


    //std::cout << "MAIN" << std::endl;
    marker_pub.publish(marker);


    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

}
