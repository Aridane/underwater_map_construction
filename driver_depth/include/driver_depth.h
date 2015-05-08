#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
class DriverDepth{
    string targetFrame_;
    string depthSubscribeTopic_;

    tf::TransformBroadcaster br_;
    tf::Transform transform_;

    ros::Subscriber odomSubscriber_;

public:
    DriverDepth(ros::NodeHandle *n);
    ~DriverDepth();
    void depthCallback(nav_msgs::Odometry odomMsg);
};
