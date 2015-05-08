#include "driver_depth.h"

DriverDepth::DriverDepth(ros::NodeHandle* n){
    ros::NodeHandle nh("~");
    nh.param("depthSubscribeTopic", depthSubscribeTopic_, string("/odom"));
    nh.param("targetFrame", targetFrame_, string("depth"));

    odomSubscriber_ = n->subscribe(depthSubscribeTopic_.c_str(), 1, &DriverDepth::depthCallback, this);
}

DriverDepth::~DriverDepth(){

}

void DriverDepth::depthCallback(nav_msgs::Odometry odomMsg){
    transform_.setOrigin( tf::Vector3(0, 0, -odomMsg.pose.pose.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "base_link", "depth"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "DriverDepth");
    ROS_INFO("Depth driver STARTS");
    ros::NodeHandle nh;
    DriverDepth depth(&nh);
    while(ros::ok()){
        ros::spinOnce();

    }
}
