#ifndef _MLSM_CORE_H
#define _MLSM_CORE_H


#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/tf.h>
#include "MLSM.h"
#include "ICP.h"


using namespace std;
using namespace mlsm;
class MLSMCore {
private:
	string cloudSubscribeTopic_;
	string markerPublishTopic_;
    string positionSubscribeTopic_;
    string orientationSubscribeTopic_;
    string velTopic_;


	double resolution_;
	double sizeXMeters_;
	double sizeYMeters_;
	bool fastUpdate_;
    double verticalElasticity_;
    geometry_msgs::Twist meanVelocity_;
    geometry_msgs::Pose poseChange_;
    geometry_msgs::Pose oldPose_;
    double lastTime_;
    double time_;
    unsigned long int velSamples_;

    bool matching_;

    tf::Quaternion lastOrientation_;
    tf::Point lastPosition_;

	visualization_msgs::MarkerArray markers;


	MLSM map_;

    ros::Subscriber cloudSubscriber_;
    ros::Subscriber orientationSubscriber_;
    ros::Subscriber positionSubscriber_;
    ros::Subscriber matchingSubscriber_;
    ros::Subscriber velSubscriber_;


	ros::Publisher markerPublisher_;
	ros::Publisher mapPublisher_;
    ros::Publisher debugPublisher_;

    ICP ICPSolver_;

    string cloudFrame_;

    double heightLimit_;

public:
	MLSMCore(ros::NodeHandle* n);
	~MLSMCore();

    int addPointCloudToMap(avora_msgs::StampedIntensityCloudPtr cloudMsg);


	void publishMapMarkers();
    void cloudCallback(avora_msgs::StampedIntensityCloudPtr cloudMsg);
    void orientationCallback(const geometry_msgs::QuaternionPtr orientationMsg);
    void lastKnownPoseCallback(const geometry_msgs::PointPtr pointMsg);
    void matchingCallback(const std_msgs::Bool msg);
    void velCallback(nav_msgs::Odometry msg);

};
#endif
