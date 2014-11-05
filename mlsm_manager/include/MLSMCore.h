#ifndef _MLSM_CORE_H
#define _MLSM_CORE_H


#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "MLSM.h"


using namespace std;

class MLSMCore {
private:
	string cloudSubscribeTopic_;
	string markerPublishTopic_;
	double resolution_;
	double sizeXMeters_;
	double sizeYMeters_;
	bool fastUpdate_;

	visualization_msgs::MarkerArray markers;


	MLSM map_;

	ros::Subscriber cloudSubscriber_;
	ros::Publisher markerPublisher_;
	ros::Publisher mapPublisher_;

public:
	MLSMCore(ros::NodeHandle* n);
	~MLSMCore();

	int addPointCloudToMap(intensityCloud cloud);

	void publishMapMarkers();
	void cloudCallback(sensor_msgs::PointCloud2 cloudMsg);
};
#endif
