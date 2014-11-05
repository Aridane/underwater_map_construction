#include "MLSMCore.h"

MLSMCore::MLSMCore(ros::NodeHandle* n){
	ros::NodeHandle nh("~");
	nh.param("cloudSubscribeTopic", cloudSubscribeTopic_, string("/Sonar/Walls"));
	nh.param("markerPublishTopic", markerPublishTopic_, string("/MLSM/Markers"));
	nh.param("resolution", resolution_, double(0.3));
	nh.param("sizeXMeters", sizeXMeters_, double(60));
	nh.param("sizeYMeters", sizeYMeters_, double(60));
	nh.param("fastUpdate", fastUpdate_, bool("false"));


	map_.init(resolution_,sizeXMeters_,sizeYMeters_);



	cloudSubscriber_ = n->subscribe(cloudSubscribeTopic_.c_str(), 0, &MLSMCore::cloudCallback, this);
	markerPublisher_ = n->advertise<visualization_msgs::MarkerArray>(markerPublishTopic_.c_str(), 1);
}

MLSMCore::~MLSMCore(){

}

int MLSMCore::addPointCloudToMap(intensityCloud cloud){
	map_.addPointCloud(cloud);
	//TODO Publish map
	//Publish markers provisionally for debugging
	//TODO publish markers on service demand
	//TODO Remove old markers??
	markers = map_.getROSMarkers();
	markerPublisher_.publish(markers);
	return 0;
}

void MLSMCore::cloudCallback(sensor_msgs::PointCloud2 cloudMsg){
	intensityCloud cloud;
	pcl::fromROSMsg(cloudMsg,cloud);
	addPointCloudToMap(cloud);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "MLSMManager");
	ROS_INFO("MLSM MANAGER STARTS");
	ros::NodeHandle nh;
	MLSMCore mlsmCore(&nh);
	while(ros::ok()){
		ros::spinOnce();

	}
}
