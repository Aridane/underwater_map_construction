#include "Wall2DDetector.h"
using namespace std;

Wall2DDetector::Wall2DDetector(ros::NodeHandle* n) {
	ros::NodeHandle nh("~");
	nh.param("cloudSubscribeTopic", cloudSubscribeTopic_,
			string("/Sonar/Scan/SonarCloud"));
	nh.param("wallPublishTopic", wallPublishTopic_, string("/Sonar/Walls"));
	nh.param("mode", mode_, string("RANSAC"));
	nh.param("nWalls", nWalls_, int(3));


	cloudSubscriber_ = n->subscribe(cloudSubscribeTopic_.c_str(), 0,
			&Wall2DDetector::cloudCallback, this);
	wallPublisher_ = n->advertise<sensor_msgs::PointCloud2>(
			wallPublishTopic_.c_str(), 1);
	debugPublisher_ = n->advertise<sensor_msgs::PointCloud2>(
			"/Debug/Wall", 1);
}

Wall2DDetector::~Wall2DDetector() {
}

void Wall2DDetector::cloudCallback(sensor_msgs::PointCloud2 cloudMsg) {
	if (mode_.find("RANSAC") != -1) {
		intensityCloud::Ptr cloud = boost::make_shared<intensityCloud>();
		pcl::fromROSMsg(cloudMsg,*cloud);
		intensityCloud result,line;

		vector<int> inliers;
		Eigen::VectorXf coefficients;
		sensor_msgs::PointCloud2 cloudMsg2;
		for (int i=0;i<nWalls_;i++){
			// created RandomSampleConsensus object and compute the appropriated model
			pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(
					new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
			ransac.setDistanceThreshold(1);
			ransac.computeModel();
			ransac.getInliers(inliers);
			ransac.setProbability(0.5);
			// copies all inliers of the model computed to another PointCloud
			pcl::copyPointCloud(*cloud,inliers,line);

			pcl::ExtractIndices<pcl::PointXYZI> extractor(true);
			extractor.setInputCloud (cloud);
			extractor.setIndices (boost::make_shared<vector<int> >(inliers));
			extractor.setNegative (true);
		    extractor.filter(*cloud);

			inliers.clear();
			ros::spinOnce();


			pcl::toROSMsg(*(cloud.get()), cloudMsg2);
			cloudMsg2.header.frame_id = "/map";
			debugPublisher_.publish(cloudMsg2);


			result += line;
			line.clear();
			ransac.getModelCoefficients(coefficients);
		}


		pcl::toROSMsg(result, cloudMsg2);
		cloudMsg2.header.frame_id = "/map";
		wallPublisher_.publish(cloudMsg2);


	}
}

void Wall2DDetector::detectWall(intensityCloud::ConstPtr cloud) {

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Wall2DDetector");
	ROS_INFO("2D Wall Detector STARTS");
	ros::NodeHandle nh;

	Wall2DDetector sonar2scan(&nh);

	while (ros::ok()) {
		ros::spinOnce();
	}

}
