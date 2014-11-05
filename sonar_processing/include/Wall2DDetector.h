#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cmath>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;

using namespace std;

class Wall2DDetector {
private:
	string mode_; // RANSAC, HOUGH
	int nWalls_;
	intensityCloud::iterator cloudIterator_;
	intensityCloud::Ptr sonarCloud_;
	ros::Subscriber cloudSubscriber_;
	ros::Publisher wallPublisher_;
	ros::Publisher debugPublisher_;

	string cloudSubscribeTopic_;
	string wallPublishTopic_;


public:
	Wall2DDetector(ros::NodeHandle* n);
	~Wall2DDetector();
	
	void cloudCallback(sensor_msgs::PointCloud2 cloudMsg);
	void detectWall(intensityCloud::ConstPtr cloud);
	void publishWall();

};
