#include <ros/ros.h>
#include <ros/console.h>

#include <avora_msgs/SonarScanLine.h>
#include <avora_msgs/SonarScan.h>
#include <avora_msgs/SonarScanCloud.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/poisson.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;

using namespace std;

class ScanProcessor {
private:
	string mode_; // LASER, SONAR, LASER+SONAR
	string thresholdMode_; //NONE, FIXED OTSU PROPOTIONAL
	string upsamplingMode_; //NONE, BILATERAL, MLS (+DISTINCT_CLOUD | +SAMPLE_LOCAL_PLANE), Poisson
							//				 (+RANDOM_UNIFORM_DENSITY | +VOXEL_GRID_DILATION )
	string outlierRemovalMode_;// NONE, STATISTICAL, RADIUS
	int thresholdValue_;
	double thresholdProportion_;
	double OTSUMultiplier_;
	int maxBinValue_;
	int sonarCloudSize_;
	int sonarCloudNBeams_;
	int scanSize_;

	vector<double> scanAngles;

	avora_msgs::SonarScanCloudPtr laserCloudMsg_;
	avora_msgs::SonarScanCloudPtr sonarCloudMsg_;
	avora_msgs::SonarScanLineConstPtr oldScanLine_;
	intensityCloud::iterator cloudIterator_;
	intensityCloud::Ptr laserCloud_;
	intensityCloud::Ptr sonarCloud_;
	ros::Subscriber beamSubscriber_;
	ros::Publisher laserCloudPublisher_;
	ros::Publisher sonarCloudPublisher_;
	ros::Publisher sonarDebugCloudPublisher_;

	string beamSubscribeTopic_;
	string sonarSubscribeTopic_;
	
	string sonarCloudPublishTopic_;
	string laserCloudPublishTopic_;


public:
	ScanProcessor(ros::NodeHandle* n);
	~ScanProcessor();
	void laserInit(ros::NodeHandle* n);
	void sonarInit(ros::NodeHandle* n);
	
	void beamCallback(avora_msgs::SonarScanLineConstPtr scanLine);
	bool hasChanged(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine);
	void thresholdCloud(intensityCloud::Ptr cloud);
	void removeCloudOutliers(intensityCloud::Ptr cloud);
	intensityCloud upSampleCloud(intensityCloud::Ptr cloud, int newDimX, int newDimY, intensityCloud::Ptr originalCloud = intensityCloud::Ptr());
	intensityCloud polar2Cartesian(intensityCloud::Ptr cloud);
	void processLaserCloud();
	void publishLaserCloud();
	void processSonarCloud();
	void publishSonarCloud();

	void intensityToRGB(intensityCloud::Ptr cloudIn, rgbCloud::Ptr rgbCloud);
};
