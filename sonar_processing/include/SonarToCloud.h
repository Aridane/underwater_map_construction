#include <ros/ros.h>
#include <ros/console.h>

#include <nodelet/nodelet.h>

#include <avora_msgs/SonarScanLine.h>
#include <avora_msgs/SonarScan.h>
#include <avora_msgs/SonarScanCloud.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <string>


namespace sonar_processing
{
	typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;

	using namespace std;

	class SonarToCloud : public nodelet::Nodelet{
	private:
        //Parameters
        string mode_;
		int sonarCloudSize_;
		int sonarCloudNBeams_;
        int laserCloudNBeams_;
		int scanSize_;

		vector<double> scanAngles;
		intensityCloud::iterator cloudIterator_;
        avora_msgs::SonarScanLineConstPtr oldScanLine_;
        //Publishers and subscribers
        ros::Subscriber beamSubscriber_;
	    ros::Publisher laserCloudPublisher_;
		ros::Publisher sonarCloudPublisher_;
		
        //Clouds that later will be converted to the message
        intensityCloud::Ptr laserCloud_;
        intensityCloud::Ptr sonarCloud_;
		
        //Topic names
        string sonarSubscribeTopic_;
		string sonarCloudPublishTopic_;
		string laserCloudPublishTopic_;
		
		ros::NodeHandle nh_;


	public:
		virtual void onInit();
		SonarToCloud();
		~SonarToCloud();
		void laserInit();
		void sonarInit();
	
		void beamCallback(avora_msgs::SonarScanLineConstPtr scanLine);
        bool newAngle(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine);
        void publishSonarCloud();
        void publishLaserCloud();
	};	
};

