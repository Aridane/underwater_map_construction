#include <ros/ros.h>
#include <ros/console.h>

#include <nodelet/nodelet.h>

#include <avora_msgs/SonarScanLine.h>
#include <avora_msgs/SonarScan.h>
#include <avora_msgs/SonarScanCloud.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <avora_msgs/StampedIntensityCloud.h>


#include <TypeDefinitions.h>

#include <string>


namespace sonar_processing
{
	typedef pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;

	using namespace std;

	class SonarToCloud : public nodelet::Nodelet{
	private:
        //Parameters
        string mode_;
        string targetFrame_;
		int sonarCloudSize_;
		int sonarCloudNBeams_;
        int laserCloudNBeams_;
		int scanSize_;

        avora_msgs::StampedIntensityCloud stampedCloudMsg_;
		vector<double> scanAngles;
		intensityCloud::iterator cloudIterator_;
        avora_msgs::SonarScanLineConstPtr oldScanLine_;
        //Publishers and subscribers
        ros::Subscriber beamSubscriber_;
	    ros::Publisher laserCloudPublisher_;
		ros::Publisher sonarCloudPublisher_;
        ros::Subscriber velSubscriber_;

		
        //Clouds that later will be converted to the message
        intensityCloud::Ptr laserCloud_;
        intensityCloud::Ptr sonarCloud_;
		
        //Topic names
        string sonarSubscribeTopic_;
		string sonarCloudPublishTopic_;
		string laserCloudPublishTopic_;
        string scanFlagTopic_;
        string velTopic_;
		
		ros::NodeHandle nh_;

        tf::TransformListener listener_;
        message_filters::Subscriber<avora_msgs::SonarScanLine> scanLine_sub_;
        tf::MessageFilter<avora_msgs::SonarScanLine> * tf_filter_;

        bool moving_;
        double startTime_;
        double accumulatedTime_;
        double heightLimit_;
        geometry_msgs::Pose poseChange_;
        geometry_msgs::Pose oldPose_;
        geometry_msgs::Pose pose_;
        double lastTime_;
        double time_;
        double oldStamp_;

	public:
		virtual void onInit();
		SonarToCloud();
		~SonarToCloud();
		void laserInit();
		void sonarInit();
	
		void beamCallback(avora_msgs::SonarScanLineConstPtr scanLine);
        bool newAngle(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine);
        void velCallback(nav_msgs::Odometry msg);
        void publishSonarCloud();
        void publishLaserCloud();
	};	
};

