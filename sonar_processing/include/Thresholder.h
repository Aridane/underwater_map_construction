#include <ros/ros.h>
#include <ros/console.h>

#include <nodelet/nodelet.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <TypeDefinitions.h>
#include <avora_msgs/StampedIntensityCloud.h>

namespace sonar_processing
{
   // typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;

    using namespace std;

    class Thresholder : public nodelet::Nodelet{
    private:
        //Parameters
        string mode_; //OTSU[PROPORTIONAL], FIXED, PROPORTIONAL, MEAN
        int fixedThresholdValue_;
        double maxThresholdProportion_;
        double minThresholdProportion_;
        double OTSUMultiplier_;
        int minThresholdValue_;
        bool keepOrganized_;
        int maxBinValue_;

        avora_msgs::StampedIntensityCloud stampedCloudMsg_;

        //Publishers and subscribers
        ros::Subscriber cloudSubscriber_;
        ros::Publisher cloudPublisher_;
        ros::Publisher rosCloudPublisher_;

        //Topic names
        string cloudSubscribeTopic_;
        string thresholdedCloudPublishTopic_;

        ros::NodeHandle nh_;

    public:
        virtual void onInit();
        Thresholder();
        ~Thresholder();

        void cloudCallback(avora_msgs::StampedIntensityCloudPtr cloudMessagePtr);
        pcl::IndicesConstPtr thresholdCloud(intensityCloud::Ptr cloudPtr);
        double getOTSUThreshold(intensityCloud::Ptr cloudPtr);
        void publishCloud(intensityCloud::Ptr cloudPtr,  std::vector<double> timeStamps);
    };
};
