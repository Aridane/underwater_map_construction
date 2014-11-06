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


namespace sonar_processing
{
    typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;
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

        int maxBinValue_;

        //Publishers and subscribers
        ros::Subscriber cloudSubscriber_;
        ros::Publisher cloudPublisher_;

        //Clouds that later will be converted to the message
        intensityCloud::Ptr cloud_;

        //Topic names
        string cloudSubscribeTopic_;
        string thresholdedCloudPublishTopic_;

        ros::NodeHandle nh_;

    public:
        virtual void onInit();
        Thresholder();
        ~Thresholder();

        void cloudCallback(sensor_msgs::PointCloud2ConstPtr cloudMessagePtr);
        void thresholdCloud(intensityCloud::Ptr cloudPtr);
        double getOTSUThreshold(intensityCloud::Ptr cloudPtr);
        void publishCloud(intensityCloud::Ptr cloudPtr);
    };
};
