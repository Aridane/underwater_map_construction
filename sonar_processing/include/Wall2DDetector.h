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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <avora_msgs/Wall.h>
#include <avora_msgs/StampedIntensityCloud.h>

#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <nodelet/nodelet.h>

namespace sonar_processing
    {
    typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;

    using namespace std;
    using namespace cv;
    class Wall2DDetector : public nodelet::Nodelet{
    private:
        string mode_; // RANSAC, HOUGH
        int nWalls_;
        intensityCloud::iterator cloudIterator_;
        intensityCloud::Ptr sonarCloud_;
        ros::Subscriber cloudSubscriber_;
        ros::Publisher wallPublisher_;
        ros::Publisher wallCoefficientPublisher_;
        ros::Publisher debugPublisher_;
        image_transport::ImageTransport* it_;
        image_transport::Publisher debugImagePublisher_;
        ros::Publisher markerPublisher_;
        avora_msgs::Wall wallMsg_;

        string cloudSubscribeTopic_;
        string wallPublishTopic_;
        string wallCoefficientsPublishTopic_;

        double RANSACDistance_;
        double RANSACProbability_;
        int RANSACMaxIterations_;
        ros::NodeHandle nh_;

    public:
        virtual void onInit();
        Wall2DDetector();
        ~Wall2DDetector();

        void cloudCallback(avora_msgs::StampedIntensityCloudConstPtr cloudMessagePtr);
        void detectWall(intensityCloud::ConstPtr cloud);
        void publishWall();

    };
};
