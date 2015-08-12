#include "OutlierRemover.h"
#include <pluginlib/class_list_macros.h>

namespace sonar_processing
{
OutlierRemover::OutlierRemover(){}

OutlierRemover::~OutlierRemover(){}

void OutlierRemover::onInit()
{
    NODELET_INFO("Initializing outlier remover nodelet...");
    nh_ = getPrivateNodeHandle();

    // Get all the parameters
    nh_.param("mode", mode_, string("RADIUS"));

    nh_.param("cloudsubscribeTopic", cloudSubscribeTopic_, string("/sonar/scan/thresholded"));
    nh_.param("cleanedCloudPublishTopic", cleanedCloudPublishTopic_, string("/sonar/scan/cleaned"));
    nh_.param("minNeighbours", minNeighbours_, int(20));
    nh_.param("minRadius", minRadius_, double(1));
    nh_.param("stddevMulThresh", stddevMulThresh_, double(0));
    nh_.param("meanK", meanK_, int(30));
    nh_.param("keepOrganized", keepOrganized_, bool(true));

    // Subscribe to incoming sonar data from driver
    cloudSubscriber_ = nh_.subscribe(cloudSubscribeTopic_.c_str(), 0, &OutlierRemover::cloudCallback, this);
    cloudPublisher_ = nh_.advertise<avora_msgs::StampedIntensityCloud>(cleanedCloudPublishTopic_.c_str(),1);
    rosCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(cleanedCloudPublishTopic_ + "/cloud",1);
}

void OutlierRemover::cloudCallback(avora_msgs::StampedIntensityCloudPtr cloudMessagePtr){
    NODELET_INFO("Outlier remover got trhe cloud!");

    //Cloud for thresholding
    stampedCloudMsg_.header = cloudMessagePtr->header;

    intensityCloud::Ptr cloudPtr = boost::make_shared<intensityCloud>();
    //Conver message to intensityCloud
    pcl::fromROSMsg(cloudMessagePtr->cloud, *cloudPtr);
    //Remove Outliers
    removeOutliersFromCloud(cloudPtr);

    std::vector<double> stamps;// = cloudMessagePtr->timeStamps;
    int j = 0;
    for (int i=0;i<cloudMessagePtr->timeStamps.size();i++){
        stamps.push_back(cloudMessagePtr->timeStamps.at(i));
    }
    //Publish cloud
    publishCloud(cloudPtr, stamps);
}

void OutlierRemover::removeOutliersFromCloud(intensityCloud::Ptr cloudPtr)
{
    if ((cloudPtr->isOrganized()) || (mode_.find("STATISTICAL") != -1)){
        ROS_INFO("STATISTICAL");
        // Create the filtering object
         pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
         sor.setInputCloud (cloudPtr);
         // Set the number of nearest neighbors to use for mean distance estimation.
         sor.setMeanK (meanK_);
         /* Set the standard deviation multiplier for the distance threshold calculation.
             The distance threshold will be equal to: mean + stddev_mult * stddev.
             Points will be classified as inlier or outlier if their average neighbor
             distance is below or above this threshold respectively.
         */
         sor.setStddevMulThresh (stddevMulThresh_);
         sor.setKeepOrganized(keepOrganized_);
         sor.filter (*cloudPtr);
    }else {
        /*
         * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         * !!  RadiusOutlierRemoval uses KdTrees (FLANN) to do nearest neighbor search,     !!
         * !!  and passing NAN data to FLANN makes no sense.                                !!
         * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        */

        ROS_INFO("RADIUS");
        // Create the radius outlier removal filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier_removal;
        // Set input cloud
        //radius_outlier_removal.setKeepOrganized(false);
        radius_outlier_removal.setInputCloud(cloudPtr);
        // Set radius for neighbor search
        radius_outlier_removal.setRadiusSearch(minRadius_);
        // Set threshold for minimum required neighbors neighbors
        radius_outlier_removal.setMinNeighborsInRadius(minNeighbours_);
        radius_outlier_removal.setKeepOrganized(keepOrganized_);
        // Do filtering
        radius_outlier_removal.filter(*cloudPtr);

    }
}


void OutlierRemover::publishCloud(intensityCloud::Ptr cloudPtr, std::vector<double> timeStamps){
    pcl::toROSMsg(*cloudPtr,stampedCloudMsg_.cloud);
    stampedCloudMsg_.header.frame_id = cloudPtr->header.frame_id;
    stampedCloudMsg_.timeStamps = timeStamps;
    rosCloudPublisher_.publish(stampedCloudMsg_.cloud);
    cloudPublisher_.publish(stampedCloudMsg_);
}

PLUGINLIB_DECLARE_CLASS(sonar_processing, OutlierRemover, sonar_processing::OutlierRemover, nodelet::Nodelet)
}
