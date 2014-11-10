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
    nh_.param("mode", mode_, string("STATISTICAL"));

    nh_.param("cloudsubscribeTopic", cloudSubscribeTopic_, string("/sonar/scan/thresholded"));
    nh_.param("cleanedCloudPublishTopic", cleanedCloudPublishTopic_, string("/sonar/scan/cleaned"));

    // Subscribe to incoming sonar data from driver
    cloudSubscriber_ = nh_.subscribe(cloudSubscribeTopic_.c_str(), 0, &OutlierRemover::cloudCallback, this);
}

void OutlierRemover::cloudCallback(sensor_msgs::PointCloud2ConstPtr cloudMessagePtr){
    //Cloud for thresholding
    intensityCloud::Ptr cloudPtr = boost::make_shared<intensityCloud>();
    //Conver message to intensityCloud
    pcl::fromROSMsg(*cloudMessagePtr, *cloudPtr);
    //Remove Outliers
    removeOutliersFromCloud(cloudPtr);
    //Publish cloud
    publishCloud(cloudPtr);
}

void OutlierRemover::removeOutliersFromCloud(intensityCloud::Ptr cloudPtr)
{
    if (mode_.find("STATISTICAL") != -1){
        // Create the filtering object
         pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
         sor.setInputCloud (cloudPtr);
         // Set the number of nearest neighbors to use for mean distance estimation.
         sor.setMeanK (20);
         /* Set the standard deviation multiplier for the distance threshold calculation.
             The distance threshold will be equal to: mean + stddev_mult * stddev.
             Points will be classified as inlier or outlier if their average neighbor
             distance is below or above this threshold respectively.
         */
         sor.setStddevMulThresh (1);
         sor.setKeepOrganized(false);
         sor.filter (*cloudPtr);
    }else {
        // Create the radius outlier removal filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier_removal;
        // Set input cloud
        //radius_outlier_removal.setKeepOrganized(false);
        radius_outlier_removal.setInputCloud(cloudPtr);
        // Set radius for neighbor search
        radius_outlier_removal.setRadiusSearch(0.8);
        // Set threshold for minimum required neighbors neighbors
        radius_outlier_removal.setMinNeighborsInRadius(20);
        // Do filtering
        radius_outlier_removal.filter(*cloudPtr);

    }
}


void OutlierRemover::publishCloud(intensityCloud::Ptr cloudPtr){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(*cloudPtr,cloudMessage);
    cloudMessage.header.frame_id = "/map";
    cloudPublisher_.publish(cloudMessage);
}

PLUGINLIB_DECLARE_CLASS(sonar_processing, OutlierRemover, sonar_processing::OutlierRemover, nodelet::Nodelet)
}
