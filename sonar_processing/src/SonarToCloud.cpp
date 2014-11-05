#include "SonarToCloud.h"
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(sonar_processing::SonarToCloud, nodelet::Nodelet)
//PLUGINLIB_DECLARE_CLASS(sonar_processing, SonarToCloud, sonar_processing::SonarToCloud, nodelet::Nodelet)
namespace sonar_processing
{
SonarToCloud::SonarToCloud(){}

SonarToCloud::~SonarToCloud(){}

void SonarToCloud::onInit()
{
    NODELET_INFO("Initilizing SonarToCloud nodelet...");
    nh_ = getPrivateNodeHandle();

    // Get all the parameters
    nh_.param("mode", mode_, string("SONAR")); // {SONAR | LASER}[CONT]
    nh_.param("scanSize", scanSize_, int(160)); // Size in beams of the cloud that will be published
    nh_.param("sonarSubscribeTopic", sonarSubscribeTopic_, string("/Sonar/raw"));
    nh_.param("sonarCloudPublishTopic", sonarCloudPublishTopic_, string("/sonar/scan/sonarCloud"));
    nh_.param("laserCloudPublishTopic", laserCloudPublishTopic_, string("/sonar/scan/laserCloud"));


    // Subscribe to incoming sonar data from driver
    beamSubscriber_ = nh_.subscribe(sonarSubscribeTopic_.c_str(), 0, &SonarToCloud::beamCallback, this);

    // Depending on the mode selected we subscribe advertise the topic needed
    // and initialise the corresponding variables
    if (mode_.find("LASER") != -1) laserInit();
    if (mode_.find("SONAR") != -1) sonarInit();
}

void SonarToCloud::laserInit(){
    laserCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(laserCloudPublishTopic_.c_str(), 1);
    laserCloud_ = boost::make_shared<intensityCloud>();
    laserCloudNBeams_ = 0;
}

void SonarToCloud::sonarInit(){
    sonarCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(sonarCloudPublishTopic_.c_str(), 1);

    sonarCloud_ = boost::make_shared<intensityCloud>();
    sonarCloudSize_ = 0;
    sonarCloudNBeams_ = 0;
    sonarCloud_->is_dense = true;
}

bool SonarToCloud::newAngle(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine){
    return (scan->sensorAngle != oldScanLine->sensorAngle);
}

void SonarToCloud::beamCallback(avora_msgs::SonarScanLineConstPtr scanLine){
    pcl::PointXYZI point;
    NODELET_INFO("BeamCallback");
    if (mode_.find("LASER") != -1){
        NODELET_INFO("Mode Laser Scan size %d", scanSize_);
        // If we have enough readings for the current cloud, or the angle of the sensor has changed
        // we process and send it
        if ((oldScanLine_ != NULL) && (newAngle(scanLine, oldScanLine_)
                || (laserCloudNBeams_ >= scanSize_))){
            NODELET_INFO("Scan full with %d beams", scanSize_);
            publishLaserCloud();
            if ((mode_.find("CONT") != -1) && !(newAngle(scanLine, oldScanLine_))){
                laserCloud_->erase(laserCloud_->begin(),laserCloud_->begin()+1);
                laserCloudNBeams_--;
            }
            else{
                laserCloud_->clear();
                laserCloudNBeams_ = 0;
            }
        }
        // We conver the readings of the new beam to intensity points and insert them into the new point cloud
        int max = 0;
        int maxIndex = 0;
        for (int i=0;i<scanLine->intensities.size();i++){
            if (scanLine->intensities[i] > max){
                max = scanLine->intensities[i];
                maxIndex = i;
            }
        }
        //TODO Check if sonar still publishes empty range resolutions
        double rangeResolution = (scanLine->range_resolution == 0) ? 30.0 / 500.0 : scanLine->range_resolution;
        //Here we get where the point is according to the angle read by the sonar. So we have its coordinates
        // in the same plane as the sensor.
        point.x = (maxIndex + 1) * rangeResolution * cos(scanLine->angle);
        point.y = (maxIndex + 1) * rangeResolution * sin(scanLine->angle);
        point.z = 0;
        // And then transform them to the robot frame, which means rotating -sensorAngle in the Y axis
        point.x = point.x * cos(-scanLine->sensorAngle);
        // Since the rotation is aroung Y, the Y coordinate is not modified
        point.z = point.x * sin(-scanLine->sensorAngle);

        point.intensity = scanLine->intensities[maxIndex];

        //We add the point to the cloud that later will be converted to the ROS message
        if (point.intensity) laserCloud_->push_back(point);
        laserCloudNBeams_++;
    }
    if (mode_.find("SONAR") != -1){
        NODELET_INFO("Mode Sonar Scan size %d", scanSize_);
        if ((oldScanLine_ != 0) && (newAngle(scanLine, oldScanLine_)
                || (sonarCloudNBeams_ == scanSize_))) {
            NODELET_INFO("Scan full with %d beams", scanSize_);

            sonarCloud_->height = sonarCloudNBeams_;
            sonarCloud_ ->width = oldScanLine_->intensities.size();
            sonarCloud_->resize(sonarCloud_->height*sonarCloud_ ->width);
            publishSonarCloud();
            // If we are in continuous mode we just delete the oldest beam of the cloud and insert the new one
            if ((mode_.find("CONT") != -1) && !(newAngle(scanLine, oldScanLine_))){
                sonarCloud_->erase(sonarCloud_->begin(),sonarCloud_->begin()+sonarCloud_->width);
                sonarCloudNBeams_--;
                sonarCloudSize_ -= sonarCloud_->width;

            }
            else {
                sonarCloud_->clear();
                sonarCloudSize_ = 0;
                sonarCloudNBeams_ = 0;
            }
        }
        double rangeResolution = (scanLine->range_resolution == 0) ? 30.0 / 500.0 : scanLine->range_resolution;

        for (int i=0;i<scanLine->intensities.size();i++){
            point.x = (i + 1) * rangeResolution * cos(scanLine->angle);
            point.y = (i + 1) * rangeResolution * sin(scanLine->angle);
            point.z = 0;
            // And then transform them to the robot frame, which means rotating -sensorAngle in the Y axis
            point.x = point.x * cos(-scanLine->sensorAngle);
            // Since the rotation is aroung Y, the Y coordinate is not modified
            point.z = point.x * sin(-scanLine->sensorAngle);

            point.intensity = scanLine->intensities[i];

            sonarCloud_->push_back(point);
            sonarCloudSize_++;
        }
        sonarCloudNBeams_++;
    }
    oldScanLine_ = boost::make_shared<avora_msgs::SonarScanLine>(*scanLine);
}
//Now that we have the sonar cloud in "sonarCloud" we convert it to a ROS message and publish it
void SonarToCloud::publishSonarCloud(){
    NODELET_INFO("Publish Sonar Cloud");
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(*sonarCloud_,cloudMessage);
    cloudMessage.header.frame_id = "/map";
    sonarCloudPublisher_.publish(cloudMessage);
}

void SonarToCloud::publishLaserCloud(){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(*laserCloud_,cloudMessage);
    cloudMessage.header.frame_id = "/map";
    laserCloudPublisher_.publish(cloudMessage);
}

PLUGINLIB_DECLARE_CLASS(sonar_processing, SonarToCloud, sonar_processing::SonarToCloud, nodelet::Nodelet)
}
