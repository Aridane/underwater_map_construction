#include "SonarToCloud.h"
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(sonar_processing::SonarToCloud, nodelet::Nodelet)
//PLUGINLIB_DECLARE_CLASS(sonar_processing, SonarToCloud, sonar_processing::SonarToCloud, nodelet::Nodelet)
namespace sonar_processing
{
SonarToCloud::SonarToCloud():
	listener_()
{}

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
    nh_.param("scanFlagTopic",scanFlagTopic_,string("sonar/scan/flag"));
    nh_.param("laserCloudPublishTopic", laserCloudPublishTopic_, string("/sonar/scan/laserCloud"));
    nh_.param("targetFrame",targetFrame_,string("depth"));
    nh_.param("heightLimit", heightLimit_, double(4.7));
    nh_.param("velSubscribeTopic", velTopic_, string("/odom"));


    // Subscribe to incoming sonar data from driver
    //beamSubscriber_ = nh_.subscribe(sonarSubscribeTopic_.c_str(), 0, &SonarToCloud::beamCallback, this);
	scanLine_sub_.subscribe(nh_,sonarSubscribeTopic_.c_str(),10);
    tf_filter_ = new tf::MessageFilter<avora_msgs::SonarScanLine>(scanLine_sub_, listener_, targetFrame_, 120);
	tf_filter_->registerCallback( boost::bind(&SonarToCloud::beamCallback, this, _1) );
    velSubscriber_ = nh_.subscribe(velTopic_.c_str(), 1, &SonarToCloud::velCallback, this);
    // Depending on the mode selected we subscribe advertise the topic needed
    // and initialise the corresponding variables
    if (mode_.find("LASER") != -1) laserInit();
    if (mode_.find("SONAR") != -1) sonarInit();

    accumulatedTime_ = 0;
    startTime_ = 0;

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

/*void SonarToCloud::velCallback(geometry_msgs::Twist::Ptr twistMessage){
    ROS_INFO("Speed x = %.2f y = %.2f z = %.2f",twistMessage->linear.x,twistMessage->linear.y,twistMessage->linear.z);

    moving_ = (twistMessage->linear.x == 0);
}*/

void SonarToCloud::velCallback(nav_msgs::Odometry msg){

    if (lastTime_ == 0){
        oldPose_.position = msg.pose.pose.position;
        lastTime_ = ros::Time::now().toSec();
    }
    else {
        if ((fabs(msg.pose.pose.position.x - oldPose_.position.x) > 0.0001) || (fabs(msg.pose.pose.position.y - oldPose_.position.y) > 0.0001)){
            time_ = (ros::Time::now().toSec() - lastTime_);
            poseChange_.position.x = (msg.pose.pose.position.x - oldPose_.position.x)/time_;
            poseChange_.position.y = (msg.pose.pose.position.y - oldPose_.position.y)/time_;
            poseChange_.position.z = (msg.pose.pose.position.z - oldPose_.position.z)/time_;
        }
        oldPose_.position = msg.pose.pose.position;

        lastTime_ = ros::Time::now().toSec();
    }

}

bool SonarToCloud::newAngle(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine){
    return (fabs(scan->sensorAngle - oldScanLine->sensorAngle) > 0.001);
}

void SonarToCloud::beamCallback(avora_msgs::SonarScanLineConstPtr scanLine){
    pcl::PointXYZI pclPoint;
    geometry_msgs::PointStamped geometryPoint;
    //NODELET_INFO("BeamCallback");
    if (mode_.find("LASER") != -1){
        //NODELET_INFO("Mode Laser Scan size %d", scanSize_);
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
        geometryPoint.header = scanLine->header;
        geometryPoint.point.x = (maxIndex + 1) * rangeResolution * cos(scanLine->angle);
        geometryPoint.point.y = (maxIndex + 1) * rangeResolution * sin(scanLine->angle);
        geometryPoint.point.z = 0;
        // And then transform them to the robot frame, which means rotating -sensorAngle in the Y axis
        //point.x = point.x * cos(-scanLine->sensorAngle);
        // Since the rotation is aroung Y, the Y coordinate is not modified
        //point.z = point.x * sin(-scanLine->sensorAngle);

        try{
            if (targetFrame_ != scanLine->header.frame_id){
            	listener_.transformPoint(targetFrame_, geometryPoint, geometryPoint);
            	laserCloud_->header.frame_id = geometryPoint.header.frame_id;
            }
            else laserCloud_->header.frame_id = scanLine->header.frame_id;
        }
        catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              laserCloud_->header.frame_id = scanLine->header.frame_id;
        }
        pclPoint.x = geometryPoint.point.x ;
        pclPoint.y = geometryPoint.point.y;
        pclPoint.z = geometryPoint.point.z;
        pclPoint.intensity = scanLine->intensities[maxIndex];

        //We add the point to the cloud that later will be converted to the ROS message
        if (pclPoint.intensity) laserCloud_->push_back(pclPoint);
        laserCloudNBeams_++;
    }
    if (mode_.find("SONAR") != -1){
        //NODELET_INFO("Mode Sonar Scan size %d", scanSize_);
        if ((oldScanLine_ != 0) && (newAngle(scanLine, oldScanLine_)
                || (sonarCloudNBeams_ == scanSize_))) {
            //NODELET_INFO("Scan full with %d beams", scanSize_);

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

        double rangeResolution = (scanLine->range_resolution == 0) ? scanLine->maxrange_meters / scanLine->intensities.size() : scanLine->range_resolution;

        for (int i=0;i<scanLine->intensities.size();i++){
            geometryPoint.header = scanLine->header;
            geometryPoint.point.x = (i + 1) * rangeResolution * cos(scanLine->angle);
            geometryPoint.point.y = (i + 1) * rangeResolution * sin(scanLine->angle);
            geometryPoint.point.z = 0;
            // And then transform them to the robot frame, which means rotating -sensorAngle in the Y axis
            //point.x = point.x * cos(-scanLine->sensorAngle);
            // Since the rotation is aroung Y, the Y coordinate is not modified

            try{
                if (targetFrame_ != scanLine->header.frame_id){
                	listener_.transformPoint(targetFrame_, geometryPoint, geometryPoint);
                	sonarCloud_->header.frame_id = geometryPoint.header.frame_id;
                }
                else sonarCloud_->header.frame_id = scanLine->header.frame_id;
            }
            catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  sonarCloud_->header.frame_id = scanLine->header.frame_id;
            }

            pclPoint.x = geometryPoint.point.x + (poseChange_.position.x * (scanLine->header.stamp.toSec() - oldStamp_.toSec()));
            pclPoint.y = geometryPoint.point.y + (poseChange_.position.y * (scanLine->header.stamp.toSec() - oldStamp_.toSec()));
            pclPoint.z = geometryPoint.point.z + (poseChange_.position.z * (scanLine->header.stamp.toSec() - oldStamp_.toSec()));
            pclPoint.intensity = scanLine->intensities[i];
            //if (moving_) pclPoint.data_c[3] = scanLine->header.stamp.toSec();
            //else pclPoint.data_c[3] = -scanLine->header.stamp.toSec();

            if(pclPoint.z > heightLimit_) continue;

            sonarCloud_->push_back(pclPoint);
            sonarCloudSize_++;
        }
        sonarCloudNBeams_++;
    }
    oldScanLine_ = boost::make_shared<avora_msgs::SonarScanLine>(*scanLine);
    oldStamp_ = scanLine->header.stamp;
}
//Now that we have the sonar cloud in "sonarCloud" we convert it to a ROS message and publish it
void SonarToCloud::publishSonarCloud(){
    //NODELET_INFO("Publish Sonar Cloud");
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(*sonarCloud_,cloudMessage);
    cloudMessage.header.frame_id = sonarCloud_->header.frame_id;
    cloudMessage.header.stamp = ros::Time::now();
    sonarCloudPublisher_.publish(cloudMessage);
}

void SonarToCloud::publishLaserCloud(){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(*laserCloud_,cloudMessage);
    cloudMessage.header.frame_id = laserCloud_->header.frame_id;
    cloudMessage.header.stamp = ros::Time::now();
    laserCloudPublisher_.publish(cloudMessage);
}

PLUGINLIB_DECLARE_CLASS(sonar_processing, SonarToCloud, sonar_processing::SonarToCloud, nodelet::Nodelet)
}
