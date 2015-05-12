#include "MLSMCore.h"
using namespace mlsm;
MLSMCore::MLSMCore(ros::NodeHandle* n):
    ICPSolver_(1000,0.15)
    {
	ros::NodeHandle nh("~");
    nh.param("cloudSubscribeTopic", cloudSubscribeTopic_, string("/sonar/scan/thresholded"));
    nh.param("orientationSubscribeTopic_", orientationSubscribeTopic_, string("/orientation"));
    nh.param("positionSubscribeTopic", positionSubscribeTopic_, string("/lastPosition"));
    nh.param("markerPublishTopic", markerPublishTopic_, string("/MLSM/Markers"));
    nh.param("velSubscribeTopic", velTopic_, string("/odom"));

    nh.param("resolution", resolution_, double(0.3));
	nh.param("sizeXMeters", sizeXMeters_, double(60));
	nh.param("sizeYMeters", sizeYMeters_, double(60));
    nh.param("fastUpdate", fastUpdate_, bool("false"));

    nh.param("verticalElasticity", verticalElasticity_, double(0));
    ROS_DEBUG("Initializing map");

    matching_ = false;
    map_.init(resolution_,sizeXMeters_,sizeYMeters_, verticalElasticity_);

	cloudSubscriber_ = n->subscribe(cloudSubscribeTopic_.c_str(), 0, &MLSMCore::cloudCallback, this);
    markerPublisher_ = n->advertise<visualization_msgs::MarkerArray>(markerPublishTopic_.c_str(), 0);
    matchingSubscriber_ = n->subscribe("startMatching", 0, &MLSMCore::matchingCallback, this);
    velSubscriber_ = n->subscribe(velTopic_.c_str(), 1, &MLSMCore::velCallback, this);
    debugPublisher_ = n->advertise<sensor_msgs::PointCloud2>("debug_cloud",0);
    ICPSolver_.setDebugPublisher(&debugPublisher_);
    lastPosition_[0] = 0;
    lastPosition_[1] = 0;
    lastPosition_[2] = 0;

    meanVelocity_.angular.x = 0;
    meanVelocity_.angular.y = 0;
    meanVelocity_.angular.z = 0;
    meanVelocity_.linear.x = 0;
    meanVelocity_.linear.y = 0;
    meanVelocity_.linear.z = 0;
    poseChange_.position.x = 0;
    poseChange_.position.y = 0;
    poseChange_.position.z = 0;
    oldPose_ = poseChange_;
    lastTime_ = 0;
    time_ = 0;
    velSamples_ = 0;
    ROS_DEBUG("Initialized");
}

MLSMCore::~MLSMCore(){

}

void MLSMCore::matchingCallback(const std_msgs::Bool msg){
    matching_ = msg.data;
}

void MLSMCore::velCallback(nav_msgs::Odometry msg){

    meanVelocity_.angular.x = 0;
    meanVelocity_.angular.y = 0;
    meanVelocity_.angular.z = 0;
    meanVelocity_.linear.x += (fabs(msg.twist.twist.linear.x) > 0.001) ? msg.twist.twist.linear.x : 0.0;
    meanVelocity_.linear.y += (fabs(msg.twist.twist.linear.y) > 0.001) ? msg.twist.twist.linear.y : 0.0;
    meanVelocity_.linear.z += (fabs(msg.twist.twist.linear.z) > 0.001) ? msg.twist.twist.linear.z : 0.0;
    if (lastTime_ == 0){
        oldPose_.position = msg.pose.pose.position;
        lastTime_ = ros::Time::now().toSec();
    }
    else {
        poseChange_.position.x += (msg.pose.pose.position.x - oldPose_.position.x);
        poseChange_.position.y += (msg.pose.pose.position.y - oldPose_.position.y);
        poseChange_.position.z += (msg.pose.pose.position.z - oldPose_.position.z);
        if ((fabs(msg.pose.pose.position.x - oldPose_.position.x) > 0.0001) || (fabs(msg.pose.pose.position.y - oldPose_.position.y) > 0.0001)){
            time_ = time_ + (ros::Time::now().toSec() - lastTime_);
        }
        oldPose_.position = msg.pose.pose.position;

        lastTime_ = ros::Time::now().toSec();
    }
    velSamples_++;

}

int MLSMCore::addPointCloudToMap(avora_msgs::StampedIntensityCloudPtr cloudMsg){
    ROS_DEBUG("Adding point cloud");
    intensityCloud cloud;
    intensityCloud::Ptr cloudPtr;
    pcl::fromROSMsg(cloudMsg->cloud,cloud);

    cloudFrame_ = cloudMsg->header.frame_id;
    Vector3d eV, eT, V, T;
    Matrix4f eR = Matrix<float, 4, 4>::Identity(), R = Matrix<float, 4, 4>::Identity();
    eT[0] = 0;
    eT[1] = 0;
    eT[2] = 0;
    eV[0] = 0;
    eV[1] = 0;
    eV[2] = 0;
    //ROS_INFO("Cloud Received Height = %d Width = %d", cloud->height, cloud->width);
    if (matching_){
        ROS_INFO("TimeChange %f",time_);

        ROS_INFO("Derived change x=%.3f y=%.3f z=%.3f",poseChange_.position.x,poseChange_.position.y,poseChange_.position.z);
        ROS_INFO("Derived vel x=%.3f y=%.3f z=%.3f",poseChange_.position.x/time_,poseChange_.position.y/time_,poseChange_.position.z/time_);


        eV[0] = 0;//(time_ != 0) ? poseChange_.position.x/time_ : time_;
        eV[1] = 0;//(time_ != 0) ? poseChange_.position.y/time_ : time_;
        eV[2] = 0;//(time_ != 0) ? poseChange_.position.z/time_ : time_;
        ROS_INFO("Estimated vel x=%.3f y=%.3f z=%.3f",eV[0],eV[1],eV[2]);

        ICPSolver_.getTransformation(cloudMsg,&map_,eV, eT, eR, &V, &T, &R);

        R(0,3) = T[0];
        R(1,3) = T[1];
        R(2,3) = T[2];
        pcl::transformPointCloud(cloud,cloud,R);
        cloudPtr = boost::make_shared<intensityCloud>(cloud);
        map_.addPointCloud(cloudPtr);


        meanVelocity_.angular.x = 0;
        meanVelocity_.angular.y = 0;
        meanVelocity_.angular.z = 0;
        meanVelocity_.linear.x = 0;
        meanVelocity_.linear.y = 0;
        meanVelocity_.linear.z = 0;
        velSamples_ = 0;
    }
    else{
        cloudPtr = boost::make_shared<intensityCloud>(cloud);
        map_.addPointCloud(cloudPtr);
    }
	//TODO Publish map
	//Publish markers provisionally for debugging
	//TODO publish markers on service demand
	//TODO Remove old markers??
    markers = map_.getROSMarkers("odom");
    markerPublisher_.publish(markers);
	return 0;
}
void MLSMCore::lastKnownPoseCallback(const geometry_msgs::PointPtr pointMsg){
    tf::pointMsgToTF(*pointMsg,lastPosition_);
}

void MLSMCore::orientationCallback(const geometry_msgs::QuaternionPtr orientationMsg){
    tf::quaternionMsgToTF(*orientationMsg,lastOrientation_);
}

void MLSMCore::cloudCallback(avora_msgs::StampedIntensityCloudPtr cloudMsg){
    //intensityCloud::Ptr cloud = boost::shared_ptr<intensityCloud>(new intensityCloud);
    //pcl::fromROSMsg(cloudMsg,*cloud);
    addPointCloudToMap(cloudMsg);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "MLSMManager");
	ROS_INFO("MLSM MANAGER STARTS");
	ros::NodeHandle nh;
	MLSMCore mlsmCore(&nh);
    ROS_INFO("MLSM_core Spinning");
	while(ros::ok()){
		ros::spinOnce();

	}
}
