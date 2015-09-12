#include "MLSMCore.h"
using namespace mlsm;
MLSMCore::MLSMCore(ros::NodeHandle* n):
    ICPSolver_(1,0.15)
    {
	ros::NodeHandle nh("~");
    nh.param("cloudSubscribeTopic", cloudSubscribeTopic_, string("/sonar/scan/cleaned"));
    nh.param("orientationSubscribeTopic_", orientationSubscribeTopic_, string("/orientation"));
    nh.param("positionSubscribeTopic", positionSubscribeTopic_, string("/lastPosition"));
    nh.param("markerPublishTopic", markerPublishTopic_, string("/MLSM/Markers"));
    nh.param("velSubscribeTopic", velTopic_, string("/odom"));

    nh.param("resolution", resolution_, double(0.3));
	nh.param("sizeXMeters", sizeXMeters_, double(60));
	nh.param("sizeYMeters", sizeYMeters_, double(60));
    nh.param("fastUpdate", fastUpdate_, bool("false"));
    nh.param("speedError", speedError_, double(0));


    int maxIterations;
    double errorThreshold;
    int nSamples;
    double sampleStep;

    nh.param("maxIterations", maxIterations, int(1000));
    nh.param("errorThreshold", errorThreshold, double(0.3));
    nh.param("nSamples", nSamples, int(5));
    nh.param("sampleStep", sampleStep, double(0.25));

    ROS_DEBUG("Initializing map");
    ROS_INFO("Map params\n\tResolution %f sizeXM %f sizeYM %f\n\t Max Iterationr: %d nSamples: %d",resolution_,sizeXMeters_,sizeYMeters_,maxIterations, nSamples);

    matching_ = false;
    map_.init(resolution_,sizeXMeters_,sizeYMeters_);

	cloudSubscriber_ = n->subscribe(cloudSubscribeTopic_.c_str(), 0, &MLSMCore::cloudCallback, this);
    markerPublisher_ = n->advertise<visualization_msgs::MarkerArray>(markerPublishTopic_.c_str(), 0);
    matchingSubscriber_ = n->subscribe("startMatching", 0, &MLSMCore::matchingCallback, this);
    velSubscriber_ = n->subscribe(velTopic_.c_str(), 1, &MLSMCore::velCallback, this);
    debugPublisher_ = n->advertise<sensor_msgs::PointCloud2>("debug_cloud",0);
    mapPublisher_ = n->advertise<sensor_msgs::PointCloud2>("/MLSM/cloud",0);


    ICPSolver_.setDebugPublisher(&debugPublisher_);
    ICPSolver_.setErrorThreshold(errorThreshold);
    ICPSolver_.setMaxIterations(maxIterations);
    ICPSolver_.setWidth(3);
    ICPSolver_.setNSamples(nSamples);
    ICPSolver_.setSampleStep(sampleStep);

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

    pose_ = msg.pose.pose;
    /*meanVelocity_.angular.x = 0;
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
    velSamples_++;*/

}

int MLSMCore::addPointCloudToMap(avora_msgs::StampedIntensityCloudPtr cloudMsg){
    ROS_DEBUG("Adding point cloud");
    intensityCloud cloud;
    intensityCloud::Ptr cloudPtr;
    intensityCloud::Ptr result;
    pcl::fromROSMsg(cloudMsg->cloud,cloud);

    cloudFrame_ = cloudMsg->header.frame_id;
    Vector3d eV, eT, V, T, direction;
    Eigen::Matrix4f eR = Matrix<float, 4, 4>::Identity(), R = Matrix<float, 4, 4>::Identity();




    eT[0] = 0;
    eT[1] = 0;
    eT[2] = 0;
    eV = V = T = eT;
    //ROS_INFO("Cloud Received Height = %d Width = %d", cloud->height, cloud->width);
    if (matching_){
        tf::Quaternion qt;
        /*tf::quaternionMsgToTF(pose_.orientation,qt);
        tf::Matrix3x3 m(qt);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("R = %f P = %f Y = %f", roll,pitch,yaw);
        direction[0] = cos(yaw)*cos(pitch);
        direction[1] = sin(yaw)*cos(pitch);
        direction[2] = sin(pitch);*/
        direction[0] = pose_.position.x;
        direction[1] = pose_.position.y;
        direction[2] = pose_.position.z;
        direction.normalize();

        ROS_INFO("X = %f Y = %f Z = %f", direction[0],direction[1],direction[2]);
        //ROS_INFO("TimeChange %f",time_);

        //ROS_INFO("Derived change x=%.3f y=%.3f z=%.3f",poseChange_.position.x,poseChange_.position.y,poseChange_.position.z);
        //ROS_INFO("Derived vel x=%.3f y=%.3f z=%.3f",poseChange_.position.x/time_,poseChange_.position.y/time_,poseChange_.position.z/time_);


        eV[0] = direction[0];//(time_ != 0) ? poseChange_.position.x/time_ : time_;
        eV[1] = direction[1];//(time_ != 0) ? poseChange_.position.y/time_ : time_;
        eV[2] = direction[2];//(time_ != 0) ? poseChange_.position.z/time_ : time_;
        //ROS_INFO("No Error vel x=%.3f y=%.3f z=%.3f",eV[0],eV[1],eV[2]);

        //eV[0] = 0;//roundf((eV[0] + eV[0]*speedError_)*1000)*0.001;
        //eV[1] = 0;//roundf((eV[1] + eV[1]*speedError_)*1000)*0.001;
        //eV[2] = 0;//roundf((eV[2] + eV[2]*speedError_)*1000)*0.001;
        //ROS_INFO("Estimated vel x=%.3f y=%.3f z=%.3f",eV[0],eV[1],eV[2]);

        result = ICPSolver_.getTransformation(cloudMsg,&map_,eV, eT, eR, &V, &T, &R);

        /*Matrix3f r3;
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                r3(i,j) = R(i,j);
            }
        }*/

        //Quaternionf eQ(r3);

        //qt.setX(eQ.x());
        //qt.setY(eQ.y());
        //qt.setZ(eQ.z());
        //qt.setW(eQ.w());
        qt.setRPY(0,0,0);
        tf::Vector3 vt2(T[0], T[1], T[2]);

        tf::Transform map_to_odom(qt, vt2);
        ros::Time current_time = ros::Time::now();

        br_.sendTransform(tf::StampedTransform(map_to_odom,
                                                current_time,
                                                "map",
                                                "odom"));

        //R(0,3) = T[0];
        //R(1,3) = T[1];
        //R(2,3) = T[2];
        //pcl::transformPointCloud(cloud,cloud,R);
        //cloudPtr = boost::make_shared<intensityCloud>(cloud);
        //map_.addPointCloud(result);


        meanVelocity_.angular.x = 0;
        meanVelocity_.angular.y = 0;
        meanVelocity_.angular.z = 0;
        meanVelocity_.linear.x = 0;
        meanVelocity_.linear.y = 0;
        meanVelocity_.linear.z = 0;
        velSamples_ = 0;
    }
    else{
        tf::Quaternion qt;
        qt.setRPY(0,0,0);
        tf::Vector3 vt2(0,0,0);

        tf::Transform map_to_odom(qt, vt2);
        ros::Time current_time = ros::Time::now();

        br_.sendTransform(tf::StampedTransform(map_to_odom,
                                                current_time,
                                                "map",
                                                "odom"));


        cloudPtr = boost::make_shared<intensityCloud>(cloud);
        map_.addPointCloud(cloudPtr);
    }
	//TODO Publish map
	//Publish markers provisionally for debugging
	//TODO publish markers on service demand
	//TODO Remove old markers??
    markers = map_.getROSMarkers("map");
    markerPublisher_.publish(markers);
    sensor_msgs::PointCloud2 msgcloud = map_.getROSCloud("map");
    mapPublisher_.publish(msgcloud);
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
    //debugPublisher_.publish(cloudMsg->cloud);
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
