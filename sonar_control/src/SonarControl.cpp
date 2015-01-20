#include "SonarControl.h"

SonarControl::SonarControl(ros::NodeHandle* n){
	ros::NodeHandle nh("~");

    nh.param("mode", mode_, string("STATIC_MAPPING"));
    nh.param("stepSize", stepSize_, int(10));
    nh.param("scanSize", scanSize_, int(120));
    nh.param("servoStateTopic", servoStateTopic_, string("/charle/sonar_to_base_link_position_controller/state"));
    nh.param("servoCommandTopic", servoCommandTopic_, string("/charle/sonar_to_base_link_position_controller/command"));
    nh.param("sonarSubscribeTopic", sonarSubscribeTopic_, string("/charle/sonar"));


    ros::ServiceServer service = nh.advertiseService("sonar_control", &SonarControl::handleRequest, this);

    beamSubscriber_ = nh.subscribe(sonarSubscribeTopic_.c_str(), 2, &SonarControl::beamCallback, this);
    servoStateSubscriber_ = nh.subscribe(servoStateTopic_.c_str(), 2,&SonarControl::jointStateCallback, this);
    servoCommandPublisher_ = nh.advertise<std_msgs::Float64>(servoCommandTopic_,1);
    std_msgs::Float64 commandMessage;
    commandMessage.data = 0;
    servoCommandPublisher_.publish(commandMessage);
    currentCommand_ = 0;
    currentCommand_ = 0;
    beamCount_ = 0;
    direction_ = 1;
}

SonarControl::~SonarControl(){

}

bool SonarControl::handleRequest(avora_msgs::sonar_controlRequest &req, avora_msgs::sonar_controlResponse &res){
    if (req.mode == "STATIC_MAPPING"){
        mode_ = req.mode;
        res.completed = true;
    }
    return true;

}

void SonarControl::jointStateCallback(control_msgs::JointControllerStateConstPtr servoStateMessage){
    currentTarget_ = servoStateMessage->set_point;
    currentPosition_ = servoStateMessage->process_value;

}

void SonarControl::loopOnce(){
    std_msgs::Float64 commandMessage;
    if (mode_ == "STATIC_MAPPING"){
        if (currentTarget_ != currentCommand_){
            commandMessage.data = currentCommand_;
            servoCommandPublisher_.publish(commandMessage);
        }
        else if (beamCount_ >= scanSize_){
            double nextCommand = currentCommand_ + direction_ * (M_PI/180.0) * stepSize_;
            if (fabs(nextCommand) >= 1.57)
            {
                direction_ *= -1;
                nextCommand = currentCommand_ + direction_ * (M_PI/180.0) * stepSize_;
            }
            currentCommand_ = nextCommand;
        }
    }
}

void SonarControl::beamCallback(avora_msgs::SonarScanLineConstPtr scanLine){
    if((currentCommand_ == currentTarget_)&&(fabs(currentCommand_ - currentPosition_)<0.001)) beamCount_++;
    else beamCount_ = 0;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "SonarControl");
    ROS_INFO("SonarControl node starts...");
    ros::NodeHandle nh;

    SonarControl sonarController(&nh);

    while (ros::ok()) {
        ros::spinOnce();
        sonarController.loopOnce();
    }
}
