#include <ros/ros.h>
#include <ros/console.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <avora_msgs/sonar_control.h>
#include <avora_msgs/SonarScanLine.h>
#include <avora_msgs/StampedIntensityCloud.h>
#include <string>

using namespace std;

typedef enum {STATIC_MAPPING, HORIZONTAL} ControlMode;

class SonarControl {
private:
    string mode_;

    double currentCommand_;
    double currentTarget_;
    double currentPosition_;

    ros::Publisher servoCommandPublisher_;
    ros::Subscriber servoStateSubscriber_;
    ros::Subscriber beamSubscriber_;

    string servoStateTopic_;
    string servoCommandTopic_;
    string sonarSubscribeTopic_;

    int beamCount_;
    int scanSize_;
    int direction_;
    int stepSize_;


public:
	SonarControl(ros::NodeHandle* n);
	~SonarControl();

    bool handleRequest(avora_msgs::sonar_controlRequest &req, avora_msgs::sonar_controlResponse &res);
    void jointStateCallback(control_msgs::JointControllerStateConstPtr);
    void beamCallback(avora_msgs::SonarScanLineConstPtr scanLine);

    void loopOnce();
};
