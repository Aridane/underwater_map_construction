#include "SonarControl.h"

SonarControl::SonarControl(ros::NodeHandle* n){
	ros::NodeHandle nh("~");
	nh.param("cloudSubscribeTopic_", cloudSubscribeTopic_, string("/Sonar/Scan/SonarCloud"));
	nh.param("cloudPublishTopic_", cloudPublishTopic_, string("/SonarControl/Cloud"));


}
int main (int argc, char** argv){

}
