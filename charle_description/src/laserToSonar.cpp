#include "laserToSonar.h"

LaserToSonar::LaserToSonar(){
    ROS_INFO("Initializing LaserToSonar node...");
    nh_ = getPrivateNodeHandle();

    nh_.param("laserTopic", laserTopic_, string("/charle/laser/scan"));
    nh_.param("sonarTopic",sonarTopic_, string("/sonar/line"));
    nh_.param("intensity_decay", intensity_decay_, int(32));
    nh_.param("step_", step_, int(3));
    nh_.param("range", range_, int(10));

}

LaserToSonar::~LaserToSonar(){

}

void LaserToSonar::laserCallback(sensor_msgs::LaserScanPtr laserMessage){

}
