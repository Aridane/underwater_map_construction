#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <avora_msgs/SonarScanLine.h>

using namespace std;

class LaserToSonar{
private:
    string laserTopic_;
    string sonarTopic_;

    int intensity_decay_;
    int step_;
    int range_;

    ros::Publisher sonarPublisher_;
    ros::Subscriber laserSubscriber_;

    ros::NodeHandle nh_;

public:
    LaserToSonar();
    ~LaserToSonar();

    void laserCallback(sensor_msgs::LaserScanPtr laserMessage);
};
