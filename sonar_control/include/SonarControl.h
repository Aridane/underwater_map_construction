#include <ros/ros.h>
#include <ros/console.h>

#include <avora_msgs/SonarScanCloud.h>

#include <string>

using namespace std;

class SonarControl {
private:
	ros::Publisher cloudPublisher_;
	ros::Subscriber cloudSubscriber_;
	string cloudSubscribeTopic_;
	string cloudPublishTopic_;
public:
	SonarControl(ros::NodeHandle* n);
	~SonarControl();

	void cloudCallback(avora_msgs::SonarScanCloudConstPtr);
};
