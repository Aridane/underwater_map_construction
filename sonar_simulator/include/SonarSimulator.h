#include <ros/ros.h>
#include <ros/console.h>

#include <avora_msgs/SonarScanLine.h>
#include <string>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

class SonarSimulator {
private:
	
	double rate_;
	double step_;
	avora_msgs::SonarScanLine scanLine_;
	ros::Publisher sonarLinePublisher_;
	Point2f square[4];

public:
	SonarSimulator(ros::NodeHandle* n);
	~SonarSimulator();
	void publishNextLine();
	double getRate();
};
