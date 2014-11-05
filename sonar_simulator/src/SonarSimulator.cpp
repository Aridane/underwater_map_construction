#include "SonarSimulator.h"
using namespace std;
using namespace cv;

SonarSimulator::SonarSimulator(ros::NodeHandle* n){
	ros::NodeHandle nh("~");
	nh.param("rate", rate_, double(1));
	nh.param("step", step_, double(3));

	sonarLinePublisher_ = n->advertise<avora_msgs::SonarScanLine>("/Sonar/raw",0,this);

	scanLine_.angle = 0;
	scanLine_.gain = 20;
	scanLine_.intensities.clear();
	for(int i=0;i<500;i++) scanLine_.intensities.push_back(128);
	scanLine_.maxrange_meters = 20;
	scanLine_.range_resolution = 20.00/500.00;
	scanLine_.sensorAngle = 0;

	square[0] = Point2f(10,10);
	square[1] = Point2f(11,13);
	square[2] = Point2f(12,10);
	square[3] = Point2f(11,8);
}

SonarSimulator::~SonarSimulator(){
}

bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                      Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < 1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

void SonarSimulator::publishNextLine(){
	// FILL ScanLine
	Point2f r, p1;
    //ROS_INFO("ANGLE %f", scanLine_.angle);
	p1.x = cos(scanLine_.angle*M_PI/180.0) * 20.0;
	p1.y = sin(scanLine_.angle*M_PI/180.0) * 20.0;

	double index;

	for (int i=0;i<4;i++){
		if (intersection(Point2f(0,0),p1,square[i],square[(i+1)%4],r)){
			float flagX = (r.x - square[i].x) * (r.x - square[(i+1)%4].x);
			float flagY = (r.y - square[i].y) * (r.y - square[(i+1)%4].y);
			if ((flagX < 0) && (flagY < 0)){
				index = sqrt(r.x * r.x + r.y * r.y);
				index = index / scanLine_.range_resolution;
				scanLine_.intensities[(int)index] = 255;
				scanLine_.intensities[((int)index + 1)%500] = 255;
				scanLine_.intensities[((int)index -1)%500] = 255;
			}

		}
	}


	//Publish ScanLine
	double oldDegrees = scanLine_.angle;
	scanLine_.angle = scanLine_.angle * M_PI /180.0;
	sonarLinePublisher_.publish(scanLine_);
	scanLine_.angle = oldDegrees;
	//Update Next Parameters
	scanLine_.intensities.clear();
	scanLine_.angle = scanLine_.angle + step_;
	if (scanLine_.angle >= 360){
		scanLine_.angle = 0;
		for(int i=0;i<500;i++) scanLine_.intensities.push_back(0);
	}
	else if (scanLine_.angle == 90) for(int i=0;i<500;i++) scanLine_.intensities.push_back(0);
	else if (scanLine_.angle == 180) for(int i=0;i<500;i++) scanLine_.intensities.push_back(0);
	else for(int i=0;i<500;i++) scanLine_.intensities.push_back(0);

	scanLine_.gain = 20;


	scanLine_.sensorAngle = 0;
}

double SonarSimulator::getRate(){
	return rate_;
}

int main (int argc, char** argv){
	ros::init(argc, argv, "SonarSimulator");
	ROS_INFO("Simulator started");
	ros::NodeHandle nh;

	SonarSimulator sonarsim(&nh);

	ros::Rate rate(sonarsim.getRate());

	while (ros::ok()) {
		rate.sleep();
		sonarsim.publishNextLine();
		ros::spinOnce();
	}

}
