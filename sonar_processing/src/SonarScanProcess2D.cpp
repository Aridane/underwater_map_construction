#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <avora_msgs/SonarScanLine.h>
#include <avora_msgs/SonarScan.h>
#include <avora_msgs/ProcessedScan.h>
#include <avora_msgs/Row.h>
#include <avora_msgs/SonarCartesianScan.h>

#include <cmath>
#include <string>
#include <deque>
using namespace cv;
using namespace std;

template< typename T>
T adjust_value( T value, T in_min, T in_max, T out_min, T out_max )
{
    const double in_range  =  in_max -  in_min;
    const double out_range = out_max - out_min;

    const double value_scaled = ( value - in_min ) / in_range;
    const double value_adjusted = ( value_scaled + out_min ) * out_range;

    return value_adjusted;
}

class SonarScanProcess2D {
private:
	// Node, image transport and publisher
	ros::NodeHandle nh;
	ros::Subscriber scanSubscriber_;
	ros::Publisher processedScanPublisher_;
	ros::Publisher thresholdedScanPublisher_;
	ros::Publisher filteredScanPublisher_;
	ros::Publisher interpolatedScanPublisher_;

	
	ros::Publisher cartesianPublisher_;	

public:
	SonarScanProcess2D()
	{
		ros::NodeHandle nh("~");
		nh.param("scan_topic_name", scanTopicName_, "/sonar/scan");
		nh.param("thresholding", thresholding_, "/sonar/scan");
		
		scanSubscriber = nh.subscribe( scanTopicName_, 1, &SonarScanProcess2D::applyFilters,this);
		
		processedScanPublisher_ = nh.advertise<avora_msgs::SonarScan>("/sonar/scan/processed", 1);
		thresholdedScanPublisher_ = nh.advertise<avora_msgs::SonarScan>("/sonar/scan/thresholded", 1);
		filteredScanPublisher_ = nh.advertise<avora_msgs::SonarScan>("/sonar/scan/filtered", 1);
		interpolatedScanPublisher_ = nh.advertise<avora_msgs::SonarScan>("/sonar/scan/interpolated", 1);

	}

	~SonarScanProcess2D(){}
	
	void applyFilters(avora_msgs::SonarScan scan )
	{
		uint8_t bin_min = 255, bin_max = 0;
		double threshold = 0;		
		uint8_t bin;
		const uint16_t nBeams = scan.nBeams;	
		avora_msgs::ProcessedScan result;
		avora_msgs::SonarCartesianScan CartesianScan;		
		
		
		CartesianScan.row = vector <avora_msgs::Row>(1000);
		for (int i=0;i<1000;i++){
			CartesianScan.row[i].column = vector <uint8_t>(1000);
			for (int j=0;j<1000;j++) CartesianScan.row[i].column[j] = 0;
		}
		
		CartesianScan.maxrange_meters = scan.maxrange_meters;
		CartesianScan.nBeams = 720;
		CartesianScan.nBins = 500;
		CartesianScan.dimX = 1000;
		CartesianScan.dimY = 1000;
		CartesianScan.servoAngle = scan.servoAngle;
		
		
		result.gain = scan.beam[0].gain;
		result.maxrange_meters = scan.beam[0].maxrange_meters;
		result.nBeams = 720;
		result.nBins = 500;
		result.servoAngle = scan.servoAngle;
		
		uint8_t beams[nBeams][500];
		result.beam = vector <avora_msgs::SonarScanLine>(720);

		int r, g, b, x, y;
		double xc, yc;
		
		//THRESHOLDING
		
		cv::Mat cvReadings = cv::Mat::zeros(nBeams, 500, CV_8UC1);	
	    for (int i=0;i<nBeams;i++){
	    	for (int j=0;j<500;j++){
       			cvReadings.at<uint8_t>(i, j) = (uint8_t)scan.beam[i].intensities[j];
       			bin = scan.beam[i].intensities[j];
               	bin_min = std::min(bin_min,bin);
				bin_max = std::max(bin_max,bin);
			}
		}
			threshold =  cv::threshold(cvReadings,cvReadings,0,255, CV_THRESH_BINARY | CV_THRESH_OTSU) + 0.1*bin_max;
		for (int i=0;i<nBeams;i++){
//((float)(0*bin_min+7*bin_max)*0.1);
			for (int j=0;j<500;j++){
				bin = scan.beam[i].intensities[j];
				if (bin<threshold) bin = 0;
				beams[i][j] = bin;
			}
		}

		//INTERPOLATION

		
		for (int i=0;i<720;i++){
			double angle = ((float)i/2.0)*M_PI/180.0;
			int length = nBeams;
			double mindis, dis;
			int j1, j2;
			mindis = 2*M_PI;
			result.beam[i].intensities = vector<uint8_t>(500);
			result.beam[i].angle = angle;
			
			for (int j = 0; j < nBeams; j++) {
				dis = abs(angle - scan.beam[j].angle);
			if (dis < mindis) {
					if (angle >= scan.beam[j].angle) {
						j1 = j;
						j2 = (j + 1 + length) % length;
					} else {
						j1 = (j - 1 + length) % length;
						j2 = j;
					}		
					mindis = dis;
				}
			}
			
			double dt = (scan.beam[j2].angle - scan.beam[j1].angle);
			double time_dif = (angle - scan.beam[j1].angle);
			
			double d1, d2, data;
			
			xc = cos(angle);
			yc = sin(angle);
			result.beam[i].angle = angle;
			
			//printf("Ángle: %f", angle);
			
			for (int j = 0; j < 500; j++) {
				d1 = (double)beams[j1][j];
				d2 = (double)beams[j2][j];
				data = ((time_dif * (d2 - d1)) / dt) + d1;
				//for (int k=0;k<500;k++) printf("_%d_", beams[0][k]);				
				//hsv_to_rgb(&r, &g, &b, bin, 255, 255);
				
				result.beam[i].intensities[j] = data;
				
				x = (j)*xc + 500.0;
				y = (j)*yc + 500.0;
				
				data = adjust_value( (uint8_t)data, bin_min, bin_max, std::numeric_limits<uint8_t>::min(), std::numeric_limits<uint8_t>::max() );
				
				CartesianScan.row[x].column[y] = data;	
			}	
		}
		
		

		cv::Mat cvMatrix = cv::Mat::zeros(1000, 1000, CV_8UC1);
  			
  		for (int i=0;i<1000;i++){
  			for (int j=0;j<1000;j++){
  				cvMatrix.at<uint8_t>(i,j) = CartesianScan.row[i].column[j];
  			}
  		}	  		
  		
		cout << "ELEMENT" << endl;
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15), cv::Point(5, 5));
		cout << "Erode" << endl;
		
				//SMOOTHING
  		cv::GaussianBlur(cvMatrix,cvMatrix,Size(7,7),0,0);
		cv::erode(cvMatrix,cvMatrix, element);
		cv::dilate(cvMatrix,cvMatrix, element);
		
		
		vector<Vec3f> circles;

  		/// Apply the Hough Transform to find the circles
  		HoughCircles( cvMatrix, circles, CV_HOUGH_GRADIENT, 1, cvMatrix.rows/8, 100, 100, 0, 0 );
		
		for( size_t i = 0; i < circles.size(); i++ )
  		{
      		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      		int radius = cvRound(circles[i][2]);
      		// circle center
      		circle( cvMatrix, center, 3, Scalar(0,0,255), -1, 8, 0 );
      		// circle outline
      		circle( cvMatrix, center, radius, Scalar(0,0,255), 3, 8, 0 );
   		}
		
		for (int i=0;i<1000;i++){
			for (int j=0;j<1000;j++){
				CartesianScan.row[i].column[j] = cvMatrix.at<uint8_t>(i,j);
  			}
  		}
		
		
		cout << "END" << endl;
		scan_pub.publish(result);
		cartesian_pub.publish(CartesianScan);
	}
};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "process_sonar_data");
	SonarScanProcess2D sp;
	while (ros::ok()) {
		ros::spin();
	}

	return EXIT_SUCCESS;
}


