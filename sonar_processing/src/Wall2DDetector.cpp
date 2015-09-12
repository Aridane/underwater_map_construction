#include "Wall2DDetector.h"
#include <pluginlib/class_list_macros.h>

namespace sonar_processing
{

using namespace std;

void polarToCart(cv::Mat polar, cv::Mat cart){
    int x, y;
    double xc, yc;
    //cv::Mat cart(polar.cols*2, polar.cols*2, CV_8UC1);
    for (int i = 0; i < polar.rows; i++) {
        xc = cos(i*2*M_PI/polar.rows);
        yc = sin(i*2*M_PI/polar.rows);
        for (int j = 0; j < polar.cols; j++) {
            x = j*xc + polar.cols;
            y = j*yc + polar.cols;

            cart.at<uint8_t>(x, y) = polar.at<uint8_t>(i, j);
        }
    }
}

void Wall2DDetector::onInit(){
    NODELET_INFO("Initializing line detection nodelet...");
    nh_ = getPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(nh_);
    nh_.param("cloudSubscribeTopic", cloudSubscribeTopic_,
            string("/sonar/scan/cleaned"));
    nh_.param("wallPublishTopic", wallPublishTopic_, string("/Sonar/Walls"));
    nh_.param("wallCoefficientsPublishTopic", wallCoefficientsPublishTopic_, string("/Sonar/Walls/Coeff"));

    nh_.param("mode", mode_, string("RANSAC")); // RANSAC | HOUGH
    nh_.param("nWalls", nWalls_, int(3));
    nh_.param("RANSACDistance", RANSACDistance_, double(0.6));
    nh_.param("RANSACProbability", RANSACProbability_, double(0.8));
    nh_.param("RANSACMaxIterations", RANSACMaxIterations_, int(5000));


    cloudSubscriber_ = nh_.subscribe(cloudSubscribeTopic_.c_str(), 0,
            &Wall2DDetector::cloudCallback, this);
    wallPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
            wallPublishTopic_.c_str(), 1);
    wallCoefficientPublisher_ = nh_.advertise<avora_msgs::Wall>(
                wallCoefficientsPublishTopic_.c_str(), 1);
    debugPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/Debug/Wall", 1);
    debugImagePublisher_ =  it_->advertise("/Sonar/Walls/image/debug", 1);
    markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/sonar/walls/markers", 5);
}

Wall2DDetector::Wall2DDetector()
{


}

Wall2DDetector::~Wall2DDetector() {
}

void Wall2DDetector::cloudCallback(avora_msgs::StampedIntensityCloudConstPtr cloudMessagePtr) {
    NODELET_INFO("Wall 2D Got the cloud");
    intensityCloud cl;
    pcl::fromROSMsg(cloudMessagePtr->cloud,cl);
    intensityCloud::Ptr cloud = boost::make_shared<intensityCloud>(cl);

	if (mode_.find("RANSAC") != -1) {
        int skipped = 0;
		intensityCloud result,line;

        vector<int> inliers;
		Eigen::VectorXf coefficients;
		sensor_msgs::PointCloud2 cloudMsg2;
        for (int i=0;i<nWalls_;i++){
            NODELET_INFO("========== Wall %d ==========",i);

			// created RandomSampleConsensus object and compute the appropriated model
			pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(
                    new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
            ransac.setDistanceThreshold(RANSACDistance_);
            ransac.setMaxIterations(RANSACMaxIterations_);
            ransac.setProbability(RANSACProbability_);
            NODELET_INFO("Compute ransac");

			ransac.computeModel();
            NODELET_INFO("Ransac computed");

            ransac.getInliers(inliers);
            ransac.getModelCoefficients(coefficients);


			// copies all inliers of the model computed to another PointCloud
            pcl::copyPointCloud(*cloud,inliers,line);
            NODELET_INFO("CLoud size before filtering out %d", (int)cloud->size());
            NODELET_INFO("Inliers %d C - I %d", (int)inliers.size(),(int)cloud->size()-inliers.size());
            pcl::ExtractIndices<pcl::PointXYZI> extractor(true);
            extractor.setInputCloud (cloud);
            extractor.setIndices (boost::make_shared<vector<int> >(inliers));
            extractor.setNegative (true);
            extractor.filter(*cloud);
            NODELET_INFO("CLoud size after filtering out %d", (int)cloud->size());

			inliers.clear();
			ros::spinOnce();


            pcl::toROSMsg(*(cloud.get()), cloudMsg2);
            cloudMsg2.header = cloudMessagePtr->header;
			debugPublisher_.publish(cloudMsg2);


            result += line;
			line.clear();

            // First three componentes of coefficients are a point in the line
            // the other three components are the direction vector
            Eigen::Vector3d a,n,p;
            a[0] = coefficients[0];
            a[1] = coefficients[1];
            a[2] = coefficients[2];
            n[0] = coefficients[3];
            n[1] = coefficients[4];
            n[2] = coefficients[5];

            p = a - (a[0]*n[0] + a[1]*n[1])*n;
            double rho = sqrt(p[0]*p[0] + p[1]*p[1]);
            double theta = asin(p[1]/rho);
            NODELET_INFO("Coefficients %f %f %f %f %f %f",coefficients[0],coefficients[1],coefficients[2],coefficients[3],coefficients[4],coefficients[5]);
            NODELET_INFO("\n\trho %f\n\ttheta %f (%f)\n\tPx %f Py %f",rho,theta,theta*180.0/M_PI,p[0],p[1]);
            wallMsg_.orientation = theta;
            wallMsg_.distance = rho;
            wallMsg_.header = cloudMessagePtr->header;
            wallCoefficientPublisher_.publish(wallMsg_);
            visualization_msgs::Marker marker;
            marker.header = cloudMessagePtr->header;
            marker.ns = "walls";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = a[0];
            marker.pose.position.y = a[1];
            marker.pose.position.z = a[2];

            tf::Quaternion q;
            q.setRPY(0,0,theta);
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            marker.scale.x = 0.3;
            marker.scale.y = 3.0;
            marker.scale.z = 0.1;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(10);

            markerPublisher_.publish(marker);
            ros::spinOnce();
		}


		pcl::toROSMsg(result, cloudMsg2);
        cloudMsg2.header = cloudMessagePtr->header;
		wallPublisher_.publish(cloudMsg2);
	}
    else if (mode_.find("HOUGH") != -1) {
        NODELET_INFO("Hough powers activate!");
        cv_bridge::CvImage cvImg;
        if (cloud->isOrganized()) {
            cv::Mat cloudImage = cv::Mat(cloud->height, cloud->width, CV_8UC1);
            if (!cloud->empty()) {
                NODELET_INFO("Cloud to image rows %d cols %d size: %d",(int)cloudImage.rows,(int)cloudImage.cols, (int)cloud->size());
                for (int h=0; h<cloudImage.rows; h++) {
                    for (int w=0; w<cloudImage.cols; w++) {
                            if (isnan(cloud->at(w, h).x) || isnan(cloud->at(w, h).y) || isnan(cloud->at(w, h).z)){
                                cloudImage.at<uint8_t>(h,w) = 0;
                            }
                            else {

                                cloudImage.at<uint8_t>(h,w) = cloud->at(w, h).intensity;
                            }
                    }
                }
                cvImg.encoding = "mono8";
                cvImg.header = cloudMessagePtr->header;
                debugImagePublisher_.publish(cvImg.toImageMsg());
                NODELET_INFO("Polar to cartesian");
                cv::Mat cartesianImage(2*cloudImage.cols,2*cloudImage.cols,CV_8UC1);
                polarToCart(cloudImage, cartesianImage);
                cvImg.image = cartesianImage;
                debugImagePublisher_.publish(cvImg.toImageMsg());
                NODELET_INFO("Detect edges");
                // Edge detection with gradient
                cv::Canny(cartesianImage,cartesianImage,30,90,3);
                // Hough transform. Used because we get the values we wanted
                // for the wall message
                // rho = lines[i][0], theta = lines[i][1];
                std::vector<Vec2f> lines;
                NODELET_INFO("Detect lines");
                HoughLines(cartesianImage, lines, 1, CV_PI/180, 100, 0, 0 );
                for( size_t i = 0; i < lines.size(); i++ )
                {
                  float rho = lines[i][0], theta = lines[i][1];
                  NODELET_INFO("RHO %f THETA %f", rho, theta);
                  double a = cos(theta), b = sin(theta);
                  double x0 = a*rho, y0 = b*rho;
                  double x1, y1, x2, y2;

                  x1 = (x0 + 1000*(-b));
                  y1 = (y0 + 1000*(a));
                  x2 = (x0 - 1000*(-b));
                  y2 = (y0 - 1000*(a));

                  wallMsg_.orientation = theta;
                  wallMsg_.distance = rho;
                  wallMsg_.header = cloudMessagePtr->header;

                  visualization_msgs::Marker marker;
                  marker.header = cloudMessagePtr->header;
                  marker.ns = "walls";
                  marker.id = i;
                  marker.type = visualization_msgs::Marker::CUBE;
                  marker.action = visualization_msgs::Marker::ADD;
                  marker.pose.position.x = (x1+x2)*0.5;
                  marker.pose.position.y = (y1+y2)*0.5;
                  marker.pose.position.z = 5;

                  tf::Quaternion q;
                  q.setRPY(0,0,theta);
                  marker.pose.orientation.x = q.x();
                  marker.pose.orientation.y = q.y();
                  marker.pose.orientation.z = q.z();
                  marker.pose.orientation.w = q.w();

                  marker.scale.x = 0.3;
                  marker.scale.y = 3.0;
                  marker.scale.z = 0.1;
                  marker.color.r = 0.0f;
                  marker.color.g = 1.0f;
                  marker.color.b = 0.0f;
                  marker.color.a = 1.0;
                  marker.lifetime = ros::Duration(10);

                  markerPublisher_.publish(marker);
                  ros::spinOnce();

                }
            }
        }
    }
}

void Wall2DDetector::detectWall(intensityCloud::ConstPtr cloud) {

}

PLUGINLIB_DECLARE_CLASS(sonar_processing, Wall2DDetector, sonar_processing::Wall2DDetector, nodelet::Nodelet)
}

