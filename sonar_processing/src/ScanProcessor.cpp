#include "ScanProcessor.h"
using namespace std;

ScanProcessor::ScanProcessor(ros::NodeHandle* n){
	ros::NodeHandle nh("~");
	nh.param("mode", mode_, string("SONAR+CONT")); //LASER, SONAR, LASER+SONAR, +CONT
	nh.param("maxBinValue", maxBinValue_, int(255));
	nh.param("scanSize", scanSize_, int(120));
	nh.param("thresholdType", thresholdMode_, string("OTSU"));
	nh.param("upsampling", upsamplingMode_, string("MLS+VOXEL_GRID_DILATION"));
	nh.param("outlierRemovalMode", outlierRemovalMode_, string("STATISTICAL"));
	nh.param("thresholdValue", thresholdValue_, int(128));
	nh.param("thresholdProportion", thresholdProportion_, double(0.75));
	nh.param("sonarSubscribeTopic", sonarSubscribeTopic_, string("/sonar"));
	nh.param("sonarCloudPublishTopic", sonarCloudPublishTopic_, string("/Sonar/Scan/SonarCloud"));
	nh.param("laserCloudPublishTopic", laserCloudPublishTopic_, string("/Sonar/Scan/laserCloud"));

	beamSubscriber_ = n->subscribe(sonarSubscribeTopic_.c_str(), 0, &ScanProcessor::beamCallback, this);
	if (mode_.find("LASER") != -1) laserInit(n);
	if (mode_.find("SONAR") != -1) sonarInit(n);
}

void ScanProcessor::laserInit(ros::NodeHandle* n){
	laserCloudPublisher_ = n->advertise<avora_msgs::SonarScanCloud>(laserCloudPublishTopic_, 1);
	laserCloud_ = boost::make_shared<intensityCloud>();
	laserCloudMsg_ = boost::make_shared<avora_msgs::SonarScanCloud>();
}

void ScanProcessor::sonarInit(ros::NodeHandle* n){
	sonarCloudPublisher_ = n->advertise<sensor_msgs::PointCloud2>(sonarCloudPublishTopic_.c_str(), 1);
	sonarDebugCloudPublisher_ = n->advertise<sensor_msgs::PointCloud2>("/DebugCloud", 1);

	sonarCloud_ = boost::make_shared<intensityCloud>();
	sonarCloudMsg_ = boost::make_shared<avora_msgs::SonarScanCloud>();
	sonarCloudSize_ = 0;
	sonarCloudNBeams_ = 0;
	sonarCloud_->is_dense = true;

}

ScanProcessor::~ScanProcessor(){
}

bool ScanProcessor::hasChanged(avora_msgs::SonarScanLineConstPtr scan, avora_msgs::SonarScanLineConstPtr oldScanLine){
	//return (scan->sensorAngle != oldScanLine->sensorAngle);
	return false;
}

void ScanProcessor::beamCallback(avora_msgs::SonarScanLineConstPtr scanLine){
	double x,y,z;
	pcl::PointXYZI point;
	int max = 0, maxIndex = 0;

	if (mode_.find("LASER") != -1){
		if (hasChanged(scanLine, oldScanLine_)) {
			processLaserCloud();
			publishLaserCloud();
			laserCloud_.reset();
		}
		for (int i=0;i<scanLine->intensities.size();i++){
			if (scanLine->intensities[i] > max){
				max = scanLine->intensities[i];
				maxIndex = i;
			}
		}
		point.x = maxIndex * scanLine->range_resolution * cos(scanLine->angle);
		point.y = maxIndex * scanLine->range_resolution * sin(scanLine->angle);
		point.z = 0;
		point.intensity = scanLine->intensities[maxIndex];

		laserCloud_->push_back(point);
	}

	if (mode_.find("SONAR") != -1){
		if (((oldScanLine_ != 0) && (hasChanged(scanLine, oldScanLine_))) || (sonarCloudNBeams_ == scanSize_)) {
			processSonarCloud();
			publishSonarCloud();

			if ((mode_.find("CONT") != -1) && !(hasChanged(scanLine, oldScanLine_))){
				sonarCloud_->erase(sonarCloud_->begin(),sonarCloud_->begin()+sonarCloud_->width);
				sonarCloudNBeams_--;
				sonarCloudSize_ -= sonarCloud_->width;

			}
			else {
				sonarCloud_->clear();
				sonarCloudSize_ = 0;
				sonarCloudNBeams_ = 0;
			}
		}
		double RR = scanLine->range_resolution;
		if (RR == 0) RR = 30.0 / 500.0;

		scanAngles.push_back(scanLine->angle);

		for (int i=0;i<scanLine->intensities.size();i++){
			//TODO Put correct values
			point.x = (i+1) * RR;// * cos(scanLine->angle);
			point.y = (i+1) * RR;// * sin(scanLine->angle);
			point.z = 0;
			point.intensity = scanLine->intensities[i];

			sonarCloud_->push_back(point);
			sonarCloudSize_++;
		}
		sonarCloudNBeams_++;
	}

	oldScanLine_ = scanLine;

}

void ScanProcessor::intensityToRGB(intensityCloud::Ptr cloudIn, rgbCloud::Ptr rgbCloudOut) {
	rgbCloudOut = boost::make_shared<rgbCloud>();
	for(cloudIterator_ = cloudIn->begin();cloudIterator_ != cloudIn->end();cloudIterator_++){
		pcl::PointXYZRGB point = pcl::PointXYZRGB(cloudIterator_->intensity, cloudIterator_->intensity, cloudIterator_->intensity);
		point.x = cloudIterator_->x;
		point.y = cloudIterator_->y;
		point.z = cloudIterator_->z;
		rgbCloudOut->push_back(point);
	}
	rgbCloudOut->height = cloudIn->height;
	rgbCloudOut ->width = cloudIn->width;
	rgbCloudOut->resize(rgbCloudOut->height*rgbCloudOut ->width);
}

void ScanProcessor::thresholdCloud(intensityCloud::Ptr cloud){
	double maxFixed = 0.0;
	//Histogram
	std::vector<double> histogram(256,0);
	// Filter for thresholding
	pcl::PassThrough<pcl::PointXYZI> passthroughFilter;


	for (cloudIterator_ = cloud->begin();cloudIterator_ != cloud->end();cloudIterator_++){
		histogram[cloudIterator_->intensity]++;
		if (cloudIterator_->intensity > maxFixed) maxFixed = cloudIterator_->intensity;
	}
	if (thresholdMode_.find("Fixed") != -1){
		// Set input cloud
		passthroughFilter.setInputCloud(cloud);
		// Field we want to filter (intensity)
		passthroughFilter.setFilterFieldName("intensity");
		// Set range of intensity values accepted
		passthroughFilter.setFilterLimits(thresholdValue_, maxBinValue_+1);
		// Keep organised setting removed points to NaN
		passthroughFilter.setKeepOrganized(true);
		// Apply filter
		passthroughFilter.filter(*cloud);
	}
	if (thresholdMode_.find("PROPORTIONAL") != -1){
		// Set input cloud
		passthroughFilter.setInputCloud(cloud);
		// Field we want to filter (intensity)
		passthroughFilter.setFilterFieldName("intensity");
		// Set range of intensity values accepted
		passthroughFilter.setFilterLimits(maxFixed*thresholdProportion_, maxBinValue_+1);
		// Keep organised setting removed points to NaN
		passthroughFilter.setKeepOrganized(true);
		// Apply filter
		passthroughFilter.filter(*cloud);
	}
	if (thresholdMode_.find("OTSU") != -1){
		//OTSU Method
		double sum = 0;
		for (int i=0 ; i<256 ; i++) sum += i * histogram[i];

		double size = cloud->size();
		double sumB = 0;
		double wB = 0;
		double wF = 0;
		double mB;
		double mF;
		double max = 0.0;
		double between = 0.0;
		double threshold1 = 0.0;
		double threshold2 = 0.0;

		for (int i = 0;i<256;i++){
			wB += histogram[i];
			if (wB == 0) continue;
			wF = size - wB;
			if (wF == 0) break;
			sumB += i * histogram[i];
			mB = sumB / wB;
			mF = (sum - sumB) / wF;
			between = wB * wF * (mB - mF) * (mB - mF);
			if (between >= max){
				threshold1 = i;
				if (between > max)
					threshold2 = i;
				max = between;
			}

		}
		double thresholdValueOTSU = (threshold1 + threshold2) / 2.0;
		ROS_INFO("OTSU THRESHOLD IS %f", thresholdValueOTSU);

		// Set input cloud
		passthroughFilter.setInputCloud(cloud);
		// Field we want to filter (intensity)
		passthroughFilter.setFilterFieldName("intensity");
		// Set range of intensity values accepted
		passthroughFilter.setFilterLimits(thresholdValueOTSU, maxBinValue_+1);
		// Keep organised setting removed points to NaN
		passthroughFilter.setKeepOrganized(true);
		// Apply filter
		passthroughFilter.filter(*cloud);

	}



}

void ScanProcessor::removeCloudOutliers(intensityCloud::Ptr cloud){
	ROS_INFO("OUTLIER %s", outlierRemovalMode_.c_str());
	if (outlierRemovalMode_.find("STATISTICAL") != -1){
		ROS_INFO("STATISTICAL");
		// Create the filtering object
		 pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
		 sor.setInputCloud (cloud);
		 // Set the number of nearest neighbors to use for mean distance estimation.
		 sor.setMeanK (50);
		 /* Set the standard deviation multiplier for the distance threshold calculation.
		 	 The distance threshold will be equal to: mean + stddev_mult * stddev.
		 	 Points will be classified as inlier or outlier if their average neighbor
		 	 distance is below or above this threshold respectively.
		 */
		 sor.setStddevMulThresh (0.5);
		 sor.setKeepOrganized(true);
		 sor.filter (*cloud);
	}else {
		ROS_INFO("RADIUS");
		// Create the radius outlier removal filter
		pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier_removal;
		// Set input cloud
		radius_outlier_removal.setInputCloud (cloud);
		// Set radius for neighbor search
		radius_outlier_removal.setRadiusSearch (oldScanLine_->range_resolution);
		// Set threshold for minimum required neighbors neighbors
		radius_outlier_removal.setMinNeighborsInRadius (20);



		radius_outlier_removal.setKeepOrganized(true);
		// Do filtering
		radius_outlier_removal.filter (*cloud);

	}

}


intensityCloud ScanProcessor::upSampleCloud(intensityCloud::Ptr cloud, int newDimX, int newDimY, intensityCloud::Ptr completeCloud){

	intensityCloud result;
	double xFactor = newDimX / cloud->width;
	double yFactor = newDimY / cloud->height;

	int upX00, upY00;
	int upX01, upY01;
	int upX10, upY10;
	int upX11, upY11;
	result.height = newDimY;
	result.width = newDimX;
	result.resize(newDimY * newDimX);
	/*		FX ->
	 * 	CY
	 * 	|
	 * 	v
	 *
	 */
	/***********XY******************XY******************************************
	 * 			00			|		10
	 * ------------------------------------------------------------------------
	 * 			01			|		11
	 *
	 **************************************************************************/

	//Expansion
	for (int i=1;i<cloud->height;i++){
	//For each four points in the original cloud, we fill each of the points within
		for (int j=1;j<cloud->width;j++){
			//Upsampled pixels are between this 4 points
			pcl::PointXYZI p00 = cloud->at(j-1,i-1);
			pcl::PointXYZI p01 = cloud->at(j-1,i);
			pcl::PointXYZI p10 = cloud->at(j,i-1);
			pcl::PointXYZI p11 = cloud->at(j,i);
			upX00 = xFactor * (j - 1);
			upX01 = xFactor * j;
			upX10 = xFactor * (j - 1);
			upX11 = xFactor * j;

			upY00 = yFactor * (i - 1);
			upY01 = yFactor * i;
			upY10 = yFactor * (i - 1);
			upY11 = yFactor * i;

			result.at(upX11, upY11) = p11;

			for (int k=upY00+1;k<upY11;k++){
				for (int l=upX00;l<upX11;l++){

					//result.at(l,k) =
				}
			}

		}
	}


	/*
	if (upsamplingMode_.find("MLS") != -1){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempRGBCloud;
	
		//intensityToRGB(cloud, tempRGBCloud);

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

		mls.setComputeNormals(false);
		mls.setInputCloud(tempRGBCloud);
		mls.setPolynomialFit(false);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(0.1);

		if (upsamplingMode_.find("NONE") != -1)
			mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::NONE);
		else if (upsamplingMode_.find("DISTINCT_CLOUD") != -1)
			mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::DISTINCT_CLOUD);
		else if (upsamplingMode_.find("SAMPLE_LOCAL_PLANE") != -1)
			mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
		else if (upsamplingMode_.find("RANDOM_UNIFORM_DENSITY") != -1)
			mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::RANDOM_UNIFORM_DENSITY);
		else if (upsamplingMode_.find("VOXEL_GRID_DILATION") != -1){
			mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::VOXEL_GRID_DILATION);
			mls.setDilationVoxelSize(0.003);
		}
		//Upsampled cloud into sonarCloud_
		mls.process(*tempRGBCloud);

		pcl::PointCloudXYZRGBtoXYZI(*rgbCloud, *sonarCloud_);


	} else if (upsamplingMode_ == "Poisson"){

	} else if (upsamplingMode_ == "Bilateral"){

	}*/
	return result;
}

intensityCloud ScanProcessor::polar2Cartesian(intensityCloud::Ptr cloudIn){
	intensityCloud cloudOut;
	int i = 0;
	double angle;
	pcl::PointXYZI p;
	for (cloudIterator_ = cloudIn->begin();cloudIterator_ != cloudIn->end();cloudIterator_++){

		angle = scanAngles[i/500];
		p.x = cloudIterator_->x * cos(angle);
		p.y = cloudIterator_->y * sin(angle);
		p.z = cloudIterator_->z;
		p.intensity = cloudIterator_->intensity;
		cloudOut.push_back(p);
		i++;
	}
	scanAngles.clear();
	return cloudOut;
}

// This procedure refines the data from the scan in the following way
// 1 - Threshold values with low intensity -> produces NaN
// 2 - Outlier Removal -> produces NaN
// 3 - Expansion due to sparse data
// 4 - [Optional?] Remove all NaN for smaller cloud
// TODO Check if expansion + outlier or outlier + expansion
void ScanProcessor::processSonarCloud(){
	sonarCloud_->height = sonarCloudNBeams_;
	sonarCloud_ ->width = oldScanLine_->intensities.size();
	sonarCloud_->resize(sonarCloud_->height*sonarCloud_ ->width);
	intensityCloud::Ptr tempSonarCloud = boost::make_shared<intensityCloud>(*sonarCloud_);

	thresholdCloud(tempSonarCloud);
	removeCloudOutliers(tempSonarCloud);
	//sonarCloud_.reset(&upSampleCloud(tempSonarCloud,  ,sonarCloud_));
	pcl::toROSMsg(polar2Cartesian(tempSonarCloud), sonarCloudMsg_->scan);
}

void ScanProcessor::publishSonarCloud(){
	sonarCloudMsg_->scan.header.frame_id = "/map";
	sonarCloudPublisher_.publish(sonarCloudMsg_->scan);
}

void ScanProcessor::processLaserCloud(){
	laserCloudMsg_->gain = oldScanLine_->gain;
	laserCloudMsg_->maxrange_meters = oldScanLine_->maxrange_meters;
	laserCloudMsg_->nBeams = laserCloud_->size();
	laserCloudMsg_->servoAngle = oldScanLine_->angle;

	pcl::toROSMsg(*(laserCloud_.get()), laserCloudMsg_->scan);
}

void ScanProcessor::publishLaserCloud(){
	laserCloudPublisher_.publish(laserCloudMsg_);
}

int main (int argc, char** argv){
	ros::init(argc, argv, "ScanProcessor");
	ROS_INFO("SCAN PROCESSOR STARTS");
	ros::NodeHandle nh;

	ScanProcessor sonar2scan(&nh);

	while (ros::ok()) {
		ros::spinOnce();
	}

}
