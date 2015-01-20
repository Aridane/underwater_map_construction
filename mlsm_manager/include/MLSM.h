#ifndef _MLSM_H
#define _MLSM_H


#include <ros/ros.h>
#include <ros/console.h>

#include <limits>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/multi_array.hpp>

#include "QuadGrid.h"

//0.3 M x 100 = 30M
#define DEFAULTSIZEXMETERS 30.0
#define DEFAULTSIZEYMETERS 30.0

class MLSM{
private:
	double resolution_;
    int spanX_;
    int spanY_;
	double sizeXMeters_;
	double sizeYMeters_;

	boost::shared_ptr<QuadGrid> grid_;

	cell occupiedBlocks_;

    int markersId;
public:
	MLSM();
	MLSM(double resolution, double sizeXMeters, double sizeYMeters);
	~MLSM();
	//Restart with given paraMeters
	void init(double resolution, double sizeXMeters, double sizeYMeters);

	int addPointCloud(intensityCloud cloud);
	visualization_msgs::MarkerArray getROSMarkers();
};
#endif
