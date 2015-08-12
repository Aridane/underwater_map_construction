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
#include <kdtree.h>
//#include <ANN/ANN.h>
#include "QuadGrid.h"

//0.3 M x 100 = 30M
#define DEFAULTSIZEXMETERS 30.0
#define DEFAULTSIZEYMETERS 30.0
using namespace mlsm;
using namespace std;
class MLSM{
private:
	double resolution_;
    int spanX_;
    int spanY_;
	double sizeXMeters_;
	double sizeYMeters_;
    double verticalElasticity_;

	boost::shared_ptr<QuadGrid> grid_;

	cell occupiedBlocks_;

    int markersId;
    struct kdtree* kdtree_;
    //ANNkd_tree* kdTree;
public:
	MLSM();
    MLSM(double resolution, double sizeXMeters, double sizeYMeters);
	~MLSM();
	//Restart with given paraMeters
    void init(double resolution, double sizeXMeters, double sizeYMeters);

    int getSpanX();
    int getSpanY();
    int addPointCloud(intensityCloud::Ptr cloud);
    visualization_msgs::MarkerArray getROSMarkers(string frameId);
    sensor_msgs::PointCloud2 getROSCloud(string frameId);
    double getResolution();
    bool checkCell(int i, int j);
    Block* findSuitableBlock(int i, int j, pcl::PointXYZI point);
    Block* findSuitableBlock(cellPtr cellP, pcl::PointXYZI point);
    Block* findClosestBlock(pcl::PointXYZI point);

};
#endif
