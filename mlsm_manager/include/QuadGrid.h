#ifndef _QUAD_GRID_H
#define _QUAD_GRID_H


#include <ros/ros.h>
#include <ros/console.h>

#include <limits>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/multi_array.hpp>
#include <TypeDefinitions.h>
using namespace mlsm;
class QuadGrid{
private:
	grid_type gridPP_;
	grid_type gridNN_;
	grid_type gridPN_;
	grid_type gridNP_;

	gridIndex gridIndex_;
    int pxSize_;
    int pySize_;
    int nxSize_;
    int nySize_;

	

	//TODO List of occupied cells??

public:
	//TODO Add option for only checking the maps that are being used
	QuadGrid();
	QuadGrid(int pxSize, int pySize, int nxSize, int nySize);
	~QuadGrid();
	//Restart with given parameters
	//void init();
	cell* operator() (int x, int y);
	cell* insert(pcl::PointXYZI point);
    int resize(int newXSize, int newYSize);

    int getXSize();
    int getYSize();

};
#endif

