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

typedef enum  {WALL, FLOOR, BUOY} BlockType;

class Block {
public:
    Block(){nPoints_ = 0; height_ = 0; depth_ = 0;}
    Block (pcl::PointXYZI mean,pcl::PointXYZI variance,double nPoints,double height,double depth, BlockType type){
		mean_ = mean;
        variance_ = variance;
		nPoints_ = nPoints;
		height_ = height;
		depth_ = depth;
        type_ = type;
	}
    ~Block(){}

    BlockType type_;

	pcl::PointXYZI mean_;
    pcl::PointXYZI variance_;
	double nPoints_;
	//Highest Z
	double height_;
	//Lowest Z
	double depth_;

    void setMean(pcl::PointXYZI newMean) {mean_ = newMean;}
    void increaseNPoints(int n) {nPoints_+=n;}

};

typedef std::vector <boost::shared_ptr<Block> > cell;

typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;

typedef boost::multi_array<cell, 2> grid_type;
typedef grid_type::index gridIndex;

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

};
#endif

