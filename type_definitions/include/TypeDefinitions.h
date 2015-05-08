#ifndef TYPE_DEFINITIONS_H
#define TYPE_DEFINITIONS_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/multi_array.hpp>

namespace mlsm {


typedef enum {WALL, FLOOR, BUOY} BlockType;

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
}

typedef std::vector <boost::shared_ptr<mlsm::Block> > cell;

typedef boost::shared_ptr<cell> cellPtr;

typedef pcl::PointCloud<pcl::PointXYZI> intensityCloud;

typedef boost::multi_array<cell, 2> grid_type;
typedef grid_type::index gridIndex;

#endif
