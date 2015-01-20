#ifndef ICP_HH
#define ICP_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include "MLSM.h"

typedef struct blockInfo{
    int i;
    int j;
    Block* blockPtr;
} blockInfo;

class ICP {
private:
	int maxIterations_;
    double errorThreshold_;
public:
	ICP();
    ICP(int maxIterations, double errorThreshold);
	~ICP();

    bool getTransformation(intensityCloud P, MLSM* X,Matrix3d eTv, Matrix3d eT, Matrix3d eR,Matrix3d &Tv ,Matrix3d &T, Matrix3d &R);
};

#endif
