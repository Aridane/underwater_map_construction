#ifndef ICP_HH
#define ICP_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include "MLSM.h"
// For limit
#include <cfloat>
#include "TypeDefinitions.h"

typedef struct BlockInfo{
    int i;
    int j;
    mlsm::Block* blockPtr;
} BlockInfo;

using namespace Eigen;

class ICP {
private:
	int maxIterations_;
    double errorThreshold_;
    ros::Publisher* debugPublisher_;
public:
	ICP();
    ICP(int maxIterations, double errorThreshold);
    ICP(int maxIterations, double errorThreshold, ros::Publisher* debugPublisher);
	~ICP();

    void setDebugPublisher(ros::Publisher *debugPublisher);
    void applyTransformation(intensityCloud::Ptr P, Matrix4f R, Vector3d T, Vector3d Tv);
    bool getTransformation(intensityCloud::Ptr P, MLSM *X, Vector3d eV, Vector3d eT, Matrix4f eR, Vector3d *Tv, Vector3d *T, Matrix4f *R);
    double registration(intensityCloud::Ptr P, std::vector<BlockInfo>* Y, Vector3d *Tv, Vector3d *T, Matrix4f *R);
};

#endif
