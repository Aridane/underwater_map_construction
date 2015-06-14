#ifndef ICP_HH
#define ICP_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <avora_msgs/StampedIntensityCloud.h>
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
    double width_;
    int nSamples_;
    double sampleStep_;
    ros::Publisher* debugPublisher_;
public:
	ICP();
    ICP(int maxIterations, double errorThreshold);
    ICP(int maxIterations, double errorThreshold, ros::Publisher* debugPublisher);
	~ICP();

    void setDebugPublisher(ros::Publisher *debugPublisher);
    void setMaxIterations(int maxIterations);
    void setErrorThreshold(double errorThreshold);
    void setWidth(double width);
    void setNSamples(int nSamples);
    void setSampleStep(double sampleStep);
    std::vector<Vector3d> applyTransformation(intensityCloud::Ptr P, std::vector<double> timeStamps, Matrix4f R, std::vector<Vector3d> transforms, Vector3d T);
    std::vector<BlockInfo> closestPoints(intensityCloud::Ptr P, MLSM *X,std::vector<double> timestamps, Vector3d eV);
    intensityCloud::Ptr getTransformation(avora_msgs::StampedIntensityCloudPtr P0, MLSM *X, Vector3d eV, Vector3d eT, Matrix4f eR, Vector3d *Tv, Vector3d *T, Matrix4f *R);
    double registration(intensityCloud::Ptr P, std::vector<BlockInfo>* Y, std::vector<double> timeStamps, std::vector<Vector3d> *Tv, Vector3d *T, Matrix4f *R, Vector3d direction);
};

#endif
