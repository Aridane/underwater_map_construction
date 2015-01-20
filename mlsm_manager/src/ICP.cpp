#include "ICP.h"

ICP::ICP(){

}

ICP::ICP(int maxIterations, double errorThreshold){
    maxIterations_ = maxIterations;
    errorThreshold_ = errorThreshold;
}

ICP::~ICP(){

}

bool ICP::getTransformation(intensityCloud::Ptr P, MLSM *X, Matrix3d eTv, Matrix3d eT, Matrix3d eR, Matrix3d &Tv, Matrix3d &T, Matrix3d &R){
    double error = DBL_MAX;
    std::vector<blockInfo> Y;
    // Initial transform

    while ((iterations < maxIterations_) && (error > errorThreshold_)){
        // Calculate closest points
        Y = closestPoints(P,X);
        // Calculate transformation
        error = registration(P,Y,Tv, T, R);
        // Transform points
        Matrix4f q = 1 * R + T + Tv;
        // Iterate through P applying the correct transformation depending on Tv and the stamp

        // Calculate error
        error = calculateError();
        // Iterate?
        iterations++;
    }
    return error < errorThreshold_;


}
