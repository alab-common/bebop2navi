//
// Created by sebastiano on 8/18/16.
//

#include "IMapPublisher.h"

using namespace ORB_SLAM2;
using namespace std;

//void IMapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
void IMapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw, const double &timestamp)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mCameraTimestamp = timestamp;

    mbCameraUpdated = true;
}

void IMapPublisher::SetCurrentCameraPoseHessian(const std::vector<std::vector<double>> hessian)
{
    unique_lock<mutex> lock(mMutexCamera);
    mPoseHessian = hessian;
}

//cv::Mat IMapPublisher::GetCameraPose()
std::pair<cv::Mat, double> IMapPublisher::GetCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    //return mCameraPose;
    return std::make_pair(mCameraPose, mCameraTimestamp);
}

std::vector<std::vector<double>> IMapPublisher::GetPoseHessian()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mPoseHessian;
}

bool IMapPublisher::isCamUpdated()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mbCameraUpdated;
}

void IMapPublisher::ResetCamFlag()
{
    unique_lock<mutex> lock(mMutexCamera);
    mbCameraUpdated = false;
}
