//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_IMAPDRAWER_H
#define ORB_SLAM2_IMAPDRAWER_H

#include "Map.h"

#include <mutex>
#include <vector>

namespace ORB_SLAM2
{

class IMapPublisher
{
public:
    IMapPublisher(Map* map) : mpMap(map) { }

    //void SetCurrentCameraPose(const cv::Mat &Tcw);
    //cv::Mat GetCameraPose();
    virtual void SetCurrentCameraPose(const cv::Mat &Tcw, const double &timestamp);
    virtual void SetCurrentCameraPoseHessian(const std::vector<std::vector<double>> hessian);
    std::pair<cv::Mat, double> GetCameraPose();
    std::vector<std::vector<double>> GetPoseHessian();

    Map *GetMap() { return mpMap; }
    bool isCamUpdated();
    void ResetCamFlag();

private:
    cv::Mat mCameraPose;
    double mCameraTimestamp;
    std::vector<std::vector<double>> mPoseHessian;
    // Pose Hessian is calculated using Hpp or Hpp - Hpl Hll^{-1} Hpl^{T}.
    // see the g2o paper to know the details of Hpp, Hll, and Hpl.

    Map* mpMap;
    std::mutex mMutexCamera;
    bool mbCameraUpdated;
    bool mbReuseMap;

};

}


#endif //ORB_SLAM2_IMAPDRAWER_H
