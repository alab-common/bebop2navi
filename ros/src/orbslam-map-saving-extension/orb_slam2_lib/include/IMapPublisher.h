//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_IMAPDRAWER_H
#define ORB_SLAM2_IMAPDRAWER_H

#include "Map.h"

#include <mutex>

namespace ORB_SLAM2
{

class IMapPublisher
{
public:
    IMapPublisher(Map* map) : mpMap(map) { }

    //void SetCurrentCameraPose(const cv::Mat &Tcw);
    //cv::Mat GetCameraPose();
	virtual void SetCurrentCameraPose(const cv::Mat &Tcw, const double &timestamp);
    std::pair<cv::Mat, double> GetCameraPose();

    Map *GetMap() { return mpMap; }
    bool isCamUpdated();
    void ResetCamFlag();

private:
    cv::Mat mCameraPose;
	double mCameraTimestamp;

    Map* mpMap;
    std::mutex mMutexCamera;
    bool mbCameraUpdated;
    bool mbReuseMap;

};

}


#endif //ORB_SLAM2_IMAPDRAWER_H
