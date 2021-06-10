//
// Created by jan on 07.05.19.
//

#ifndef ORB_SLAM2_ISTATESUBSCRIBER_H
#define ORB_SLAM2_ISTATESUBSCRIBER_H

#include "Tracking.h"


namespace ORB_SLAM2
{
    class IStateSubscriber
    {
    public:
        virtual void SetCurrentState(const ORB_SLAM2::Tracking::eTrackingState &state);
    };
}

#endif //ORB_SLAM2_ISTATESUBSCRIBER_H

