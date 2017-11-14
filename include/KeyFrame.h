//
// Created by rain on 17-9-19.
//

#ifndef RAIN_VIO_KEYFRAME_H
#define RAIN_VIO_KEYFRAME_H


#include "Frame.h"
#include "Map.h"

namespace RAIN_VIO
{

class Frame;
class Map;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap);

    void SetPose(Eigen::Matrix<double, 3, 4> Tcw);

};

}



#endif //RAIN_VIO_KEYFRAME_H
