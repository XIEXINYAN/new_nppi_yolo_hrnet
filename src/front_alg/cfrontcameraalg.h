#ifndef CFRONTCAMERAALG_H
#define CFRONTCAMERAALG_H

#include <chrono>
#include "../driver/cfrontframe.h"
#include "detect/camera_detect.h"
#include "distance/ccameradis.h"

class CFrontCameraAlg
{
public:
    CFrontCameraAlg();
    ~CFrontCameraAlg();
    void process();
private:
    CFrontFramePtr m_frame_ptr;
    CCameraDisPtr m_short_dis_ptr;
    CCameraDisPtr m_long_dis_ptr;
    CameraDetect* m_detect_ptr;
    void showFrame(float* data, std::vector<vector<Object>>& det_objs, vector<vector<cv::Point2d>> pts_2d);
    cv::Scalar getColor(int i);
    double round(double num, int percision);

};
typedef boost::shared_ptr<CFrontCameraAlg> CFrontCameraAlgPtr;

#endif // CFRONTCAMERAALG_H
