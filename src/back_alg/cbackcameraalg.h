#ifndef CBACKCAMERAALG_H
#define CBACKCAMERAALG_H
#include "hrnetseg.h"
#include "../driver/cbackframe.h"

class CBackCameraAlg
{
public:
    CBackCameraAlg();
    ~CBackCameraAlg();
    void process();
private:
    CBackFramePtr m_frame_ptr;
    HrNetSeg m_seg;
    void showFrame(const float* img_data, const std::vector<cv::Point> point_vec, float mean_height, float mean_std);
};
typedef boost::shared_ptr<CBackCameraAlg> CBackCameraAlgPtr;

#endif // CBACKCAMERAALG_H
