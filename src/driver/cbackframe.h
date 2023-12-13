#ifndef CBACKFRAME_H
#define CBACKFRAME_H
#include "cframe.h"
class CBackFrame
{
public:
    CBackFrame();
    ~CBackFrame();
    float* getData();
private:
    CFramePtr m_frame_ptr;
    static float* d_hrnet_data;
    static unsigned char* d_dst;
    cudaError_t m_err_api;
    const int m_width=960;
    const int m_height=540;

    float d_scale={0.003921569}; //1/255.0
    float d_hrnet_mean[3]={-0.515523,-0.526210, -0.545487}; //1/255.0
    float d_hrnet_std[3]={18.95, 18.44, 19.15}; //1/255.0
};
typedef boost::shared_ptr<CBackFrame>CBackFramePtr;

#endif // CBACKFRAME_H
