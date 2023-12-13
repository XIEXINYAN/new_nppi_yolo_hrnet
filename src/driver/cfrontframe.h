#ifndef CFRONTFRAME
#define CFRONTFRAME
#include "cframe.h"

class CFrontFrame
{
public:
    CFrontFrame();
    ~CFrontFrame();
    float* getData();
    cv::Mat getRemapImage();
private:
    cv::Mat m_bgr_img;
    CFramePtr m_short_ptr;
    CFramePtr m_long_ptr;
    int m_batchsize=0;
    static float* d_yolo_data; //1638400
    static float* d_yolo_data_tmp; //1638400
    static unsigned char* d_dst;//2073600
    static unsigned char* d_dst_uchar;

    static float* d_short_mapx;
    static float* d_short_mapy;
    static float* d_long_mapx;
    static float* d_long_mapy;
    cudaError_t m_err_api;
    const int m_width=640;
    const int m_height=640;
    void setData();
    void cudaRemap(unsigned char* data, float* mapx, float* mapy, int offset);
    void getResizeWHXY(int& resize_w,int&resize_h, int&init_w, int&init_h);
    int m_resize_init_w;
    int m_resize_init_h;
    int m_resize_w;
    int m_resize_h;

    float d_scale={0.003921569}; //1/255.0
};
typedef boost::shared_ptr<CFrontFrame>CFrontFramePtr;



#endif





