#include "cframe.h"
#include <sys/timeb.h>
#include <math.h>
#include "../common/ccameraparam.h"
#include "nppi_color_conversion.h"

unsigned char* CFrame::d_img_data=nullptr;
unsigned char* CFrame::yuv_data=nullptr;
CFrame::CFrame()
{

    m_width = CCameraParam::m_width;
    m_height = CCameraParam::m_height;
    m_err_api = cudaMalloc((void **)&d_img_data, m_width*m_height * 3 * sizeof(unsigned char));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_img_data cudaMalloc failure" << std::endl;
    }
    m_err_api = cudaMalloc((void **)&yuv_data, m_width * m_height*2*sizeof(unsigned char));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "yuv_data cudaMalloc failure" << std::endl;
    }
}
CFrame::~CFrame()
{
    m_err_api=cudaFree(d_img_data);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_img_data cudaFree failure" << std::endl;
    }
}
unsigned char* CFrame::getData()
{
    writeImage();
    return d_img_data;
}
long long CFrame::getTimeStamp()
{
    struct timeb tb;
    ftime(&tb);
    return tb.time*1000+tb.millitm;
}
void CFrame::cudaYUV2RGB(const unsigned char*  yuv_ptr)
{
    m_err_api=cudaMemcpy(yuv_data, yuv_ptr, m_width*m_height*2*sizeof(unsigned char), cudaMemcpyHostToDevice);
    NppiSize srcsize = {m_width, m_height};
    NppStatus status;
    status= nppiCbYCr422ToRGB_8u_C2C3R(yuv_data,
                                       m_width*2,
                                       d_img_data,
                                       m_width*3,
                                       srcsize);
    if(status != NPP_SUCCESS)
    {
        std::cout << "[GPU] ERROR nppiCbYCr422ToRGB_8u_C2C3R failed, status = " << status << std::endl;
    }
    if(CCameraParam::m_result_show && SHOW_DEUBG && 0)
    {
        showFrame();
    }
    return;
}
void CFrame::showFrame(string name) const
{
    cv::Mat bgr_img;
    bgr_img.create(m_height,m_width,CV_8UC3);
    cudaMemcpy(bgr_img.data, d_img_data,m_width*m_height*3*sizeof(unsigned char), cudaMemcpyDeviceToHost);
    cv::imshow(name, bgr_img);
    cv::waitKey(1);
}
// void CFrame::cudaMemClear()
// {
//     cudaError_t m_err_api;
    
//     m_err_api = cudaMemset(d_dst, 0, m_width * m_height * 3 * sizeof(unsigned char));
//     if (m_err_api != cudaSuccess)
//     {
//         std::cout << "d_dst cudaMemset failure" << std::endl;
//     }
    
//     return;

// }
