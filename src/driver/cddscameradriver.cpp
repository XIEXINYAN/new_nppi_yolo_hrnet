#include "cddscameradriver.h"
#include <opencv2/imgproc/types_c.h>

void CDDSCameraDriver::writeImage()
{
    long long time_stamp =0;
    cv::Mat yuv_img;
    m_sub_camera->getData(yuv_img, time_stamp, time_stamp);
    cudaYUV2RGB(yuv_img.data);
    m_time_stamp = time_stamp;
    return;
}
