/*
 * init a image frame
 *
 */
#ifndef CFRAME_H
#define CFRAME_H
#include <cstring>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include "nppdefs.h"
using namespace std;

#define SHOW_DEUBG 0
class CFrame 
{
public:
    CFrame();
    ~CFrame();
    unsigned char* getData();
protected:
    static unsigned char *d_img_data;
    static unsigned char* yuv_data;
    virtual void writeImage()=0;
    long long getTimeStamp();
    long long m_time_stamp;
    void cudaYUV2RGB(const unsigned char* yuv_ptr);
    int m_width;
    int m_height;
    cudaError_t m_err_api;
private:
    void showFrame(string name="row_data") const;
};
typedef boost::shared_ptr<CFrame>CFramePtr;
#endif // CFRAME_H
