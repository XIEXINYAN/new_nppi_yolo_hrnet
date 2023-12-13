#ifndef CV4L2DRIVER_H
#define CV4L2DRIVER_H
#include "stdio.h"
#include <linux/videodev2.h>
#include <getopt.h>
#include "cframe.h"


struct buffer {
    void *                  start;
    size_t                  length;
};

class CV4l2driver : public CFrame
{
public:
    CV4l2driver(int dev_num) : CFrame()
    {
        sprintf(m_dev_name, "/dev/video%d",dev_num);
//        std::cout<<"open device "<<dev_num<<std::endl;
//        m_pixel_format = V4L2_PIX_FMT_YUYV;
        m_pixel_format = V4L2_PIX_FMT_UYVY;
        m_filed = V4L2_FIELD_ANY;
        open_device();
        init_device();
        start_capturing();
        camera_connect_flag = false;
    }
    ~CV4l2driver()
    {
        stop_capturing();
        uninit_device();
        close_device();
    }
    void writeImage() override;
    bool camera_connect_flag=false;
private:
    int m_fd;
    char m_dev_name[15];
    unsigned int m_pixel_format;
    struct buffer *m_buffers;
    unsigned int n_buffers;
    unsigned int m_filed;
    void open_device(void);
    void init_device(void);
    void start_capturing(void);
    void stop_capturing(void);
    void uninit_device(void);
    void close_device(void);
    void init_mmap(void);
};

#endif // CV4L2DRIVER_H
