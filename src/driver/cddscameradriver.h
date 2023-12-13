#ifndef CDDSCAMERADRIVER_H
#define CDDSCAMERADRIVER_H
#include <perception/dds_codes/dds_common.hpp>
#include <perception/dds_codes/dds_topic.hpp>
#include "cframe.h"
class CDDSCameraDriver:public CFrame
{
public:
    CDDSCameraDriver(string topic):CFrame()
    {
        m_topic=topic;
        m_sub_camera = new DDS_Topic<cv::Mat, DDS_Image>(m_topic,'s');
        std::cout<<"dds sub init "<<m_topic<<std::endl;
    }
    ~CDDSCameraDriver()
    {
        if(m_sub_camera)
        {
            delete m_sub_camera;
            m_sub_camera = nullptr;
        }
    }
private:
    void writeImage() override;
    DDS_Topic<cv::Mat, DDS_Image> *m_sub_camera=nullptr;
    string m_topic;
};

#endif // CDDSCAMERADRIVER_H
