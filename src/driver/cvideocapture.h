#ifndef CVIDEOCAPTURE_H
#define CVIDEOCAPTURE_H
#include "cframe.h"

class CVideoCapture : public CFrame
{
public:
    CVideoCapture(string avi_path):CFrame()
    {
        std::cout<<"video capture construct "<< m_avi_path<<std::endl;
        m_capture.open(m_avi_path);
        if(!m_capture.isOpened())
        {
            std::cout<<"could not read this capture "<<m_avi_path<<std::endl;
            return ;
        }
    }
    CVideoCapture(int dev_num):CFrame()
    {
        std::cout<<"video capture construct device num"<< dev_num<<std::endl;
        m_capture.open(dev_num);
        if(!m_capture.isOpened())
        {
            std::cout<<"could not read this capture "<<dev_num<<std::endl;
            return ;
        }
    }
    CVideoCapture(std::string rtps_ip, int flag):CFrame()
    {
        std::cout<<"video capture construct "<< rtps_ip<<std::endl;
        m_capture.open(rtps_ip, flag);
        if(!m_capture.isOpened())
        {
            std::cout<<"could not read this capture "<<rtps_ip<<std::endl;
            return ;
        }
    }
    ~CVideoCapture()
    {
        m_capture.release();
    }
    void writeImage() override
    {
        m_time_stamp = getTimeStamp();
        cv::Mat m_bgr_img;
        m_capture.read(m_bgr_img);
        return;
    }
private:
    cv::VideoCapture m_capture;
    string m_avi_path;
};

#endif // CVIDEOCAPTURE_H
