#include "cbackcameraalg.h"
#include "../driver/cddscameradriver.h"
#include "../driver/cv4l2driver.h"
#include "../driver/cvideocapture.h"
#include "../driver/cimagelist.h"
#include "../common/ccameraparam.h"

CBackCameraAlg::CBackCameraAlg()
{
    m_frame_ptr.reset(new CBackFrame());
}
CBackCameraAlg::~CBackCameraAlg()
{

}
void CBackCameraAlg::process()
{
    auto start = std::chrono::high_resolution_clock::now();
    long long time;
    float* data;
    data=m_frame_ptr->getData();
    std::vector<cv::Point> point_vec;
    point_vec=m_seg.hrNetSegProcess(data);
    if(CCameraParam::m_result_show)
    {
        showFrame(data, point_vec,0.0,0.0);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout<<"back all time"<<std::chrono::duration_cast<std::chrono::milliseconds>(end- start).count()<<std::endl;

}
void CBackCameraAlg::showFrame(const float* img_data, const std::vector<cv::Point> point_vec, float mean_height, float mean_std)
{

    float d_hrnet_mean[3]={-0.515523,-0.526210, -0.545487}; //1/255.0
    float d_hrnet_std[3]={18.95, 18.44, 19.15}; //1/255.0
    int m_height=540;
    int m_width=960;
    cv::Mat img;
    img.create(m_height, m_width, CV_32FC3);
    cv::Mat channels2[3];
    for (int i = 0; i < 3; i++)
    {
        channels2[i].create(m_height, m_width, CV_32FC1);
    }

    cudaError_t m_err_api;
    m_err_api = cudaMemcpy(channels2[0].data, img_data + 2 * m_height * m_width, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);
    m_err_api = cudaMemcpy(channels2[1].data, img_data + m_height * m_width, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);
    m_err_api = cudaMemcpy(channels2[2].data, img_data, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);

    cv::Mat timg(m_height, m_width, CV_32FC1, cv::Scalar(1 / d_hrnet_std[0]));
    channels2[2] = channels2[2].mul(timg);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(-d_hrnet_mean[0]));
    cv::add(channels2[2], timg, channels2[2]);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(255.0));
    channels2[2] = channels2[2].mul(timg);

    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(1 / d_hrnet_std[1]));
    channels2[1] = channels2[1].mul(timg);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(-d_hrnet_mean[1]));
    cv::add(channels2[1], timg, channels2[1]);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(255.0));
    channels2[1] = channels2[1].mul(timg);

    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(1 / d_hrnet_std[2]));
    channels2[0] = channels2[0].mul(timg);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(-d_hrnet_mean[2]));
    cv::add(channels2[0], timg, channels2[0]);
    timg = cv::Mat(m_height, m_width, CV_32FC1, cv::Scalar(255.0));
    channels2[0] = channels2[0].mul(timg);

    cv::merge(channels2, 3, img);
    cv::Mat float_img;
    float_img.create(m_height, m_width, CV_8UC3);
    img.convertTo(float_img, CV_8UC3);

    for (int i = 0; i < point_vec.size(); i++)
    {
        cv::Scalar color(255, 0, 0);
        cv::circle(float_img, point_vec[i], 1, color, 8);
    }

    if (fabs(mean_std) > 10)
    {
        cv::putText(float_img,
                    std::to_string(mean_height) + "  " + std::to_string(mean_std),
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_PLAIN,
                    2.0,
                    cv::Scalar(0x00, 0xff, 0x00),
                    2);
    }
    else
    {
        cv::putText(float_img,
                    std::to_string(mean_height) + "  " + std::to_string(mean_std),
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_PLAIN,
                    2.0,
                    cv::Scalar(0xff, 0x00, 0x00),
                    2);
    }
    cv::imshow("back_seg_img", float_img);
    cv::waitKey(1);
}
