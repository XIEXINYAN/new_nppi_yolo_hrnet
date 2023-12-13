#include "cbackframe.h"
#include "../common/ccameraparam.h"
#include "cddscameradriver.h"
#include "cimagelist.h"
#include "cv4l2driver.h"
#include "cvideocapture.h"
#include "nppi_arithmetic_and_logical_operations.h"
#include "nppi_data_exchange_and_initialization.h"
#include "nppi_geometry_transforms.h"
float* CBackFrame::d_hrnet_data=nullptr;
unsigned char* CBackFrame::d_dst=nullptr;
CBackFrame::CBackFrame()
{
    switch (CCameraParam::m_camera)
    {
    case 0:
    {
        if ("" == CCameraParam::m_avi_path)
        {
            m_frame_ptr.reset(new CVideoCapture(CCameraParam::m_back_dev_num));
        }
        else
        {
            m_frame_ptr.reset(new CVideoCapture(CCameraParam::m_avi_path));
        }
    }
    break;
    case 1:
    {
        m_frame_ptr.reset( new CV4l2driver(CCameraParam::m_back_dev_num));
    }
    break;
    case 2:
    {
        m_frame_ptr.reset(new CImageList(CCameraParam::m_images_path));
    }
    break;
    case 3:
    {
        m_frame_ptr.reset(new CDDSCameraDriver(IMAGE_FRONT_RAW));
    }
    break;
    default:
        break;
    }
    m_err_api = cudaMalloc((void **)&d_hrnet_data, 3 * m_width * m_height * sizeof(float));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_hrnet_data cudaMalloc failure" << std::endl;
    }
    m_err_api = cudaMalloc((void **)&d_dst, 3 * CCameraParam::m_width *CCameraParam::m_height * sizeof(unsigned char));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst cudaMalloc failure" << std::endl;
    }
}
CBackFrame::~CBackFrame()
{
    m_err_api=cudaFree(d_hrnet_data);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_hrnet_data cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_dst);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst cudaFree failure" << std::endl;
    }
}
float* CBackFrame::getData()
{
    m_err_api = cudaMemset(d_hrnet_data, 0,3 * m_width * m_height * sizeof(float));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_hrnet_data cudaMemset failure" << std::endl;
    }
    m_err_api = cudaMemset(d_dst, 0,3 * CCameraParam::m_width *CCameraParam::m_height * sizeof(unsigned char));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst cudaMemset failure" << std::endl;
    }
    unsigned char* raw_data = m_frame_ptr->getData();
    NppStatus status;
    status = nppiResize_8u_C3R(raw_data,
                               CCameraParam::m_width*3,
                               NppiSize{CCameraParam::m_width, CCameraParam::m_height},
                               NppiRect{0, 0, CCameraParam::m_width, CCameraParam::m_height},
                               (unsigned char*)d_hrnet_data,
                               m_width*3,
                               NppiSize{m_width, m_height},
                               NppiRect{0, 0, m_width, m_height},
                               NPPI_INTER_LINEAR);
    if(status != NPP_SUCCESS)
    {
        std::cout << "[GPU] ERROR nppiResize_8u_C3R failed, status = " << status << std::endl;
    }
    if(CCameraParam::m_result_show && SHOW_DEUBG)
    {
        cv::Mat resize_img;
        resize_img.create(m_height, m_width, CV_8UC3);
        m_err_api = cudaMemcpy(resize_img.data, d_hrnet_data, m_height * m_width * 3, cudaMemcpyDeviceToHost);
        cv::imshow("resize_img_back", resize_img);
        cv::waitKey(1);
    }

    //split
    m_err_api=cudaMemset(d_dst,0,CCameraParam::m_width*CCameraParam::m_height*3*sizeof(unsigned char));
    Npp8u* r_plane = d_dst+m_width*m_height*2;
    Npp8u* g_plane = d_dst+m_width*m_height;
    Npp8u* b_plane = d_dst;
    Npp8u* d_dst_planes[3]={b_plane, g_plane, r_plane};
    status = nppiCopy_8u_C3P3R((unsigned char*)d_hrnet_data,
                               m_width*3,
                               d_dst_planes,
                               m_width,
                               NppiSize{m_width, m_height});
    if(status != NPP_SUCCESS)
    {
        std::cout << "[GPU] ERROR nppiCopy_8u_C3P3R failed, status = " << status << std::endl;
    }
    if(CCameraParam::m_result_show && SHOW_DEUBG)
    {
        cv::Mat img_merge;
        img_merge.create(m_height, m_width, CV_8UC3);
        cv::Mat channels[3];
        for (int i = 0; i < 3; i++)
        {
            channels[i].create(m_height, m_width, CV_8UC1);
        }
        // bgr
        m_err_api = cudaMemcpy(channels[0].data, r_plane, m_height * m_width, cudaMemcpyDeviceToHost);
        m_err_api = cudaMemcpy(channels[1].data, g_plane, m_height * m_width, cudaMemcpyDeviceToHost);
        m_err_api = cudaMemcpy(channels[2].data, b_plane, m_height * m_width, cudaMemcpyDeviceToHost);
        cv::merge(channels, 3, img_merge);
        cv::imshow("merge_img", img_merge);
        cv::waitKey(1);
    }
    
    // 6220800 -- float small == unsigned char big
    m_err_api=cudaMemset(d_hrnet_data,0,m_width*m_height*3*sizeof(float));
    status = nppiConvert_8u32f_C3R(d_dst,
                                   m_width*3,
                                   d_hrnet_data,
                                   m_width*3*sizeof(float),
                                   NppiSize{m_width, m_height});
    if (status != NPP_SUCCESS)
    {
        std::cout << "[GPU] ERROR nppiConvert_8u32f_C3R failed, status = " << status << std::endl;
    }
    Npp32f* b_bgr=d_hrnet_data;
    Npp32f* g_bgr=d_hrnet_data+m_width*m_height;
    Npp32f* r_bgr=d_hrnet_data+2*m_width*m_height;
    int step = m_width * sizeof(float);
    NppiSize size= {m_width, m_height};
    status = nppiMulC_32f_C1IR(d_scale,b_bgr,step,size);
    status = nppiAddC_32f_C1IR(d_hrnet_mean[0],b_bgr,step,size);
    status = nppiMulC_32f_C1IR(d_hrnet_std[0],b_bgr,step,size);
    status = nppiMulC_32f_C1IR(d_scale,g_bgr,step,size);
    status = nppiAddC_32f_C1IR(d_hrnet_mean[1],g_bgr,step,size);
    status = nppiMulC_32f_C1IR(d_hrnet_std[1],g_bgr,step,size);
    status = nppiMulC_32f_C1IR(d_scale,r_bgr,step,size);
    status = nppiAddC_32f_C1IR(d_hrnet_mean[2],r_bgr,step,size);
    status = nppiMulC_32f_C1IR(d_hrnet_std[2],r_bgr,step,size);
    if(CCameraParam::m_result_show && SHOW_DEUBG)
    {
        cv::Mat img_merge2;
        img_merge2.create(m_height, m_width, CV_32FC3);
        cv::Mat channels2[3];
        for (int i = 0; i < 3; i++)
        {
            channels2[i].create(m_height, m_width, CV_32FC1);
        }

        cudaError_t m_err_api;
        m_err_api = cudaMemcpy(channels2[0].data, d_hrnet_data+2*m_height*m_width, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);
        m_err_api = cudaMemcpy(channels2[1].data, d_hrnet_data+m_height*m_width, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);
        m_err_api = cudaMemcpy(channels2[2].data, d_hrnet_data, m_height * m_width * sizeof(float), cudaMemcpyDeviceToHost);
        
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

        cv::merge(channels2, 3, img_merge2);
        cv::Mat float_img;
        float_img.create(m_height, m_width, CV_8UC3);
        img_merge2.convertTo(float_img, CV_8UC3);
        cv::imshow("final_image_back", float_img);
        cv::waitKey(1);
    }

    return d_hrnet_data;
}