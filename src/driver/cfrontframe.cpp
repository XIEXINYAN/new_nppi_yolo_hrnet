#include "cfrontframe.h"
#include "../common/ccameraparam.h"
#include "cddscameradriver.h"
#include "cimagelist.h"
#include "cv4l2driver.h"
#include "cvideocapture.h"
#include "nppdefs.h"
#include "nppi_geometry_transforms.h"
#include "nppi_arithmetic_and_logical_operations.h"
#include "nppi_data_exchange_and_initialization.h"


float* CFrontFrame::d_yolo_data=nullptr;
float* CFrontFrame::d_yolo_data_tmp=nullptr;
unsigned char* CFrontFrame::d_dst=nullptr;
float *CFrontFrame::d_short_mapx=nullptr;
float *CFrontFrame::d_short_mapy=nullptr;
float *CFrontFrame::d_long_mapx=nullptr;
float *CFrontFrame::d_long_mapy=nullptr;
unsigned char* CFrontFrame::d_dst_uchar=nullptr;

inline void CFrontFrame::getResizeWHXY(int& resize_w,int&resize_h, int&init_w, int&init_h)
{
    float r_w = m_width / (CCameraParam::m_width*1.0);
    float r_h = m_height / (CCameraParam::m_height*1.0);
    if(r_h>r_w)
    {
        resize_w=m_width;
        resize_h=r_w*CCameraParam::m_height;
        init_w=0;
        init_h=(m_height-resize_h)/2;
    }
    else
    {
        resize_w=r_h*CCameraParam::m_width;
        resize_h=m_height;
        init_w=(m_width-resize_w)/2;
        init_h=0;
    }
    return;
}

CFrontFrame::CFrontFrame()
{
    switch (CCameraParam::m_camera)
    {
    case 0:
    {
        if ("" == CCameraParam::m_avi_path)
        {
            if(CCameraParam::m_short_flag)
            {
                m_short_ptr.reset(new CVideoCapture(CCameraParam::m_short_dev_num));
                m_batchsize++;
            }
            if(CCameraParam::m_long_flag)
            {
                m_long_ptr.reset(new CVideoCapture(CCameraParam::m_long_dev_num));
                m_batchsize++;
            }
        }
        else
        {
            if(CCameraParam::m_short_flag)
            {
                m_short_ptr.reset(new CVideoCapture(CCameraParam::m_avi_path));
                m_batchsize++;
            }
            if(CCameraParam::m_long_flag)
            {
                m_long_ptr.reset(new CVideoCapture(CCameraParam::m_avi_path));
                m_batchsize++;
            }
        }
    }
    break;
    case 1:
    {
        if(CCameraParam::m_short_flag)
        {
            m_short_ptr.reset(new CV4l2driver(CCameraParam::m_short_dev_num));
            m_batchsize++;
        }
        if(CCameraParam::m_long_flag)
        {
            m_long_ptr.reset(new CV4l2driver(CCameraParam::m_long_dev_num));
            m_batchsize++;
        }
    }
    break;
    case 2:
    {
        if(CCameraParam::m_short_flag)
        {
            m_short_ptr.reset(new CImageList(CCameraParam::m_images_path));
            m_batchsize++;
        }
        if(CCameraParam::m_long_flag)
        {
            m_long_ptr.reset(new CImageList(CCameraParam::m_images_path));
            m_batchsize++;
        }
    }
    break;
    case 3:
    {
        if(CCameraParam::m_short_flag)
        {
            m_short_ptr.reset(new CDDSCameraDriver(IMAGE_FRONT_RAW));
            m_batchsize++;
        }
        if(CCameraParam::m_long_flag)
        {
            m_long_ptr.reset(new CDDSCameraDriver(IMAGE_FRONT_RAW2));
            m_batchsize++;
        }
    }
    break;
    default:
        break;
    }
    m_err_api = cudaMalloc((void **)&d_yolo_data, m_batchsize * 3 * m_width * m_height * sizeof(float));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_yolo_data cudaMalloc failure" << std::endl;
    }
    m_err_api = cudaMalloc((void **)&d_dst, m_batchsize * 3 * CCameraParam::m_width *CCameraParam::m_height );
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst cudaMalloc failure" << std::endl;
    }
    if(CCameraParam::m_short_flag)
    {
        cv::Mat camera_inner, camera_distor;
        cv::FileStorage fcam_set(CCameraParam::m_short_config_file, cv::FileStorage::READ);
        if (!fcam_set.isOpened())
        {
            cout << "camera_config can not open file at  " << CCameraParam::m_short_config_file << endl;
        }
        fcam_set["CameraMat"] >> camera_inner;
        fcam_set["DisCoffes"] >> camera_distor;
        std::cout << CCameraParam::m_short_config_file << "  " << camera_inner << "  " << camera_distor << std::endl;
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size image_size(CCameraParam::m_width,CCameraParam::m_height);
        cv::Mat mapx = cv::Mat::zeros(image_size, CV_32FC1);
        cv::Mat mapy = cv::Mat::zeros(image_size, CV_32FC1);
        cv::initUndistortRectifyMap(camera_inner,
                                    camera_distor,
                                    R,
                                    camera_inner,
                                    image_size,
                                    CV_32FC1,
                                    mapx,
                                    mapy);

        int memsize1 = CCameraParam::m_width * CCameraParam::m_height * sizeof(float);
        m_err_api = cudaMalloc((void **)&d_short_mapx, memsize1);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_x cudaMalloc failure" << std::endl;
        }
        m_err_api = cudaMalloc((void **)&d_short_mapy, memsize1);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_y cudaMalloc failure" << std::endl;
        }
        m_err_api = cudaMemcpy(d_short_mapx, mapx.data, memsize1, cudaMemcpyHostToDevice);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_x cudaMemcpyHostToDevice failure" << std::endl;
        }
        m_err_api = cudaMemcpy(d_short_mapy, mapy.data, memsize1, cudaMemcpyHostToDevice);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_y cudaMemcpyHostToDevice failure" << std::endl;
        }
    }
    m_err_api =cudaMalloc((void**)&d_dst_uchar, m_batchsize*3*m_width*m_height);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst_uchar cudaMemcpyHostToDevice failure" << std::endl;
    }
    if(CCameraParam::m_long_flag)
    {
        cv::Mat camera_inner, camera_distor;
        cv::FileStorage fcam_set(CCameraParam::m_long_config_file, cv::FileStorage::READ);
        if (!fcam_set.isOpened())
        {
            cout << "camera_config can not open file at  " << CCameraParam::m_long_config_file << endl;
        }
        fcam_set["CameraMat"] >> camera_inner;
        fcam_set["DisCoffes"] >> camera_distor;
        std::cout << CCameraParam::m_long_config_file << "  " << camera_inner << "  " << camera_distor << std::endl;
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size image_size(CCameraParam::m_width, CCameraParam::m_height);
        cv::Mat mapx = cv::Mat::zeros(image_size, CV_32FC1);
        cv::Mat mapy = cv::Mat::zeros(image_size, CV_32FC1);
        cv::initUndistortRectifyMap(camera_inner,
                                    camera_distor,
                                    R,
                                    camera_inner,
                                    image_size,
                                    CV_32FC1,
                                    mapx,
                                    mapy);

        int memsize1 = CCameraParam::m_width * CCameraParam::m_height * sizeof(float);
        m_err_api = cudaMalloc((void **)&d_long_mapx, memsize1);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_x cudaMalloc failure" << std::endl;
        }
        m_err_api = cudaMalloc((void **)&d_long_mapy, memsize1);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_y cudaMalloc failure" << std::endl;
        }
        m_err_api = cudaMemcpy(d_long_mapx, mapx.data, memsize1, cudaMemcpyHostToDevice);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_x cudaMemcpyHostToDevice failure" << std::endl;
        }
        m_err_api = cudaMemcpy(d_long_mapy, mapy.data, memsize1, cudaMemcpyHostToDevice);
        if (m_err_api != cudaSuccess)
        {
            std::cout << "map_y cudaMemcpyHostToDevice failure" << std::endl;
        }
    }
    getResizeWHXY(m_resize_w,m_resize_h,m_resize_init_w,m_resize_init_h);
    if(CCameraParam::m_result_show)
    {
        m_bgr_img.create(m_batchsize*CCameraParam::m_height, CCameraParam::m_width, CV_8UC3);
    }
    m_err_api = cudaMalloc((void**)&d_yolo_data_tmp, m_width * m_height * 3 * sizeof(float));
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_yolo_data_tmp cudaMalloc failure" << std::endl;
    }
    return;

}
CFrontFrame::~CFrontFrame()
{
    m_err_api=cudaFree(d_yolo_data);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_yolo_data cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_yolo_data_tmp);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_yolo_data_tmp cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_dst);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_short_mapx);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_short_mapx cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_short_mapy);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_short_mapy cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_long_mapx);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_long_mapx cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_long_mapy);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_long_mapy cudaFree failure" << std::endl;
    }
    m_err_api=cudaFree(d_dst_uchar);
    if (m_err_api != cudaSuccess)
    {
        std::cout << "d_dst_uchar cudaFree failure" << std::endl;
    }
    return;
}
float* CFrontFrame::getData()
{
    setData();
    return d_yolo_data;
}
cv::Mat CFrontFrame::getRemapImage()
{
    return m_bgr_img;   
}
void CFrontFrame::setData()
{
    // cudaMemset(d_dst,0,m_batchsize*CCameraParam::m_width*CCameraParam::m_height);
    // cudaMemset(d_yolo_data, 0, m_batchsize*m_width*m_height*sizeof(float));

    if(1==m_batchsize)
    {
        unsigned char* data;
        if(CCameraParam::m_short_flag)
        {
            data = m_short_ptr->getData();
            cudaRemap(data,d_short_mapx,d_short_mapy, 0);
        }
        if(CCameraParam::m_long_flag)
        {
            data=m_long_ptr->getData();
            cudaRemap(data, d_long_mapx,d_long_mapy, 0);
        }
    }
    else if(2==m_batchsize)
    {
        unsigned char* short_data;
        short_data = m_short_ptr->getData();
        cudaRemap(short_data,d_short_mapx,d_short_mapy, 0);
        
        unsigned char* long_data;
        long_data = m_long_ptr->getData();
        cudaRemap(long_data,d_long_mapx,d_long_mapy, 1);
    }
    else
    {

    }

    if (CCameraParam::m_result_show)
    {
        m_err_api = cudaMemcpy(m_bgr_img.data, d_dst, CCameraParam::m_height * CCameraParam::m_width * 3*m_batchsize, cudaMemcpyDeviceToHost);
    }
   
    NppStatus status;
    for(int b=0;b<m_batchsize;b++)
    {

        NppiSize srcsize = {CCameraParam::m_width, CCameraParam::m_height};
        NppiRect srcroi = {0, 0, CCameraParam::m_width, CCameraParam::m_height};
        status = nppiResize_8u_C3R(d_dst + b * CCameraParam::m_width * CCameraParam::m_height * 3,
                                   CCameraParam::m_width * 3,
                                   srcsize,
                                   srcroi,
                                   d_dst_uchar + b * m_height * m_width * 3 + m_resize_w * 3 * m_resize_init_h,
                                   m_resize_w * 3,
                                   NppiSize{m_resize_w, m_resize_h},
                                   NppiRect{0, 0, m_resize_w, m_resize_h},
                                   NPPI_INTER_LINEAR);

        if (status != NPP_SUCCESS)
        {
            std::cout << "[GPU] ERROR nppiResize_8u_C3R failed, status = " << status << std::endl;
        }
        if (CCameraParam::m_result_show && SHOW_DEUBG)
        {
            char cc[50];
            sprintf(cc, "resize_img%d.jpg", b);
            std::string sttt = cc;
            cv::Mat resize_img;
            resize_img.create(m_height, m_width, CV_8UC3);
            m_err_api = cudaMemcpy(resize_img.data, d_dst_uchar + b * m_width * m_height * 3, m_width * m_height * 3, cudaMemcpyDeviceToHost);
            cv::resize(resize_img, resize_img, cv::Size(m_width, m_height));
            cv::imshow(sttt, resize_img);
            cv::waitKey(1);
        }
        cudaMemset(d_dst,0,CCameraParam::m_width*CCameraParam::m_height*3);
        Npp8u *r_plane = d_dst+ m_width * m_height * 2;
        Npp8u *g_plane = d_dst+ m_width * m_height;
        Npp8u *b_plane = d_dst;
        Npp8u *d_dst_planes[3] = {b_plane, g_plane, r_plane};
        status = nppiCopy_8u_C3P3R(d_dst_uchar+b*m_width*m_height*3,
                                   m_width*3,
                                   d_dst_planes,
                                   m_width,
                                   NppiSize{m_width, m_height});

        if (status != NPP_SUCCESS)
        {
            std::cout << "[GPU] ERROR nppiCopy_8u_C3P3R failed, status = " << status << std::endl;
        }
        if (CCameraParam::m_result_show && SHOW_DEUBG)
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
            cv::resize(img_merge, img_merge, cv::Size(m_width, m_height));
            char cc[50];
            sprintf(cc, "merge_img%d.jpg", b);
            std::string sttt = cc;
            cv::imshow(sttt, img_merge);
            cv::waitKey(1);
        }
        int yolo_size = m_width*m_height;
        m_err_api = cudaMemset(d_yolo_data+b*m_width*m_height*3, 0.0, yolo_size * 3);
        status = nppiConvert_8u32f_C3R(d_dst, //usined char 
                                       m_width * 3,
                                       d_yolo_data+b*m_width*m_height*3, //float
                                       m_width * 3 * sizeof(float),
                                       NppiSize{m_width, m_height});
        if (status != NPP_SUCCESS)
        {
            std::cout << "[GPU] ERROR nppiConvert_8u32f_C3R failed, status = " << status << std::endl;
        }
        Npp32f *b_bgr = d_yolo_data+b*m_width*m_height*3;
        Npp32f *g_bgr = d_yolo_data+b*m_width*m_height*3 + m_height * m_width;
        Npp32f *r_bgr = d_yolo_data+b*m_width*m_height*3 + 2 * m_height * m_width;
        status = nppiMulC_32f_C1IR(d_scale,
                                   b_bgr,
                                   m_width * sizeof(float),
                                   NppiSize{m_width, m_height});
        status = nppiMulC_32f_C1IR(d_scale,
                                   g_bgr,
                                   m_width * sizeof(float),
                                   NppiSize{m_width, m_height});
        status = nppiMulC_32f_C1IR(d_scale,
                                   r_bgr,
                                   m_width * sizeof(float),
                                   NppiSize{m_width, m_height});
        if (status != NPP_SUCCESS)
        {
            std::cout << "[GPU] ERROR nppiMulC_32f_C3IR failed, status = " << status << std::endl;
        }
        // else if(1==b)
        // {
        //     m_err_api = cudaMemset(d_yolo_data+yolo_size*3, 0.0, yolo_size * 3);
        //     m_err_api = cudaMemset(d_yolo_data_tmp, 0.0, yolo_size * 3);
        //     status = nppiConvert_8u32f_C3R(d_dst,
        //                                    m_width * 3,
        //                                    d_yolo_data_tmp,
        //                                    m_width * 3 * sizeof(float),
        //                                    NppiSize{m_width, m_height});
        //     if (status != NPP_SUCCESS)
        //     {
        //         std::cout << "[GPU] ERROR nppiConvert_8u32f_C3R failed, status = " << status << std::endl;
        //     }
        //     Npp32f *b_bgr = d_yolo_data_tmp;
        //     Npp32f *g_bgr = d_yolo_data_tmp + m_height * m_width;
        //     Npp32f *r_bgr = d_yolo_data_tmp + 2 * m_height * m_width;
        //     status = nppiMulC_32f_C1IR(d_scale,
        //                                b_bgr,
        //                                m_width * sizeof(float),
        //                                NppiSize{m_width, m_height});
        //     status = nppiMulC_32f_C1IR(d_scale,
        //                                g_bgr,
        //                                m_width * sizeof(float),
        //                                NppiSize{m_width, m_height});
        //     status = nppiMulC_32f_C1IR(d_scale,
        //                                r_bgr,
        //                                m_width * sizeof(float),
        //                                NppiSize{m_width, m_height});
        //     if (status != NPP_SUCCESS)
        //     {
        //         std::cout << "[GPU] ERROR nppiMulC_32f_C1IR failed, status = " << status << std::endl;
        //     }
        //     m_err_api = cudaMemcpy(d_yolo_data + yolo_size * 3, d_yolo_data_tmp, yolo_size, cudaMemcpyDeviceToDevice);
        //     m_err_api = cudaMemcpy(d_yolo_data + yolo_size * 3 + m_width * m_height, d_yolo_data_tmp + m_width * m_height, yolo_size, cudaMemcpyDeviceToDevice);
        //     m_err_api = cudaMemcpy(d_yolo_data + yolo_size * 3 + 2*m_width * m_height, d_yolo_data_tmp + 2*m_width * m_height, yolo_size, cudaMemcpyDeviceToDevice);
        // 
        // }
        if (CCameraParam::m_result_show && SHOW_DEUBG)
        {
            cv::Mat img_merge_float, channels[3];
            img_merge_float.create(m_height, m_width, CV_32FC3);
            for (int i = 0; i < 3; i++)
            {
                channels[i].create(m_height, m_width, CV_32FC1);
            }
            m_err_api = cudaMemcpy(channels[0].data, d_yolo_data +b*m_width*m_height*3 + m_width * m_height * 2, yolo_size, cudaMemcpyDeviceToHost);
            m_err_api = cudaMemcpy(channels[1].data, d_yolo_data +b*m_width*m_height*3 + m_width * m_height, yolo_size, cudaMemcpyDeviceToHost);
            m_err_api = cudaMemcpy(channels[2].data, d_yolo_data +b*m_width*m_height*3, yolo_size, cudaMemcpyDeviceToHost);
            cv::merge(channels, 3, img_merge_float);
            cv::Mat timg(m_height, m_width, CV_32FC3, cv::Scalar(255.0, 255.0, 255.0));
            img_merge_float = img_merge_float.mul(timg);
            cv::Mat img;
            img_merge_float.convertTo(img, CV_8UC3);
            cv::resize(img, img, cv::Size(m_width, m_height));
            char cc[50];
            sprintf(cc, "merge_final%d.jpg", b);
            std::string sttt = cc;
            cv::imshow(sttt, img);
            cv::waitKey(1);
        }
    }
   
    //  if(0==b)
    //     {

    //         m_err_api = cudaMemset(d_yolo_data, 0.0, yolo_size * 3);
    //         status = nppiConvert_8u32f_C3R(d_dst,
    //                                        m_width * 3,
    //                                        d_yolo_data,
    //                                        m_width * 3 * sizeof(float),
    //                                        NppiSize{m_width, m_height});
    //         if (status != NPP_SUCCESS)
    //         {
    //             std::cout << "[GPU] ERROR nppiConvert_8u32f_C3R failed, status = " << status << std::endl;
    //         }
    //         Npp32f *b_bgr = d_yolo_data;
    //         Npp32f *g_bgr = d_yolo_data + m_height * m_width;
    //         Npp32f *r_bgr = d_yolo_data + 2 * m_height * m_width;
    //         status = nppiMulC_32f_C1IR(d_scale,
    //                                    b_bgr,
    //                                    m_width * sizeof(float),
    //                                    NppiSize{m_width, m_height});
    //         status = nppiMulC_32f_C1IR(d_scale,
    //                                    g_bgr,
    //                                    m_width * sizeof(float),
    //                                    NppiSize{m_width, m_height});
    //         status = nppiMulC_32f_C1IR(d_scale,
    //                                    r_bgr,
    //                                    m_width * sizeof(float),
    //                                    NppiSize{m_width, m_height});
    //         if (status != NPP_SUCCESS)
    //         {
    //             std::cout << "[GPU] ERROR nppiMulC_32f_C3IR failed, status = " << status << std::endl;
    //         }
            
    //     }
    // for (int b = 0; b < m_batchsize; b++)
    // {
        
    //     int yolo_size=m_width*m_height*sizeof(float);
    //     if (CCameraParam::m_result_show && SHOW_DEUBG)
    //     {
    //         cv::Mat img_merge_float, channels[3];
    //         img_merge_float.create(m_height, m_width, CV_32FC3);
    //         for (int i = 0; i < 3; i++)
    //         {
    //             channels[i].create(m_height, m_width, CV_32FC1);
    //         }
    //         m_err_api = cudaMemcpy(channels[0].data, d_yolo_data+m_width * m_height * 2 + b*yolo_size*3, yolo_size, cudaMemcpyDeviceToHost);
    //         m_err_api = cudaMemcpy(channels[1].data, d_yolo_data+m_width * m_height + b*yolo_size*3, yolo_size, cudaMemcpyDeviceToHost);
    //         m_err_api = cudaMemcpy(channels[2].data, d_yolo_data+b*yolo_size*3, yolo_size, cudaMemcpyDeviceToHost);
    //         cv::merge(channels, 3, img_merge_float);
    //         // cv::Mat timg(m_height, m_width, CV_32FC3, cv::Scalar(255.0, 255.0, 255.0));
    //         // img_merge_float = img_merge_float.mul(timg);
    //         cv::Mat img;
    //         img_merge_float.convertTo(img, CV_8UC3);
    //         cv::resize(img, img, cv::Size(m_width, m_height));
    //         char cc[50];
    //         sprintf(cc, "merge_final%d.jpg", b);
    //         std::string sttt = cc;
    //         cv::imshow(sttt, img);
    //         cv::waitKey(1);
    //     }
    // }
    return;
}

void CFrontFrame::cudaRemap(unsigned char* data, float* mapx, float* mapy, int offset)
{
    NppStatus status;

    NppiSize srcsize = {CCameraParam::m_width, CCameraParam::m_height};
    NppiRect srcroi = {0, 0, CCameraParam::m_width, CCameraParam::m_height};

    status = nppiRemap_8u_C3R((Npp8u *)data,
                              srcsize,
                              CCameraParam::m_width * 3,
                              srcroi,
                              mapx,
                              (CCameraParam::m_width * sizeof(float)),
                              mapy,
                              (CCameraParam::m_width * sizeof(float)),
                              (Npp8u *)d_dst + offset * CCameraParam::m_width * CCameraParam::m_height * 3,
                              CCameraParam::m_width * 3,
                              srcsize,
                              NPPI_INTER_LINEAR);
    if (status != NPP_SUCCESS)
    {
        std::cout << "[GPU] ERROR nppiRemap_8u_C3R failed, status = " << status << std::endl;
        }

}