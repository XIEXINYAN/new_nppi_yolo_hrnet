#include "ccameradis.h"
#include <cmath>

CCameraDis::CCameraDis(string cam_type)
{
    if ("short"==cam_type)
    {
        cv::FileStorage fcam_set(CCameraParam::m_short_config_file, cv::FileStorage::READ);
        if (!fcam_set.isOpened())
        {
            cout << "camera_config can not open file at  " << CCameraParam::m_short_config_file << endl;
        }
        fcam_set["CameraMat"] >> m_camera_inner;
        fcam_set["ExtrinsicRVec"] >> m_r_mat;
        fcam_set["ExtrinsicTVec"] >> m_t_vec;
        m_fx = m_camera_inner.at<double>(0, 0);
        m_fy = m_camera_inner.at<double>(1, 1);
        m_u0 = m_camera_inner.at<double>(0, 2);
        m_v0 = m_camera_inner.at<double>(1, 2);
        std::cout << "camera dis init param: " << m_camera_inner
                                    << " " << m_r_mat
                                    << "  " << m_t_vec
                                     << std::endl;
        m_install_angle=CCameraParam::m_short_install_angle;
    }
    else if ("long"==cam_type)
    {
        cv::FileStorage fcam_set(CCameraParam::m_long_config_file, cv::FileStorage::READ);
        if (!fcam_set.isOpened())
        {
            cout << "camera_config can not open file at  " << CCameraParam::m_long_config_file << endl;
        }
        fcam_set["CameraMat"] >> m_camera_inner;
        fcam_set["ExtrinsicRVec"] >> m_r_mat;
        fcam_set["ExtrinsicTVec"] >> m_t_vec;
        m_fx = m_camera_inner.at<double>(0, 0);
        m_fy = m_camera_inner.at<double>(1, 1);
        m_u0 = m_camera_inner.at<double>(0, 2);
        m_v0 = m_camera_inner.at<double>(1, 2);
        std::cout << "camera dis init param: " << m_camera_inner 
                                    << " " << m_r_mat
                                    << "  " << m_t_vec
                                     << std::endl;
        
        m_install_angle=CCameraParam::m_long_install_angle;
    }
    return;
}
void CCameraDis::obInit(vector<Object>&objs)
{
    vector<Object>::iterator it=objs.begin();
    for(;it!=objs.end();++it)
    {
        (*it).min[0]=0;
        (*it).min[1]=0;
        (*it).min[2]=0;
        (*it).max[0]=0;
        (*it).center[1]=255;
        (*it).center[0]=0;
        (*it).center[2]=0;
        (*it).max[2]=0;
        (*it).deepInfo=0;
    }
    return;
}

float CCameraDis::getDustThickness(cv::Mat &src)
{
    float res = 0.0;
    double eps;
    int row = src.rows;
    int col = src.cols;
    //    std::cout<<row<<"  "<<col<<std::endl;
    cv::Mat M = cv::Mat::zeros(row, col, CV_8UC1);
    cv::Mat M_ave = cv::Mat::zeros(row, col, CV_8UC1);
    cv::Mat L = cv::Mat::zeros(row, col, CV_8UC1);

    double m_av, A;
    double sum = 0.0;
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            int tmp1 = cv::min(
                cv::min(src.at<cv::Vec3b>(i, j)[1],
                        src.at<cv::Vec3b>(i, j)[0]),
                src.at<cv::Vec3b>(i, j)[2]);
            M.at<uchar>(i, j) = (uchar)tmp1;
            sum += tmp1;
        }
    }
    m_av = sum / (row * col * 255);
    eps = 0.9 / m_av;
    cv::boxFilter(M, M_ave, CV_8UC1, cv::Size(5, 5));
    double delta = cv::min(0.9, eps * m_av);
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            L.at<uchar>(i, j) = cv::min((int)(delta * M_ave.at<uchar>(i, j)), (int)M.at<uchar>(i, j));
        }
    }
    int dust_point = 0;
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            if (L.at<uchar>(i, j) > 110)
            {
                bool flag = true;
                if (flag)
                {
                    dust_point++;
                }
            }
        }
    }
    res = dust_point * 1.0 / (row * col);
    return res;
}
