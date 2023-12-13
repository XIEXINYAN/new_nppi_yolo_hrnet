#include "cfrontcameraalg.h"
#include "../common/ccameraparam.h"
#include "distance/ccdisfromlidar.h"
#include "distance/cdisfromheight.h"
#include <cmath>

CFrontCameraAlg::CFrontCameraAlg()
{
    m_frame_ptr.reset(new CFrontFrame());
    m_detect_ptr=new CameraDetect(CCameraParam::m_short_flag+CCameraParam::m_long_flag);
    if(CCameraParam::m_short_flag)
    {
        if(CCameraParam::m_lidar_flag)
        {
            m_short_dis_ptr.reset(new CDisFromLidar("short"));
        }
        else
        {
            m_short_dis_ptr.reset(new CDisFromHeight("short"));
        }
    }
    if(CCameraParam::m_long_flag)
    {
        if(CCameraParam::m_lidar_flag)
        {
            m_long_dis_ptr.reset(new CDisFromLidar("long"));
        }
        else
        {
            m_long_dis_ptr.reset(new CDisFromHeight("long"));
        }
    }
    cout<<"front alg inited"<<endl;
}
CFrontCameraAlg::~CFrontCameraAlg()
{

}
void CFrontCameraAlg::process()
{
    auto start = std::chrono::high_resolution_clock::now();
    long long time;
    float* data;
    data=m_frame_ptr->getData();
    std::vector<std::vector<Object>> obj;
    obj = m_detect_ptr->cameraDetectProcess(data);
    vector<vector<cv::Point2d>> pts_2d_all;
    if(CCameraParam::m_short_flag && CCameraParam::m_long_flag)
    {
        vector<cv::Point2d> pts_2d_short=m_short_dis_ptr->getObjetDis(obj[0]);
        vector<cv::Point2d> pts_2d_long=m_long_dis_ptr->getObjetDis(obj[1]);
        pts_2d_all.push_back(pts_2d_short);
        pts_2d_all.push_back(pts_2d_long);
    }
    else if(CCameraParam::m_short_flag)
    {
        vector<cv::Point2d> pts_2d_short=m_short_dis_ptr->getObjetDis(obj[0]);
        pts_2d_all.push_back(pts_2d_short);
    }
    else if(CCameraParam::m_long_flag)
    {
        vector<cv::Point2d> pts_2d_long=m_long_dis_ptr->getObjetDis(obj[0]);
        pts_2d_all.push_back(pts_2d_long);
    }
    if(CCameraParam::m_result_show)
    {
        showFrame(data, obj,pts_2d_all);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout<<"front all time"<<std::chrono::duration_cast<std::chrono::milliseconds>(end- start).count()<<std::endl;
    return;
}

void CFrontCameraAlg::showFrame(float* data, std::vector<vector<Object>>& det_objs, vector<vector<cv::Point2d>> pts_2d)
{
    int batchsize=CCameraParam::m_short_flag+CCameraParam::m_long_flag;
    cv::Mat img=m_frame_ptr->getRemapImage();
    cv::Mat short_img= img(cv::Rect(0, 0, CCameraParam::m_width, CCameraParam::m_height));
    for (auto ob : det_objs[0])
    {
        cv::rectangle(short_img,
                      cv::Point2d(ob.min_2d[0], ob.min_2d[1]),
                      cv::Point2d(ob.max_2d[0], ob.max_2d[1]),
                      getColor(ob.object_type),
                      2);
        cv::putText(short_img,
                    std::to_string(round(ob.center[0],2))+"  "+
                    std::to_string(round(ob.center[1],2))+"  "+
                    std::to_string(ob.object_type),
                    cv::Point2d(ob.min_2d[0], ob.min_2d[1] + 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.0, getColor(ob.object_type + 2),
                    2);
    }
    for(size_t i=0;i<pts_2d[0].size();++i)
    {
        cv::Point2d p=pts_2d[0][i];
        cv::circle(short_img, p, 2, cv::Scalar(255, 0, 0), 2, 8);
    }
    cv::imshow("short_img", short_img);

    if(2==batchsize)
    {
        cv::Mat long_img = img(cv::Rect(0, CCameraParam::m_height, CCameraParam::m_width, CCameraParam::m_height));
        for (auto ob : det_objs[1])
        {
            cv::rectangle(long_img,
                          cv::Point2d(ob.min_2d[0], ob.min_2d[1]),
                          cv::Point2d(ob.max_2d[0], ob.max_2d[1]),
                          getColor(ob.object_type),
                          2);
            cv::putText(long_img,
                        std::to_string(round(ob.center[0],2))+"  "+
                        std::to_string(round(ob.center[1],2))+"  "+
                        std::to_string(ob.object_type),
                        cv::Point2d(ob.min_2d[0], ob.min_2d[1] + 30),
                        cv::FONT_HERSHEY_PLAIN,
                        1.0, getColor(ob.object_type + 2),
                        2);
        }
        for (size_t i = 0; i < pts_2d[1].size(); ++i)
        {
            cv::Point2d p = pts_2d[1][i];
            cv::circle(long_img, p, 2, cv::Scalar(255, 0, 0), 2, 8);
        }
        cv::imshow("long_img", long_img);
    } 
    cv::waitKey(1);
}
inline cv::Scalar CFrontCameraAlg::getColor(int i)
{
    cv::Scalar color;
    int k = i % 7;
    switch (k)
    {
    case 0:
        color = cv::Scalar(0, 0, 255);
        break;
    case 1:
        color = cv::Scalar(0, 255, 0);
        break;
    case 2:
        color = cv::Scalar(0, 0, 255);
        break;
    case 3:
        color = cv::Scalar(0, 255, 255);
        break;
    case 4:
        color = cv::Scalar(130, 0, 75);
        break;
    case 5:
        color = cv::Scalar(238, 130, 238);
        break;
    case 6:
        color = cv::Scalar(139, 139, 0);
        break;
    default:
        break;
    }
    return color;
}
inline double CFrontCameraAlg::round(double num, int percision)
{
    double factor=pow(10,percision);
    return std::round(num*factor)/factor;
}