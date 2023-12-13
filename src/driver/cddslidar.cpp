#include "cddslidar.h"

CDDSLidar::CDDSLidar()
{
    m_sub_left = new DDS_Topic<PointCloud, DDS_LidarCloud>(LIDAR_LEFT_RAW_CLOUD, 's');
    m_sub_right = new DDS_Topic<PointCloud, DDS_LidarCloud>(LIDAR_RIGHT_RAW_CLOUD, 's');
    std::cout<<"dds lidar init"<<std::endl;
}
vector<cv::Point3f> CDDSLidar::getLidar()
{
    vector<cv::Point3f> pts_3d;
    long long stamp_left = 0;
    long long stamp_right = 0;
    PointCloud data_right, data_left;
    m_sub_left->getData(data_left, stamp_right, stamp_left);
    m_sub_right->getData(data_right, stamp_left, stamp_right);
    for(auto p : data_right)
    {
        if(p.x>-25 && p.x<25 && p.y>3 &&p.y<70)
        {

            pts_3d.push_back(cv::Point3f(p.x, p.y, p.z));
        }
    }
    for(auto p: data_left)
    {
        if(p.x>-25 && p.x<25 && p.y>3 &&p.y<70)
        {

            pts_3d.push_back(cv::Point3f(p.x, p.y, p.z));
        }
    }
    return pts_3d;
}
CDDSLidar::~CDDSLidar()
{
    if(m_sub_left)
    {
        m_sub_left=nullptr;
        delete m_sub_left;
    }
    if(m_sub_right)
    {
        m_sub_right=nullptr;
        delete m_sub_right;
    }
}
