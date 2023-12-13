#ifndef CDDSLIDAR_H
#define CDDSLIDAR_H
#include <perception/dds_codes/dds_common.hpp>
#include <perception/dds_codes/dds_topic.hpp>

class CDDSLidar
{
public:
    CDDSLidar();
    ~CDDSLidar();
    typedef SPoint PointType;
    typedef pcl::PointCloud<PointType> PointCloud;
    vector<cv::Point3f> getLidar();
private:
    DDS_Topic<PointCloud, DDS_LidarCloud> *m_sub_left;
    DDS_Topic<PointCloud, DDS_LidarCloud> *m_sub_right;
};

#endif // CDDSLIDAR_H
