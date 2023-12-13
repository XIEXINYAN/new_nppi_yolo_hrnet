#ifndef CCDISFROMLIDAR_H
#define CCDISFROMLIDAR_H
#include "ccameradis.h"
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>

class CDisFromLidar : public CCameraDis
{
public:
    CDisFromLidar(string cam_type) : CCameraDis(cam_type)
    {
         m_lidar_driver_ptr= new CDDSLidar();
         initPtsContainer();
         std::cout<<"dis from lidar init"<<std::endl;
    }
    ~CDisFromLidar();
    vector<cv::Point2d> getObjetDis(std::vector<Object>& camera_obj) override;
private:
    void initPtsContainer();
    vector<cv::Point3f> m_pts_3d;
    std::vector<std::vector<std::vector<pcl::PointXYZI>>> m_pts_container;
    CDDSLidar* m_lidar_driver_ptr=nullptr;
    std::vector<cv::Point2d> project2dCloud();
    void clearPtsContainer();
    bool objProcess(std::vector<Object> &object);
    void disCal(int min_x, int max_x, int min_y, int max_y, float &dis_long, float &dis_lat);
    bool buildBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, Object *ob);
    const int m_ratio{10};
};

#endif // CCDISFROMLIDAR_H
