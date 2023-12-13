#include "ccdisfromlidar.h"
#include <pcl/features/normal_3d_omp.h>

CDisFromLidar::~CDisFromLidar()
{
    if(m_lidar_driver_ptr)
    {
        m_lidar_driver_ptr=nullptr;
        delete  m_lidar_driver_ptr;
    }
}

vector<cv::Point2d> CDisFromLidar::getObjetDis(std::vector<Object>& camera_obj)
{
    obInit(camera_obj);
    m_pts_3d.clear();
    m_pts_3d = m_lidar_driver_ptr->getLidar();
    vector<cv::Point2d> pts_2d = project2dCloud();
    objProcess(camera_obj);
    return pts_2d;
    
}
std::vector<cv::Point2d> CDisFromLidar::project2dCloud()
{
    clearPtsContainer();
    std::vector<cv::Point2f> pts_2f;
    pts_2f.clear();
    cv::Mat camera_distor= cv::Mat::zeros(5, 1, CV_32F);
    cv::projectPoints(m_pts_3d, m_r_mat, m_t_vec, m_camera_inner, camera_distor, pts_2f);
    std::vector<cv::Point2d> pts_2d;
    pts_2d.clear();
    for(int i=0;i<pts_2f.size();i++)
    {
        cv::Point2f pf = pts_2f[i];
        if (pf.x <= 0 || pf.x >= CCameraParam::CCameraParam::m_width || pf.y <= 0 || pf.y >= CCameraParam::m_height)
        {
            continue;
        }
        cv::Point2d pd;
        pd.x=(int)pf.x;
        pd.y=(int)pf.y;
        pcl::PointXYZI pi_3d;
        pi_3d.x = m_pts_3d[i].x;
        pi_3d.y = m_pts_3d[i].y;
        pi_3d.z = m_pts_3d[i].z;
        pi_3d.intensity = 1;
        m_pts_container[(int)(pd.y/m_ratio)][(int)(pd.x/m_ratio)].push_back(pi_3d); // if wrong image-x,y
        pts_2d.push_back(pd);
    }
    return pts_2d;
}
void CDisFromLidar::clearPtsContainer()
{
    for (int y = 0; y < CCameraParam::m_height / m_ratio; y++)
    {
        for (int x = 0; x < CCameraParam::m_width / m_ratio; x++)
        {
            m_pts_container[y][x].clear();
        }
    }
    return;
}
void CDisFromLidar::initPtsContainer()
{
    for (int y = 0; y < CCameraParam::m_height / m_ratio; y++)
    {
        std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
        for (int x = 0; x < CCameraParam::m_width / m_ratio; x++)
        {
            std::vector<pcl::PointXYZI> pts_container;
            row_pts_container.push_back(pts_container);
        }
        m_pts_container.push_back(row_pts_container);
    }
    return;
}

bool CDisFromLidar::objProcess(std::vector<Object> &object)
{
    vector<Object>::iterator it=object.begin();
    for(;it!=object.end();it++)
    {
        int min_x = (*it).min_2d[0];
        int min_y = (*it).min_2d[1];
        int max_x = (*it).max_2d[0];
        int max_y = (*it).max_2d[1];
        min_x = min_x < 0 ? 0 : min_x;
        min_y = min_y < 0 ? 0 : min_y;
        max_x = max_x > (CCameraParam::m_width - 1) ? (CCameraParam::m_width - 1) : max_x;
        max_y = max_y > (CCameraParam::m_height - 1) ? (CCameraParam::m_height - 1) : max_y;
        int width = (max_x - min_x);
        int height = (max_y - min_y);
        // cv::Rect re = cv::Rect(min_x,min_y,width,height);
        pcl::PointCloud<pcl::PointXYZI>::Ptr obj_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        obj_ptr->points.clear();
        int num[10];
        memset(num, 0, 10 * sizeof(int));
        int total_cnt = 0;
        for (int i = min_x / m_ratio; i < max_x / m_ratio; i++)
        {
            for (int j = min_y / m_ratio; j < max_y / m_ratio; j++)
            {
                for (int n = 0; n < m_pts_container[j][i].size(); n++)
                {
                    int dis_long = static_cast<int>(m_pts_container[j][i][n].y);
                    num[dis_long / 10]++;
                    total_cnt++;
                }
            }
        }
        if(((*it).object_type<15 || (*it).object_type==18) && total_cnt<10)
        {
            continue;
        }
        if((*it).object_type!=18 && (*it).object_type>=16 && total_cnt<5)
        {
            float dis_long = 0.0;
            float dis_lat = 0.0;
            int cnt = 0;
            while (fabs(dis_lat) < 0.01 && dis_long < 0.1 && cnt < 5)
            {
                min_x -= width;
                max_x += width;
                min_x = std::max(0, min_x);
                max_x = std::min(1919, max_x);
                disCal(min_x, max_x, min_y, max_y, dis_long, dis_lat);
                cnt++;
            }
            if (dis_long > 0.1)
            {
                (*it).center[0] = dis_lat;
                (*it).center[1] = dis_long;
                if ((*it).object_type == 16)
                {
                    (*it).width = 1;
                    (*it).height = 1.75;
                    (*it).length = 1;
                }
                (*it).deepInfo = 1;
                if (0)
                {
                    (*it).min_2d[0] = min_x;
                    (*it).min_2d[1] = min_y;
                    (*it).max_2d[0] = max_x;
                    (*it).max_2d[1] = max_y;
                }
            }
            continue;
        }
        int dis_min=-1;
        int dis_max=0;
        int sum_f=0;
        for (int i = 0; i < 10; i++)
        {
            if (num[i] == 0)
                continue;
            int sum = num[i];
            int tmp_dis_max = 0;
            for (int j = i + 1; j < 10; j++)
            {
                if (num[j] == 0)
                    break;
                sum += num[j];
                tmp_dis_max = j;
            }
            if (sum > sum_f)
            {
                dis_min = i;
                dis_max = tmp_dis_max == 0 ? dis_min : tmp_dis_max;
                dis_max += 1; // max_dis_long the max thred
                sum_f = sum;
            }
        }
        total_cnt=0;
        for (int i = min_x / m_ratio; i < max_x / m_ratio; i++)
        {
            for (int j = min_y / m_ratio; j < max_y / m_ratio; j++)
            {
                for (int num = 0; num < m_pts_container[j][i].size(); num++)
                {
                    if (m_pts_container[j][i][num].y > dis_min * 10 && m_pts_container[j][i][num].y < dis_max * 10)
                    {
                        obj_ptr->points.push_back(m_pts_container[j][i][num]);
                        total_cnt++;
                    }
                }
            }
        }

        if(total_cnt>10)
        {
            buildBox(obj_ptr, &(*it));
        }
        else
        {
            continue;
        }
    }
    return true;
}
void CDisFromLidar::disCal(int min_x, int max_x, int min_y, int max_y, float &dis_long, float &dis_lat)
{
    dis_lat = 0.0;
    dis_long = 0.0;
    int total_cnt = 0;
    for (int i = min_x / m_ratio; i < max_x / m_ratio; i++)
    {
        for (int j = min_y / m_ratio; j < max_y / m_ratio; j++)
        {
            for (int n = 0; n < m_pts_container[j][i].size(); n++)
            {
                dis_long += (int)(m_pts_container[j][i][n].y * 100) / 100.0;
                dis_lat += (int)(m_pts_container[j][i][n].x * 100) / 100.0;
                total_cnt++;
            }
        }
    }
    if (total_cnt == 0)
    {
        return;
    }
    dis_lat /= total_cnt;
    dis_long /= total_cnt;
    assert(fabs(dis_lat) < 100);
    return;
}


bool CDisFromLidar::buildBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, Object *ob)
{
    Eigen::Vector4f centroid, tmp_min, tmp_max;
    pcl::compute3DCentroid(*in_cloud_ptr, centroid);
    pcl::getMinMax3D(*(in_cloud_ptr), tmp_min, tmp_max);
    ob->min[0] = tmp_min[0];
    ob->min[1] = tmp_min[1];
    ob->min[2] = tmp_min[2];
    ob->max[0] = tmp_max[0];
    ob->max[1] = tmp_max[1];
    ob->max[2] = tmp_max[2];
    ob->center[0] = centroid[0];
    ob->center[1] = centroid[1];
    ob->center[2] = centroid[2];
    if(ob->object_type!=21)
    {
        double tmp_length, tmp_height, tmp_width;

        tmp_length = fabs(ob->max[0] - ob->min[0]);
        tmp_width = fabs(ob->max[1] - ob->min[1]);
        tmp_height = fabs(ob->max[2] - ob->min[2]);
        ob->length = tmp_width;
        ob->width = tmp_length;
        ob->height = tmp_height;
        // ob->seq = in_cloud_ptr->points.size();
    }
    ob->deepInfo=1;
    return true;
}

