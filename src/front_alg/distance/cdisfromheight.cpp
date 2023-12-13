#include "cdisfromheight.h"

CDisFromHeight::~CDisFromHeight()
{

}
vector<cv::Point2d>  CDisFromHeight::getObjetDis(vector<Object>& objs)
{
    vector<cv::Point2d> pts_2d;
    pts_2d.clear();
    obInit(objs);
    vector<Object>::iterator it=objs.begin();
    for(;it!=objs.end();++it)
    {
        int type = (*it).object_type;
        float land_x = ((*it).min_2d[0] + (*it).max_2d[0]) / 2;
        int land_y = (*it).max_2d[1];
        int h1 = (*it).max_2d[1] - (*it).min_2d[1];
        float real_h = label_heihgt[type];
        float range = real_h * m_fy * cos(m_install_angle) / h1;
        float angle;
        float fovx = (2 * atan2(CCameraParam::m_width, 2*m_fx));
        int dx = land_x - m_u0;
        int mult = 1;
        if (dx < 0)
            mult = -1;
        dx = abs(dx);
        angle = atan2f(2 * dx * tan(fovx / 2), CCameraParam::m_width);
        angle *= mult;
        float dis_long = range * cos(angle);
        float dis_lat = range * sin(angle);
        if (dis_long > 100)
        {
            return pts_2d;
        }
        else
        {
            (*it).center[0] = dis_lat;
            (*it).center[1] = dis_long;
            (*it).center[2] = real_h / 2;
            (*it).min[0] = dis_lat - 1;
            (*it).min[1] = dis_long - 1;
            (*it).min[2] = 0;
            (*it).max[0] = dis_lat + 1;
            (*it).max[1] = dis_long + 1;
            (*it).max[2] = real_h;
            if((*it).object_type!=21)
            {
                (*it).width = 1;
                (*it).height = real_h;
                (*it).length = 1;
            }
        }
    }
    return pts_2d;
}
