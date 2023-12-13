#ifndef CCAMERADIS_H
#define CCAMERADIS_H
#include "../driver/cddslidar.h"
#include "object_height.h"
#include <boost/shared_ptr.hpp>
#include "../../common/ccameraparam.h"

class CCameraDis
{
public:
    CCameraDis(string cam_type);
    ~CCameraDis()
    {

    }
    virtual vector<cv::Point2d> getObjetDis(vector<Object>& objs)=0;
protected:
    float m_install_angle;
    cv::Mat m_camera_inner;
    cv::Mat m_r_mat;
    cv::Mat m_t_vec;
    float m_fx,m_u0,m_fy,m_v0;
    void obInit(vector<Object>& objs);
private:
    float getDustThickness(cv::Mat &src);
};
typedef boost::shared_ptr<CCameraDis>CCameraDisPtr;

#endif // CCAMERADIS_H
