#ifndef CDISFROMHEIGHT_H
#define CDISFROMHEIGHT_H
#include "ccameradis.h"

class CDisFromHeight : public CCameraDis
{
public:
    CDisFromHeight(string cam_type) : CCameraDis(cam_type)
    {}
    ~CDisFromHeight();
    vector<cv::Point2d> getObjetDis(vector<Object>& objs) override;
private:
};

#endif // CDISFROMHEIGHT_H
