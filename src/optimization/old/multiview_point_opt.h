#ifndef  _MULTIVIEW_POINT_OPT_H
#define _MULTIVIEW_POINT_OPT_H

#include <opencv2/core.hpp>
#include <vector>
#include "stuff/se3.h"
namespace ucoslam{
class MultiViewPointOpt{
public:
    //uses no distortion
    cv::Point3f optimize(cv::Point3f &p, const std::vector<cv::Point2f> &projs, const std::vector<se3> &poses, cv::Mat &CameraMatrix);

private:
    struct proj_info{
        float x,y;
        float dx[9];//dx_rx,dx_ry,dx_rz,dx_tx,dx_ty,dx_tz,dx_X,dx_Y,dx_Z
        float dy[9];//dy_rx,dy_ry,dy_rz,dy_tx,dy_ty,dy_tz,dy_X,dy_Y,dy_Z
    };
    proj_info project_and_derive(const cv::Point3f  &point, const se3 &rt);

    float fx,fy,cx,cy;
};

}

#endif
