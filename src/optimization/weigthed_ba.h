#ifndef _UCOSLAM_WeightedBAH_
#define _UCOSLAM_WeightedBAH
 #include <opencv2/core.hpp>
#include "stuff/se3.h"
#include "imageparams.h"
namespace ucoslam{

//Implements a bundle adjustment for position estimation using a different weighth for each point
//The idea is to weight the influence of the points according to their matching distance to the keypoint descriptor
//Also marker corners are employed and their influence is also considered, i.e, they have no error in the detection, so they are very robust

class WeightedBA{
public:
    struct Params{
        Params(){}
        int nIters=100;//number of iterations
        bool fixIntrinsics=true;//set the intrisics as fixed
        bool verbose=false;
        double w_markers=0.25;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
    };
    //! \param points [in-out]
    //! \param points_weight weight of the points
    //! \param marker_corners
    //! \param pose_io [in-out] initial pose, and the result will be saved here
    void optimize(  se3 &pose_io,const std::vector<cv::Point3f>  &points,const std::vector<cv::Point2f> & points_2d, const std::vector<float>  &points_weight,
                   const std::vector<cv::Point3f>  &marker_corners,const std::vector<cv::Point2f> & marker_corners_2d,
                  const ImageParams &ip,const Params &p=Params() )throw(std::exception);

private:

};
}

#endif
