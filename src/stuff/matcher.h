#ifndef _UCOSLAM_Matcher_
#define _UCOSLAM_Matcher_
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "basic_types/frame.h"
namespace ucoslam{
class Matcher{


public:

    void setReferenceFrame(const Frame &ref);
    void trackPoints(const Frame &frame, float minDescDist, float radiusSearch=2.5 );
    const std::vector<cv::DMatch> getMatches()const{return matches;}

    bool isValid()const{return lastSeenPos.size()>0;}

private:
    Frame _refFrame;
    std::vector<cv::DMatch> matches;
    std::vector<cv::Point2f> lastSeenPos;
    std::vector<uint32_t> timesUnseen;


};
};
#endif

