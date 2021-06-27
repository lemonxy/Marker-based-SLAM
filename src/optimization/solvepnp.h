
#ifndef SLAMUCO_POSEOPT_H
#define SLAMUCO_POSEOPT_H
#include <opencv2/core.hpp>
#include "stuff/se3.h"
#include "basic_types/frame.h"
#include "map.h"
namespace ucoslam{

void poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d,
                    se3 &pose_io, std::vector<bool> &vBadMatches, const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor,const std::vector<float> &pointConfidence={});


void solvePnp( const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &pose_io ,int64_t currentKeyFrame=-1);

bool solvePnPRansac( const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &pose_io);


}
#endif
