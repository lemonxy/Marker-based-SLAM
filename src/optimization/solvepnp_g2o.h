#ifndef G2OPOSEOPT_H
#define G2OPOSEOPT_H
#include <opencv2/core.hpp>
#include "stuff/se3.h"
#include "basic_types/frame.h"
#include "map.h"
namespace ucoslam{



void poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d,
                    se3 &pose_io, std::vector<bool> &vBadMatches, const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor,const std::vector<float> &pointConfidence={});

void poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d,
                       se3 &pose_io, std::vector<bool> &vBadMatches, const cv::Mat &CameraParams, float scaleFactor,const std::vector<float> &pointConfidence={});

void poseEstimation(uint32_t currentKeyFrame,const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &pose_io );
}
#endif
