#ifndef UCOSLAM_H
#define UCOSLAM_H
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>
namespace ucoslam {

//types of descriptors that can be used


//Processing parameters for the SLAM
struct Params{
    Params();
    bool detectMarkers=true;
    bool aprilTag = false;
     float effectiveFocus=-1;//value to enable normalization across different cameras and resolutions
    bool removeKeyPointsIntoMarkers=true;
    bool forceInitializationFromMarkers=false;
    float minDescDistance=50;//minimum distance between descriptors to consider a possible match
    float baseline_medianDepth_ratio_min=0.01;
    int projDistThr=15;//when searching for points by projection, maximum 2d distance for search radius
     std::string global_optimizer= "g2o";//which global optimizer to use
    int minNumProjPoints=3;//minimum number of keyframes in which a point must be seen to keep it
    float keyFrameCullingPercentage=0.8;
    int fps=30;//Frames per second of the video sequence
    float thRefRatio=0.9;//ratoi of matches found in current frame compared to ref keyframe to consider a new keyframe to be inserted
    int maxFeatures=2000;
    int nOctaveLevels=8;
    float scaleFactor=1.2;


    int maxVisibleFramesPerMarker=10;
    float minBaseLine=0.07;//minimum preffered distance  between keyframes
    float max_makr_rep_err = 2.5;


    float aruco_markerSize=1;            //! Size of markers in meters
    int aruco_minNumFramesRequired=3;            //minimum number of frames
    float aruco_minerrratio_valid=3;//minimum error ratio between two solutions to consider a initial pose valid
    bool aruco_allowOneFrameInitialization=false;
    float aruco_maxRotation=0.1;//minimum rotation between current frame and current keyframe to add this as keyframe
    float min_distR = 1.2;
    float max_theta = 32;
    float delta_theta = 1;
    aruco::MarkerDetector::Params aruco_DetectorParams;


    void toStream(std::ostream &str);
    void fromStream(std::istream &str);
    uint64_t getSignature()const;


    void saveToYMLFile(const std::string &path);
    void readFromYMLFile(const std::string &path);
    //--- do not use
    bool runSequential=false;//avoid parallel processing

    private:
    template<typename Type>
    void attemtpRead(const std::string &name,Type &var,cv::FileStorage&fs ){
        if ( fs[name].type()!=cv::FileNode::NONE)
            fs[name]>>var;
    }


};




}

#endif
