#ifndef UCOSLAM_SLAM_H
#define UCOSLAM_SLAM_H
#include "ucoslam_exports.h"
#include <map>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <aruco/marker.h>
#include <aruco/markerdetector.h>
#include <memory>
#include "stuff/se3.h"
#include "stuff/covisgraph.h"
#include "stuff/frameextractor.h"
#include "stuff/framedatabase.h"
#include "map.h"
#include "mapmanager.h"
#include "ucoslam.h"
namespace ucoslam{
class KPFrameDataBase;
class MapInitializer;
class UCOSLAM_API Slam{
    friend class slam_sglcvviewer;
    friend class ucoslam_Debugger;
public:
    enum STATE{STATE_ZERO,STATE_TRACKING,STATE_LOST};
    enum MODES{MODE_SLAM,MODE_LOCALIZATION};


    Slam();
    ~Slam();

    //Clear this object setting it to intial state
    void clear();
//set the image pararms
    void setParams(std::shared_ptr<Map> map, const  Params &p, const string &vocabulary="");
    static Params   getParams() {return _params;}

    //main input  returns global to camera matrix
    cv::Mat process(  cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx, const cv::Mat & depth=cv::Mat());


     //pose is given as the transform from  global ref system  to current frame position (f<-g)
    se3 getCurrentPose_f2g()const{return _curPose_f2g;}

    //Reset the current frame pose. Use it to start tracking in a known map
     void resetTracker();
//Returns the poses stored during the processs
    map<uint32_t,se3> getFramePoses()const{ return frame_pose;}




    //sets the system mode
    void setMode(MODES mode){
        if (mode==MODE_LOCALIZATION) currentState=STATE_LOST;
            currentMode=mode;
    }


    // I/O

    void saveToFile(std::string filepath)throw(std::exception);
    void readFromFile(std::string filepath)throw(std::exception);
    friend ostream& operator<<(ostream& os, const Slam & w);


    //  DEBUG
    //calculates the global reprojection error of all points in all frames
    uint64_t getSignature()const;
  //  bool checkConsistency();
    uint32_t getLastProcessedFrame()const{return _cFrame.fseq_idx;}



    static Params _params;
    std::shared_ptr<Map> TheMap;
    FrameExtractor fextractor;
    uint32_t num_of_image;
    uint32_t num_of_success;

    // TODO: temporary public methods
    //! \brief Estimate current pose using either aruco or the database
    //! pose_out is invalid if the pose is not correcly estimated. If correct estimation, the currentKeyFrame is also indicated.
    //! if keyFrame==-1 value, then a new keyframe should be added
    bool relocalize(Frame &f, se3 &pose_f2g_out ) ;
    bool relocalize_withkeypoints( Frame &f,se3 &pose_f2g_out );
    bool relocalize_withmarkers(Frame &f, se3 &pose_f2g_out) ;


    //only for debugging pourposes perform the global optimization
    void globalOptimization();


    cv::Mat process(const Frame &frame) ;

    //waits for all threads to finish
    void waitForFinished();
private:
    void drawMatches(cv::Mat &image,float inv_ScaleFactor )const;//return a drawable image from last process

    //returns the last valid pose looking in  frame_pose map
    pair<uint32_t,se3> getLastValidPose()const;
    //compute speed using last 20 valid frames
    float computeSpeed()const;


    // DEBUG
    //converts a uint64 to a string for visualization in debug mode
    string sigtostring(uint64_t);

    //Pose estimation
     uint32_t getBestReferenceFrame(const Frame &curKeyFrame,  const se3 &curPose_f2g);





    cv::Mat getPoseFromMarkersInMap(const Frame &frame);
   // bool initialize_depth(const cv::Mat &in_image, const cv::Mat & depth, const ImageParams ip, uint32_t frameseq_idx);


    bool initialize(Frame &f2);
    bool initialize_stereo(Frame &frame);
    bool initialize_monocular(Frame &f2) ;
    se3 track(Frame &f, se3 lastKnownPose);


    //! \brief For the map points in the frame  indicated, finds its nearest match constrained by spatial distance. Descriptors of map points are updated and frame keypoints are associated to map points
    //! \param used_frame  frame from the map to be used
    //! \param distThr Spatial distance (i.e. radius) in pixels. Def. 15
    //! \return In match, 'trainIdx' corresponds to map points and 'queryIdx' corresponds to keypoints of curframe
//     std::vector<cv::DMatch> matchMapPtsOnFrame(const std::vector<uint32_t > &  used_frames, Frame & curframe, const cv::Mat & pose, float minDescDist, float maxRepjDist, bool markMapPointsAsVisible);

    //!\brief given lastframe, current and the map of points, search for the map points  seen in lastframe in current frame
    //Returns query-(id position in the currentFrame). Train (id point in the TheMap.map_points)

    std::vector<cv::DMatch> matchMapPtsOnPrevFrame(Frame & curframe, Frame &prev_frame,  float minDescDist, float maxReprjDist);


    void putText(cv::Mat &im,string text,cv::Point p );


    //used internally
     Frame _cFrame,_prevFrame;//current and previous Frame
    std::shared_ptr<MapInitializer> map_initializer;
    bool isInitialized=false;
    se3 _curPose_f2g;//current pose
    int64_t _curKFRef=-1;//current reference key frame    
     STATE currentState=STATE_ZERO;
    MODES currentMode=MODE_SLAM;
    MapManager TheMapManager;
    map<uint32_t,se3> frame_pose;//for each processed frame, the computed pose





};
}

#endif // BASIC_TYPES_H
