#ifndef _UCOSLAM_MAP_H
#define _UCOSLAM_MAP_H
#include <mutex>
#include <xflann/xflann.h>
#include "stuff/covisgraph.h"
#include "stuff/debug.h"
#include "stuff/framedatabase.h"
#include "basic_types/mappoint.h"
#include "basic_types/frame.h"
#include "basic_types/marker.h"

namespace ucoslam {


/**The map the environment
 */
class Map {
//static Map*ptr;
public:



    ReusableContainer<MapPoint> map_points;
    std::map<uint32_t,Marker> map_markers;
    FrameSet keyframes;
    CovisGraph TheKpGraph;         //! Covisibility graph from keypoints
    KFDataBase TheKFDataBase;//database of bag of words for images
 public:

    Map();
    bool isEmpty()const{return keyframes.size()==0;};
    void  saveToPcd(std::string filepath,float pointViewDirectionDist=0)throw(std::exception);

    //returns the effective focus of the mpa. If -1, it means that there is no keyframe yet
    float getEffectiveFocus()const;

    //returns the instance of the map
   // static Map* singleton();

    //!
    MapPoint & addNewPoint(uint32_t frameSeqId);
    //adds a mappoint. Returns the point id
    MapPoint & addNewPoint(const MapPoint &mp);
    //adds a new frame in the map and returns a reference to it
    Frame &addKeyFrame(const Frame&f, bool setId=false);

    //adds a marker too the map and returns a reference to it
    Marker &addMarker(const aruco::Marker &m);

    //return a sorted list of the nearest neighbors key frames to the one specified.
    std::vector<uint32_t> getNearestNeighborsKeyFrames(const Frame &frame, size_t nneighbors, float minDist=std::numeric_limits<float>::max());

    //returns the neighbor frames of a given one using both graphs
    std::set<uint32_t> getNeighborKeyFrames(uint32_t fidx,bool includeFidx);

    void addMarkerObservation(uint32_t marker,uint32_t KeyFrame);

    void addMapPointObservation(uint32_t mapPoint,uint32_t KeyFrame,uint32_t KfKPtIdx);
    //remove the indicated points
    void removePoint(uint32_t pid,bool fullRemoval=true);

    //remove the indicated points
    template<typename It>
    void removePoints(It beg, It end,bool fullRemoval=true){    for(auto it=beg;it!=end;it++) removePoint(*it,fullRemoval);}

    //remove mappoint observation
    //returns true if the point has also been removed
    bool removeMapPointObservation(uint32_t kpt, uint32_t frame, uint32_t minNumProjPoints);
    //remove point association and returns the number of frames the point remains visible from
    //If minNumProj>0, the point will be removed if the number of observations falls below this value
   // uint32_t removePointObservation(uint32_t point,uint32_t frame,int32_t minNumProj=0 );
    //returns the median depth of the indicated frame
    float  getFrameMedianDepth(uint32_t frame_idx);

    //remove the indicates keyframes
    //returns  the the points that that after the operation have a the number of projections below minNumProjPoints
    set<uint32_t> removeKeyFrames(const std::set<uint32_t> &keyFrames, int minNumProjPoints);

     //clears the map
    void clear();
    //returns the expected id of the next frame to be inserted
    uint32_t getNextFrameIndex()const;


    //fuses two map points removing the second one.
    void fuseMapPoints(uint32_t mp1, uint32_t mp2 ,bool fullRemovePoint2=false );


    bool hasPointStereoObservations(uint32_t mpIdx)const;

    //checks that data is consistent
    bool checkConsistency(bool checkCovisGraph=false, bool useMutex=true);


    void saveToFile(string fpath)throw(std::exception);
    void readFromFile(std::string  fpath)throw(std::exception);
    //saves the set of markers to a marker map file that can be used with aruco.
    void saveToMarkerMap(std::string filepath)const ;

    //---------------------
    //serialization routines
    void toStream(std::iostream &str) throw (std::exception);
    void fromStream(std::istream &str)throw (std::exception);



    uint64_t getSignature()const;

   // float markerSize()const{return 1;}
    void lock(const std::string &func_name,const std::string &file, int line  ){
       // _debug_msg_("LOCK MAP:"+file+":"+func_name+":"+std::to_string(line));
        IoMutex.lock();
    // _debug_msg_("LOCK MAP GRANTED:"+file+":"+func_name+":"+std::to_string(line));
    }
    void unlock(const std::string &func_name,const std::string &file,int line){
        //_debug_msg_("UN_LOCK MAP:"+file+":"+func_name+":"+std::to_string(line));
        IoMutex.unlock();}

    void addKeyFrameObservation(uint32_t mapId,uint32_t frameId,uint32_t frame_kpidx);


    //returns the ids of the best frames for relocalisation
    vector<uint32_t> relocalizationCandidates(Frame &frame)throw(std::exception);

    //finds the best matches of the frame and the keyframe indicated assuming no initial knowledge of the pose
    vector<cv::DMatch> matchFrameToKeyFrame(const Frame&, uint32_t kf, float minDescDist);
    vector<cv::DMatch> matchFrameToKeyFrame(const Frame&, uint32_t kf, float minDescDist,xflann::Index &xfindex);
    //given a  Frame 'curFrame' and a initial position for it, find the keypoints by projecting the mappoints in the refKFrame KeyFrame and its neighbors
    //returns the points of the map visible in the used_frames that are visible in curFrame
    std::vector<cv::DMatch> matchFrameToMapPoints(const std::vector<uint32_t> &used_frames, Frame & curframe, const cv::Mat & pose_f2g, float minDescDist, float maxRepjDist, bool markMapPointsAsVisible, bool useAllPoints=false);


    double globalReprojChi2(const std::vector<uint32_t> &used_frames ={}, std::vector<float > *chi2vector=0,
                                   std::vector<std::pair<uint32_t,uint32_t> > *map_frame_ids=0, bool useKeyPoints=true, bool useMarkers=false) ;



    //updates the normals and other information of the point when its position or the position of the related frames changes
    void updatePointInfo(uint32_t pid);

    void removeBadAssociations(const vector<std::pair<uint32_t,uint32_t>> &BadAssociations,int minNumPtObs);

    //given the frame passed, returns the best possible estimation of the pose using only the markers in the map
    se3 getBestPoseFromValidMarkers(const Frame &frame,
                                    const vector<uint32_t> &markersOfFrameToBeUsed,
                                    float minErrorRatio,
                                    float max_theta);

    template<typename Iterator>
    std::vector<uint32_t> getMapPointsInFrames(Iterator fstart,Iterator fend);

private:
    std::mutex IoMutex;//mutex to syncronize mapper and tracker
    std::mutex consitencyMutex;//mutex to syncronize mapper and tracker


    std::vector<cv::Vec4f> getPcdPointsLine(const cv::Point3f &a,const cv::Point3f &b,cv::Scalar color,int npoints );
    std::vector<cv::Vec4f> getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints=100 );
    std::vector<cv::Vec4f>  getMarkerIdPcd( cv::Mat rt_g2m_, float _markerSize,uint32_t id,cv::Scalar color );

};

template<typename Iterator>
std::vector<uint32_t> Map::getMapPointsInFrames(Iterator fstart,Iterator fend){

     vector<char> usedpoints(map_points.capacity(),0);
    for(auto f=fstart;f!=fend;f++){
        if (!keyframes.is(*f)) continue;
        const auto &keyframe=keyframes[*f];
        if (keyframe.isBad) continue;
        for(auto id:keyframe.ids){
            if (id!=std::numeric_limits<uint32_t>::max()){
                if (id< usedpoints.size())
                    usedpoints[id]=1;
            }
        }
    }
    std::vector<uint32_t> usedPointIds;
     usedPointIds.reserve(usedpoints.size());
    for(size_t i=0;i<usedpoints.size();i++)
        if (usedpoints[i]){
            if(map_points.is(i))
                if (!map_points[i].isBad)
                    usedPointIds.push_back(i);
        }

    return usedPointIds;
}


}

#endif

