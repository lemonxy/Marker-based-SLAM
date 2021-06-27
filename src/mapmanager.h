#ifndef MapManager_H
#define MapManager_H

#include <memory>
#include <mutex>
#include <thread>
#include <mutex>
#include <atomic>
#include "stuff/tsqueue.h"
#include "stuff/loopdetector.h"
#include "map.h"
namespace  ucoslam {
class Map;
class GlobalOptimizer;

//Class that runs in parallel taking care of the map
class MapManager{
friend class ucoslam_Debugger;

public:

    enum STATE { IDLE,WORKING,WAITINGFORUPDATE};

    MapManager();
    ~MapManager();
    void setMap(std::shared_ptr<Map> map){TheMap=map;_TheLoopDetector.setParams(map);}
    bool hasMap()const{ return !(!TheMap);}

    void start();
    void stop();
    //call whenever a new frame is avaiable.
    //return 0 if nothing is done
    int newFrame(Frame &kf,  int32_t curkeyFrame);
    //applies the changes to map require tracking to be stoped, such as removing frames  or points
    Se3Transform mapUpdate();

    void toStream(std::ostream &str) ;
    void fromStream(istream &str);

    uint64_t getCounter()const {return _counter;}
private:


    std::shared_ptr<Map> TheMap;
    void runThread();
    void mainFunction();
    std::thread _TThread;
    std::mutex mutex_addKf;
    bool _mustAddKeyFrame=false;

    Frame &addKeyFrame(Frame *f);
    bool mustAddKeyFrame(const Frame &frame_in , uint32_t curKFRef);
    bool mustAddKeyFrame_Markers(const Frame & frame_in , uint32_t curKFRef);
    bool mustAddKeyFrame_KeyPoints(const Frame & frame_in, uint32_t curKFRef );
    bool  mustAddKeyFrame_stereo(const Frame &frame_in,uint32_t curKFRef);


    list< MapPoint>  createNewPoints(Frame &NewFrame , uint32_t nn=20);
    std::list<MapPoint> createCloseStereoPoints(Frame & newFrame, int nn=10);
    vector<uint32_t>  mapPointsCulling();
    void  localOptimization(uint32_t _newKFId);
    void  globalOptimization(int niters=10 );

     set<uint32_t> keyFrameCulling(uint32_t keyframe_idx) ;
    set<uint32_t> keyFrameCulling_Markers(uint32_t keyframe_idx);
    set<uint32_t> keyFrameCulling_KeyPoints(uint32_t keyframe_idx);

    ///BIT ADD
    double computeTheta(Frame f1, Frame f2);

    inline uint64_t join(uint32_t a ,uint32_t b){
       uint64_t a_b;
       uint32_t *_a_b_16=(uint32_t*)&a_b;
       _a_b_16[0]=a;
       _a_b_16[1]=b;
       return a_b;
   }

    std::atomic<STATE> _curState;

   bool mustExit=false;
   TSQueue<Frame*>  keyframesToAdd;
  // std::map<uint32_t,uint64_t> unStablePoints;//points recently added yet on probation
    uint64_t _counter=0;
   uint32_t _CurkeyFrame=std::numeric_limits<uint32_t>::max();//current keyframe of the tracker



   //we need to save now
   std::shared_ptr<GlobalOptimizer> Gopt;
   vector<uint32_t> PointsToRemove;
   set<uint32_t> KeyFramesToRemove;

   //

    vector<uint32_t> getMatchingFrames(Frame &NewFrame, size_t n)    ;


  //  void correctLoop(Frame &f, int32_t curRefKf, uint32_t matchingFrameIdx, cv::Mat estimatedPose);
   //adds the keyframe, remove useless points, and do global optimization
   void loopClosurePostProcessing(Frame &frame, const LoopDetector::LoopClosureInfo &lci);


   struct LoopCandidate{
       std::set<uint32_t> nodes;
       uint32_t nobs;
       uint32_t timesUnobserved=0;
       void toStream(std::ostream &str)const;
       void fromStream(std::istream &str);
   };

   //std::map<uint32_t,LoopCandidate > loopCandidates;
   vector<uint32_t> searchInNeighbors(Frame &mpCurrentKeyFrame);
   std::map<uint32_t,uint32_t > youngKeyFrames;


    LoopDetector _TheLoopDetector;
    LoopDetector::LoopClosureInfo _LoopClosureInfo;
    uint32_t _lastAddedKeyFrame;
    bool _hasMapBeenScaled=false;
};

}

#endif
