#ifndef _UCOSLAM_LOOPDETECTOR_H
#define _UCOSLAM_LOOPDETECTOR_H
#include "map.h"

namespace  ucoslam {

class LoopDetector{
    std::shared_ptr<Map> TheMap;
public:



    //Information about to the loop closure
    struct LoopClosureInfo{
        void clear(){
            optimPoses.clear();
            marker_change.clear();
            point_change.clear();
            loopMarkers.clear();
         }

        bool foundLoop() const{ return optimPoses.size()!=0; }



        //reference frame where the loop is detected
        uint32_t curRefFrame=std::numeric_limits<uint32_t>::max();
        //the other side of the loop
        uint32_t matchingFrameIdx;
        //expected pose of current frame it is was estimated with the info from the other side of the loop
        cv::Mat expectedPos;
        //matches between the current frame an the map
        std::vector<cv::DMatch> map_matches;
        std::map<uint32_t, cv::Mat> optimPoses;//new optimized poses for the keyframes
        std::map<uint32_t,se3> marker_change;//how much change must be applied to each marker
        std::map<uint32_t,se3> point_change;//how much change must be applied to each point
        std::vector<uint32_t> loopMarkers;

    };

    void setParams(std::shared_ptr<Map> map);
    LoopClosureInfo  detectLoopFromMarkers(Frame &frame, int32_t curRefKf );
    LoopClosureInfo detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf);
    void correctMap(const LoopClosureInfo &lcsol);


    void toStream(ostream &rtr)const;
    void fromStream(istream &rtr);

private:


    struct KpLoopCandidate{
        std::set<uint32_t> nodes;
        uint32_t nobs;
        uint32_t timesUnobserved=0;
        void toStream(std::ostream &str)const{

            toStream__(nodes,str);
            str.write((char*)&nobs,sizeof(nobs));
            str.write((char*)&timesUnobserved,sizeof(timesUnobserved));
        }
        void fromStream(std::istream &str){

            fromStream__(nodes,str);
            str.read((char*)&nobs,sizeof(nobs));
            str.read((char*)&timesUnobserved,sizeof(timesUnobserved));
        }
    };

    vector<LoopClosureInfo>  detectLoopClosure_Markers(Frame & frame,int64_t curkeyframe);
    std::vector<LoopClosureInfo> detectLoopClosure_KeyPoints( Frame &frame, int32_t curRefKf);

    void solve(Frame &frame,   LoopClosureInfo &loopc_info);
     double testCorrection(const LoopClosureInfo &lcsol);
    void postLoopClosureCorrection(Frame &frame,const LoopClosureInfo &lcsol);

    std::map<uint32_t,KpLoopCandidate > loopCandidates;


//    void correctLoopClosure_Markers(const LoopClosureInfo &lci,  Frame &frame, uint64_t curKeyFrame);
//    void correctLoopClosure_Markers(Frame &frame, int32_t curRefKf,uint32_t matchingFrameIdx,cv ::Mat expectedPose);

//    void correctLoopForKeyPoints(Frame &frame, int32_t curRefKf,uint32_t matchingFrameIdx,cv ::Mat expectedPose);



};
}
#endif

