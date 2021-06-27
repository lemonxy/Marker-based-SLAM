#include "matcher.h"
#include "utils.h"
#include "basic_types/mappoint.h"

namespace ucoslam{
void Matcher::setReferenceFrame(const Frame &ref){
    _refFrame=ref;
    _refFrame.nonMaximaSuppresion();
    uint32_t npoints=ref.und_kpts.size();
    matches.clear();
    lastSeenPos.resize(npoints);
    timesUnseen.resize(npoints);
    for(uint32_t i=0;i<npoints;i++){
        lastSeenPos[i]=    ref.und_kpts[i].pt;
        timesUnseen[i]=0;
    }
}

void Matcher::trackPoints(const Frame &frame, float minDescDist, float radiusSearch){
    matches.clear();
//for each valid point find the best match around the last position
    for(size_t i=0;i<_refFrame.und_kpts.size();i++){
        if (!_refFrame.nonMaxima[i] && timesUnseen[i]<4){
            float scfact=frame.scaleFactors[_refFrame.und_kpts[i].octave];
            //find its correspondece around the last position in the other frame
            auto possible_matches=frame.getKeyPointsInRegion(lastSeenPos[i],scfact*radiusSearch*(1+timesUnseen[i]),_refFrame.und_kpts[i].octave-1,_refFrame.und_kpts[i].octave);
            pair<int,float> best(-1,std::numeric_limits<float>::max());
            for(auto m:possible_matches){
                float dist= MapPoint::getDescDistance(_refFrame.desc.row(i),frame.desc.row(m));
                if (dist <minDescDist && dist<best.second)
                        best={m,dist};
                }
            if (best.first!=-1){
                cv::DMatch match;
                match.trainIdx=i;
                match.queryIdx=best.first;
                match.distance=best.second;
                matches.push_back(match);
            }
        }
    }
    //now, remove ambiguous
    filter_ambiguous_query(matches);
    //set the positions of the valid matches
    for(auto &tu:timesUnseen) tu++;
    for(auto m:matches){
        lastSeenPos[m.trainIdx]=frame.und_kpts[m.queryIdx].pt;
        timesUnseen[m.trainIdx]=0;
    }
}


}
