#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_POINTS_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_POINTS_H_

#include "slam.h"
#include "sparselevmarq.h"
#include <unordered_map>
#include <unordered_set>
#include "globaloptimizer.h"
#include "ucosba.h"

#include "proj_info.h" //to remove from here
namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class   GlobalOptimizerPoints: public GlobalOptimizer{
public:

    void setParams(Slam &TWorld, const ParamSet &p=ParamSet() )throw(std::exception);
    void optimize()throw(std::exception) ;
    void getResults(Slam &w)throw(std::exception);

    void optimize(Slam &w,const ParamSet &p=ParamSet() )throw(std::exception) ;
    string getName()const{return "points";}

private:
     void error(const SparseLevMarq<float>::eVector &sol,SparseLevMarq<float>::eVector &err);





    ParamSet _params;

    uint32_t _errVectorSize;
    cv::Mat cameraMatrix;
    float fx,fy,cx,cy;
    SparseLevMarq<float>::eVector  _solution;
    cv::Mat LPm;//precomputed left part of projection matrix
    SparseLevMarq<float> optimizer;


    void saveToStream_impl(std::ostream &str){};
    void readFromStream_impl(std::istream &str){};



    vector<std::pair<uint32_t,uint32_t> > _badAssociations;
    vector<float> _InvScaleFactors;

    vector<uint32_t> usedFramesIdOpt,usedPointsIdOpt,usedMapPoints;
    vector<char> isFixedFrame;
    const uint32_t INVALID_IDX=std::numeric_limits<uint32_t>::max();
    const uint32_t INVALID_VISITED_IDX=std::numeric_limits<uint32_t>::max()-1;


    ucosba::SBA_Data SBAData;

};
}
#endif
