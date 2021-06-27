#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_G2O_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_G2O_H_
#include "slam.h"
#include "globaloptimizer.h"

#include "proj_info.h" //to remove from here
namespace g2o{
class SparseOptimizer;
};

namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class   GlobalOptimizerG2O: public GlobalOptimizer{
public:

    void setParams(std::shared_ptr<Map> map, const ParamSet &p=ParamSet() )throw(std::exception);
    void optimize()throw(std::exception) ;
    void getResults(std::shared_ptr<Map> map)throw(std::exception);

    void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() )throw(std::exception) ;
    vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return _badAssociations;}

    string getName()const{return "g2o";}

private:





    ParamSet _params;


    void saveToStream_impl(std::ostream &str){};
    void readFromStream_impl(std::istream &str){};
    std::shared_ptr<g2o::SparseOptimizer> Optimizer;


    uint64_t join(uint32_t a ,uint32_t b){
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }
    inline pair<uint32_t,uint32_t> separe(uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);}



    vector<std::pair<uint32_t,uint32_t> > _badAssociations;
    vector<float> _InvScaleFactors;
    struct edge_frameId_stereo{
        edge_frameId_stereo(void *f,uint32_t s,bool isSt){
            first=f;
            second=s;
            isStereo=isSt;
        }
        void *first;
        uint32_t second;
        bool isStereo=false;
    };
    std::vector<std::vector< edge_frameId_stereo > > point_edges_frameId;
    uint32_t totalNEdges=0;
//    vector<ucoslam::MarkerEdge>

    vector<uint32_t> usedFramesIdOpt,usedPointsIdOpt,usedMapPoints;
    std::map<uint32_t,uint32_t> usedMarkersIdOp;//for each marker, the id of the vertex employed in the optimizer
    vector<char> isFixedFrame;
    const uint32_t INVALID_IDX=std::numeric_limits<uint32_t>::max();
    const uint32_t INVALID_VISITED_IDX=std::numeric_limits<uint32_t>::max()-1;

    const char UNFIXED=0;
    const char FIXED_WITHOUTPOINTS=1;
    const char FIXED_WITHPOINTS=2;
};
}
#endif
