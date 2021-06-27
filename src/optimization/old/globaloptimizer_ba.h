#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_BA_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_BA_H_
#include "basic_types.h"
#include "exports.h"
#include "sparselevmarq.h"
#include <cvba/bundler.h>
#include <set>
namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class GlobalOptimizerBA{
public:
    struct Params{
        friend class GlobalOptimizer;
        Params(){}
        std::set<uint32_t> fixed_frames;//set of frames to set fixed
        int nIters=100;//number of iterations
        bool fixIntrinsics=true;//set the intrisics as fixed
        bool verbose=false;
        double w_markers=0.25;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
        //---do not use from here
    };

    void optimize(Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const Params &p=Params())throw(std::exception);
private:
    void convert(Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const GlobalOptimizerBA::Params &p, cvba::BundlerData<double> &bdata);
    void convert(cvba::BundlerData<double> &bdata,Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const GlobalOptimizerBA::Params &p);

    Params _params;
    std::unordered_map<uint32_t,uint32_t> point_pos;

};
}
#endif
