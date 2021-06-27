#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_CVBAPBA_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_CVBAPBA_H_
#include "globaloptimizer.h"
#include <cvba/bundler.h>

namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class GlobalOptimizerCVBA_PBA: public GlobalOptimizer{
public:

    void setParams(Slam &w, const ParamSet &p=ParamSet(), vector<float> *ScaleFactors=0)throw(std::exception);
    void optimize()throw(std::exception) ;
    void getResults(Slam &w)throw(std::exception);

    void optimize(Slam &w,const ParamSet &p=ParamSet())throw(std::exception) ;
    string getName()const{return "cvba";}
    void saveToStream_impl(std::ostream &str);
    void readFromStream_impl(std::istream &str);

private:
    ParamSet _params;
    cvba::BundlerData<float> _bdata;

    void  convert(const Slam &sl,   ParamSet &p, cvba::BundlerData<float> &bdata);
    std::map<uint32_t,uint32_t> frame_index;//for each frame, an index in the bundler data
std::map<uint32_t,uint32_t> mappoints_used;
 std::map<uint32_t,uint32_t> mapmarkers_used;
cv::Mat cameraMatrix;
};

}
#endif
