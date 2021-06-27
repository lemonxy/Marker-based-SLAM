#ifndef _UCOSLAM_GlobalOptimizerH_
#define _UCOSLAM_GlobalOptimizerH_
#include "../map.h"
#include <unordered_set>
namespace ucoslam{
class Slam;

/**Base class for global optimization of points,markers and camera locations
 */
class GlobalOptimizer{
 public:
    struct ParamSet {
        ParamSet(bool Verbose=false):verbose(Verbose){}
        std::unordered_set<uint32_t> used_frames;//which are used. If empty, all
        std::set<uint32_t> fixed_frames;//which are set as not movable
        bool fixFirstFrame=true;
        float minStepErr=1e-2;
        float minErrPerPixel=0.5;//below this value, the optimization stops
        int nIters=100;//number of iterations
        bool verbose=false;
        float w_markers=0.25;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
        //---do not use from here
        float markerSize=-1;
    };

    //set the required params
    virtual void setParams(std::shared_ptr<Map>   map, const ParamSet &ps )=0;
    virtual void optimize()throw(std::exception) =0;
    virtual void getResults(std::shared_ptr<Map> map)throw(std::exception)=0;

    //one funtion to do everything
    virtual void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() )throw(std::exception)=0;
    virtual string getName()const=0;

    //returns a vector of mapPointId,FrameId indicating the bad associations that should be removed
    virtual vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return {};}


    static std::shared_ptr<GlobalOptimizer> create(string type="")throw(std::exception);

    void saveToStream(std::ostream &str);
    void readFromStream(std::istream &str);
protected:
    virtual void saveToStream_impl(std::ostream &str)=0;
    virtual void readFromStream_impl(std::istream &str)=0;
};
}
#endif
