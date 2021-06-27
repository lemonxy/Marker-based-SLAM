#include "globaloptimizer.h"
#include "globaloptimizer_simple.h"
#include "globaloptimizer_points.h"
#include "globaloptimizer_g2o.h"
#include "globaloptimizer_full.h"
//#ifdef USE_CVBA
//#include "globaloptimizer_cvba_pba.h"
//#include "globaloptimizer_cvba_ucosba.h"
//#endif
namespace ucoslam{
std::shared_ptr<GlobalOptimizer> GlobalOptimizer::create(string type)throw(std::exception){
    if(type.empty() || type=="g2o")
        return std::make_shared<GlobalOptimizerG2O>();

//    return std::make_shared<GlobalOptimizerSimple>();
//    else if(type=="full")
//        return std::make_shared<GlobalOptimizerFull>();
//    else if (type=="points")
//        return std::make_shared<GlobalOptimizerPoints>();
//    else
//        if( type=="g2o")


//#ifdef USE_CVBA
//    else if (type=="cvba_pba"){
//        return std::make_shared<GlobalOptimizerCVBA_PBA>();
//    }
//#endif
//#ifdef USE_CVBA
//    else if (type=="cvba_uco"){
//        return std::make_shared<GlobalOptimizerCVBA_UCO>();
//    }
//#endif

    else throw std::runtime_error("GlobalOptimizer::create could not load the required optimizer");
}


void GlobalOptimizer::saveToStream(std::ostream &str){

}

void GlobalOptimizer::readFromStream(std::istream &str){

}
}
