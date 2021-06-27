#include <cvba/bundler.h>
#include "globaloptimizer_ba.h"

namespace ucoslam{

void GlobalOptimizerBA::convert(Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const GlobalOptimizerBA::Params &p, cvba::BundlerData<double> &bdata){
    bdata.resize(fset.size(),map.size());
    std::map<uint32_t,uint32_t> frame_camba;
    ///HHHHERE
    int idx=0;
    for(const auto &p:map.map_points) {
        bdata.points[idx]=p.second.getCoordinates();
        point_pos[p.first]=idx;
        idx++;
    }

    int fidx=0;
    for(const auto &f:fset){
        ip.CameraMatrix.convertTo(bdata.cameraMatrix[fidx],CV_64F);
        bdata.distCoeffs[fidx]=cv::Mat::zeros(1,5,CV_64F);//ip.Distorsion.clone();

        f.second.pose_f2g.getRvec().convertTo(bdata.R[fidx],CV_64F);
        f.second.pose_f2g.getTvec().convertTo(bdata.T[fidx],CV_64F);
        frame_camba[f.first]=fidx;
        fidx++;
    }


    //set value and position
    for(const auto &p:map.map_points){
        for(auto f_i:p.second.frames){
            bdata.visibility [frame_camba[ f_i.first]][point_pos[p.first] ]=1;
            bdata.imagePoints[frame_camba[ f_i.first]][point_pos[p.first]]= fset[f_i.first].und_kpts[ f_i.second ].pt;
        }
    }



}


void GlobalOptimizerBA::convert(cvba::BundlerData<double> &bdata,Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const GlobalOptimizerBA::Params &p){
    {int idx=0;
        for(auto &p:map.map_points)
            p.second.setCoordinates(bdata.points[idx]);
    }
    int fidx=0;
    for(auto &f: fset ){
//        ip.CameraMatrix=bdata.cameraMatrix[fidx];
//        ip.Distorsion=bdata.distCoeffs[fidx];
        cv::Mat aux;
        bdata.R[fidx].convertTo(aux,CV_32F);
        f.second.pose_f2g.setRotation( aux);
        bdata.T[fidx].convertTo(aux,CV_32F);
        f.second.pose_f2g.setTranslation( aux);
        fidx++;
    }

}

void GlobalOptimizerBA::optimize(Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const Params &p)throw(std::exception){
    point_pos.clear();
    _params=p;
    cvba::BundlerData<double> bdata;
    convert(map,fset,ip,markerSize,p,bdata);


    auto bundler=cvba::Bundler::create("pba");
    cvba::Bundler::Params bparams;
    for(auto f:_params.fixed_frames)
        bparams.fixCameraExtrinsics(f);
    bparams.fixAllCameraIntrinsics=true;
    bparams.verbose=_params.verbose;
    bparams.type=cvba::Bundler::MOTIONSTRUCTURE;
    bundler->setParams(bparams);
    bdata.saveToFile("bdata.cvba");
    auto init_err=bdata.avrg_repj_error();
    bundler->run(bdata);
    convert(bdata,map,fset,ip,markerSize,p);
     auto end_err=bdata.avrg_repj_error();
    cerr<<init_err<<" - "<<end_err<<endl;

}


}
