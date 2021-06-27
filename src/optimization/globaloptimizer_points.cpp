#include "globaloptimizer_points.h"
#include "../utils.h"
#include <chrono>

namespace ucoslam{
void GlobalOptimizerPoints::setParams(Slam &TWorld, const ParamSet &p )throw(std::exception){




    auto ToVec=[](const Se3Transform &T,float *sol){
        cv::Mat rv=T.getRvec();
        for(int i=0;i<3;i++)
            sol[i]=rv.ptr<float>(0)[i];
        cv::Mat tv=T.getTvec();
        for(int i=0;i<3;i++)
            sol[3+i]=tv.ptr<float>(0)[i];

    };

    _params=p;
    _params.markerSize=TWorld.markerSize();
    _InvScaleFactors.reserve(TWorld.TheFrameSet.front().scaleFactors.size());
    for(auto f:TWorld.TheFrameSet.front().scaleFactors) _InvScaleFactors.push_back(1./f);


    //Find the frames and points that will be used
    isFixedFrame.resize(TWorld.TheFrameSet.capacity());
    usedFramesIdOpt.resize(TWorld.TheFrameSet.capacity());
    usedPointsIdOpt.resize(TWorld.TheMap.map_points.capacity());
    for(auto &v:usedFramesIdOpt)v=INVALID_IDX;
    for(auto &v:usedPointsIdOpt)v=INVALID_IDX;
    for(auto &v:isFixedFrame)v=false;


    uint32_t opIdFrame=0;//index in the solution vector
    if (_params.used_frames.size()==0){     //if non indicated, use all
        for(auto f:TWorld.TheFrameSet)
            usedFramesIdOpt[f.idx]=opIdFrame++;
    }
    else {//else, set the required ones
        for(auto f:_params.used_frames)
            usedFramesIdOpt[f]=opIdFrame++;
    }

    if(_params.fixFirstFrame )
        _params.fixed_frames.insert(TWorld.TheFrameSet.front().idx);

    //add also the fixed ones (in case they are not in the used_frames map
    for(auto f:_params.fixed_frames) {
        if (usedFramesIdOpt[f]==INVALID_IDX)//add it if not yet
            usedFramesIdOpt[f]=opIdFrame++;
        isFixedFrame[f]=true;
    }
    uint32_t opIdPoint=0;//index in the solution vector

    usedMapPoints.clear();
    //now, go thru points assigning ids
    for( uint32_t f=0;f<usedFramesIdOpt.size();f++){
        if ( usedFramesIdOpt[f]==INVALID_IDX)continue;
        if (  isFixedFrame[f])continue;

        for(auto point_id:TWorld.TheFrameSet[f].ids){
            if ( point_id==INVALID_IDX) continue;
            if (usedPointsIdOpt[point_id]==INVALID_IDX){
                if(TWorld.TheMap.map_points[point_id].frames.size()<2)
                    usedPointsIdOpt[point_id]=INVALID_VISITED_IDX;
                else{
                    usedPointsIdOpt[point_id]=opIdPoint++;
                    assert( std::find(usedMapPoints.begin(),usedMapPoints.end(),point_id)== usedMapPoints.end());
                    usedMapPoints.push_back(point_id);
                    for( const auto &f_info:TWorld.TheMap.map_points[point_id].frames)
                        if (usedFramesIdOpt[f_info.first]==INVALID_IDX){
                            usedFramesIdOpt[f_info.first]=opIdFrame++;
                            isFixedFrame[f_info.first]=true;
                        }
                }
            }
        }
    }

    ///TRANSFER DATA TO UCOSBA Optimizer
    SBAData.camParams.fx=TWorld.TheImageParams.CameraMatrix.at<float>(0,0);
    SBAData.camParams.fy=TWorld.TheImageParams.CameraMatrix.at<float>(1,1);
    SBAData.camParams.cx=TWorld.TheImageParams.CameraMatrix.at<float>(0,2);
    SBAData.camParams.cy=TWorld.TheImageParams.CameraMatrix.at<float>(1,2);
    SBAData.camParams.width=TWorld.TheImageParams.CamSize.width;
    SBAData.camParams.height=TWorld.TheImageParams.CamSize.height;
    SBAData.pointSetInfo.reserve(usedMapPoints.size());
    for(auto mp_id:usedMapPoints){
        MapPoint &mp=TWorld.TheMap.map_points[mp_id];
        ucosba::PointInfo pi;
        cv::Point3f cvp3d=mp.getCoordinates();
        memcpy(&pi.p3d,&cvp3d,3*sizeof(float));
        pi.id=mp_id;
        ucosba::PointProjection pprj;
        for( const auto &f_info:mp.frames)
        {
            assert( usedFramesIdOpt.at(f_info.first)!=INVALID_IDX);
            auto &Kp=TWorld.TheFrameSet[f_info.first].und_kpts[f_info.second];
            memcpy(&pprj.p2d,&Kp.pt,2*sizeof(float));
            pprj.camIndex=usedFramesIdOpt[f_info.first];
            pprj.weight=_InvScaleFactors[Kp.octave];
            pi.pointProjections.push_back(pprj);
        }
        SBAData.pointSetInfo.push_back(pi);
    }
    //now, add the cameras
    ucosba::CameraInfo cinfo;
    for( uint32_t fid=0;fid<usedFramesIdOpt.size();fid++){
        if (usedFramesIdOpt[fid]!=INVALID_IDX)
        {
            cinfo.fixed=isFixedFrame[fid];
            cinfo.id=fid;
            cinfo.pose.set( TWorld.TheFrameSet[fid].pose_f2g.data());
        }
        SBAData.cameraSetInfo.push_back(cinfo);
    }

    cout<<"TEST CHI2="<<SBAData.computeChi2()<<endl;
}




void GlobalOptimizerPoints::optimize(Slam &w, const ParamSet &p )throw(std::exception) {
    ScopedTimerEvents timer("GlobalOptSimple");
    setParams(w,p);
    timer.add("Setparams");
    optimize();
    timer.add("Optmize");
    getResults(w);
    timer.add("getResults");

}




void GlobalOptimizerPoints::error(const SparseLevMarq<float>::eVector &sol,SparseLevMarq<float>::eVector &err){
//    //precompute the marker projection matrix
//    vector<cv::Mat> Ks(SlamPtr.TheFrameSet.capacity());
//    uint32_t startFrameSol=SlamPtr.TheMap.map_points.size()*3;
//    for(  auto &frame:SlamPtr.TheFrameSet){
//        se3 pose;
//        for(int i=0;i<6;i++)pose[i]=sol(startFrameSol++);
//        //create the projection matrix
//        Ks[frame.idx]=getFastProjectK(SlamPtr.TheImageParams.CameraMatrix,pose);
//    }

//    //for each point, project
//    err.resize(_errVectorSize);
//    uint32_t sol_idx=0,err_idx=0;
//    for(const auto &p:SlamPtr.TheMap.map_points){
//        cv::Point3f p3d(sol(sol_idx),sol(sol_idx+1),sol(sol_idx+2));sol_idx+=3;

//        float W=1;
//        if ( p.frames.size()>4)W=0.8;
//        if ( p.frames.size()>7)W=0.7;
//        for(auto frame_info:p.frames){
//            cv::Point2f prj=fast__project__( Ks[frame_info.first].ptr<float>(), p3d );
//            cv::Point2f porg=SlamPtr.TheFrameSet[ frame_info.first].und_kpts[ frame_info.second].pt;
//            err(err_idx++)=W*( prj.x-porg.x);
//            err(err_idx++)=W*(prj.y-porg.y);
//        }
//    }
//    assert(err_idx==_errVectorSize);

 }



void GlobalOptimizerPoints::optimize( )throw(std::exception){
    //compute the relative weight of points and marker points

    ucosba::OptimizationParams optParams;
    optParams.minGlobalChi2 =float(_errVectorSize)*1e-1;
    optParams.minStepChi2 =float(_errVectorSize)*1e-2;
    optParams.maxIterations=_params.nIters;

    ucosba::BasicOptimizer optimizer;
    optimizer.optimize(SBAData,optParams);

}

void GlobalOptimizerPoints::getResults(Slam &w)throw(std::exception){
//    uint32_t sidx=0;
//    for(auto &p:w.TheMap.map_points){
//        p.setCoordinates(cv::Point3f(_solution(sidx),_solution(sidx+1),_solution(sidx+2)));
//        sidx+=3;
//    }
//    //now, the frames
//    for(auto &frame:SlamPtr.TheFrameSet){
//         se3 se3_pose_f2g;
//         for(int i=0;i<6;i++)
//             se3_pose_f2g[i]=_solution(sidx++);
//        frame.pose_f2g=se3_pose_f2g;
//    }

}




}
