//#include <cvba/bundler.h>
#include "globaloptimizer_simple.h"
#include "../utils.h"
#include <chrono>


namespace ucoslam{
void GlobalOptimizerSimple::setParams(Slam &w, const ParamSet &p )throw(std::exception){

    _params=p;
    _params.markerSize=w.markerSize();
     convert(w,_params,_solution);

}



void GlobalOptimizerSimple::convert(Slam &sl,  ParamSet &p, SparseLevMarq<float>::eVector &solution){

    Map &map=sl.TheMap;
    FrameSet &fset=sl.TheFrameSet;
    ImageParams &ip=sl.TheImageParams;


    //determine the frames to be used
    //if non indicated, use all
    if (p.used_frames.size()==0)
        for(const auto &f:fset)
            p.used_frames.insert(f.idx);

    //add also the fixed ones (in case they are not in the used_frames map
    for(auto f:p.fixed_frames) p.used_frames.insert(f);
    //determine the set of points to be optimized
    pmap_info.clear();
    markermap_info.clear();
    for(auto f:p.used_frames){//go through frames, and add the id of the mappoints projecting in them
        if (p.fixed_frames.count(f)==0){//if not fixed
            auto &frame=fset[f];
            for(auto id:frame.ids){//go through keypoints assignaions
                if (id!=std::numeric_limits<uint32_t>::max())//if not invalid
                    if (pmap_info.count(id)==0)//not already inserted
                        pmap_info[id]=point_info();//insert
            }
            //now, the markers
            for(const auto &m:frame.markers){
                assert(map.map_markers.count(m.id));
                if ( map.map_markers.at(m.id).pose_g2m.isValid()  )
                    markermap_info[m.id]=marker_info();
            }
        }
    }
    //fix the first frame to avoid infinite solutions
    //do not do it before, or its points and markers wont be included
   if (p.fixFirstFrame)  p.fixed_frames.insert(fset.front().idx);

    solution.resize(pmap_info.size()*3+6*(p.used_frames.size()-p.fixed_frames.size())+6*markermap_info.size());
    //first, all points, then the frames except the first one
    //p0_x,p0_y,p0_z,p1_x.... | f1_rx,f1_ry...
    int sol_idx=0;//index in the solution vector
    for(auto &pi:pmap_info){//for each used map point
        auto &map_point=map.map_points[pi.first];//retrieve the map point
        auto p3d=map_point.getCoordinates();//
        solution[sol_idx]=(p3d.x);//copy data in solution vector
        solution[sol_idx+1]=(p3d.y);
        solution[sol_idx+2]=(p3d.z);
        //set projection info
        pi.second.sol_idx=sol_idx;
        //add the projections of the point
        pi.second.frame_projections.reserve(map_point.frames.size());
        for(auto f_i:map_point.frames)//each map knows in which frames it projects and the keypoint position in the vector of frame keypoints
            if (p.used_frames.count(f_i.first))//it is a used frame
                pi.second.frame_projections.push_back(point_prj_info(f_i.first,fset[f_i.first].und_kpts[f_i.second].pt ) );
        sol_idx+=3;//increment solution index counter
    }


    //now, the frame locations
    frame_solpos.clear();
    for(const auto &f:p.used_frames){
        auto &frame=fset[f];
        if (!p.fixed_frames.count(f)){//if not fixed, add its info to the vector
            frame_solpos[f]=sol_idx;//register the location and save in solution vector
            se3 se3_pose_f2g=frame.pose_f2g;
            for(int i=0;i<6;i++) solution[sol_idx++]= se3_pose_f2g[i];
        }
        //fixed frame, save its pose for later
        else fixedframe_poses[f]=frame.pose_f2g;
    }


    //add marker params
    for(auto &mi:markermap_info){
        const auto &marker=sl.TheMap.map_markers[mi.first];
        mi.second.sol_idx=sol_idx;
        for(int i=0;i<6;i++) solution[sol_idx++]= marker.pose_g2m[i];
        //now, register the location it is seen in each frame
        for(auto fidx:marker.frames)
            if (p.used_frames.count(fidx))
                mi.second.frame_projections.push_back(marker_prj_info( fidx, fset[fidx].getMarker(mi.first)));
    }

    assert(sol_idx==solution.size());
    //copy camera intrinsics for later
    assert(ip.CameraMatrix.type()==CV_32F);
    assert(ip.CameraMatrix.total()==9);
    assert(ip.CameraMatrix.isContinuous());
    cameraMatrix=ip.CameraMatrix.clone();

    //let us speed up the proces by changing the key of frame_projections for an index in a vector with the set of frames employed.
    //it will have impact in the project function since we avoid continous search in a map
    {
        std::map<uint32_t,uint32_t> frame_indices;
        frame_solpos_fsi.clear();
        int idx=0;
        for(const auto &f:p.used_frames) {
            frame_indices.insert(std::make_pair(f,idx++));
            frame_solpos_fsi.push_back(frame_solpos[f]);
            fixed_frames_fsi.push_back( p.fixed_frames.count(f));
        }
        //now, do the substitution
        for(auto &pi:pmap_info)//for each used map point
            for(auto &f_i:pi.second.frame_projections){
                assert(frame_indices.count(f_i.frame_idx));
                f_i.frame_sol_idx=frame_indices[f_i.frame_idx ];//replace here
            }
        //now, do the substitution for markers
        for(auto &mi:markermap_info)//for each used map point
            for(auto &f_i:mi.second.frame_projections){
                assert(frame_indices.count(f_i.frame_idx));
                f_i.frame_sol_idx=frame_indices[f_i.frame_idx ];//replace here
            }
    }


    //precomputed projection matrix lhs
    //  |fx 0   cx |   |1 0 0 0|
    //  |0  fy  cy | * |0 1 0 0|
    //  |0  0   1  |   |0 0 1 0|
     LPm=cv::Mat::eye(4,4,CV_32F);
     LPm.at<float>(0,0)=sl.TheImageParams.CameraMatrix.at<float>(0,0);
     LPm.at<float>(0,2)=sl.TheImageParams.CameraMatrix.at<float>(0,2);
     LPm.at<float>(1,1)=sl.TheImageParams.CameraMatrix.at<float>(1,1);
     LPm.at<float>(1,2)=sl.TheImageParams.CameraMatrix.at<float>(1,2);

     //finally, compute the error position of each element
     //row in which each element place its error in the error vector

     //for each point, project
     uint32_t e_idx=0;
     for( auto &p:pmap_info){//were it projects
         for(auto &f_p:p.second.frame_projections){
             f_p.errIdx=e_idx;
             e_idx+=2;
         }
     }
     //for each marker, project
     for( auto &m:markermap_info){
         for(auto &f_p:m.second.frame_projections){
             f_p.errIdx=e_idx;
             e_idx+=8;
         }
     }

}



void GlobalOptimizerSimple::optimize(Slam &w, const ParamSet &p  )throw(std::exception) {
    ScopedTimerEvents timer("GlobalOptSimple");
    setParams(w,p);
    timer.add("Setparams");
    optimize();
    timer.add("Optmize");
    getResults(w);
    timer.add("getResults");

}



void GlobalOptimizerSimple::getMarker3dpoints(const se3 &pose_g2m,float markerSize,vector<cv::Point3f>  &corners){
    corners.resize(4);
    //the corner points
    float ms2=markerSize/2.;
    float m_p3d[16]= { -ms2, ms2,0   ,ms2, ms2,0   ,ms2, -ms2,0  , -ms2, -ms2,0  };
    //the transform matrix-
    float rt[16];
    pose_g2m.convert(rt);
    for(int i=0;i<4;i++)
        mult_matrix_point(rt, m_p3d+i*3,(float*)&corners[i]);
}

void GlobalOptimizerSimple::error(const SparseLevMarq<float>::eVector &sol,SparseLevMarq<float>::eVector &err){
    //ScopeTimer timer("GlobalOptimizerSimple::error");



    err.resize(_errVectorSize);
    std::vector<cv::Mat> frame_PrjMatrix;//precomputed poses for fast calculation
    for(const auto &f:_params.used_frames)
        frame_PrjMatrix.push_back(  (LPm*getFramePose(sol,f).convert()));
    //for each point, project
    for(const auto &p:pmap_info){
        cv::Vec4f p3d(sol(p.second.sol_idx),sol(p.second.sol_idx+1),sol(p.second.sol_idx+2),1);
        //were it projects
        for(auto &f_p:p.second.frame_projections){
            //project the point
            auto prj=fast__project__(frame_PrjMatrix[f_p.frame_sol_idx].ptr<float>(0),(float*)&p3d );
            auto w=hubber_weight(fabs(prj.x-f_p.proj.x)+fabs(prj.y-f_p.proj.y),2.5);
            err(f_p.errIdx)  =w*(prj.x-f_p.proj.x);
            err(f_p.errIdx+1)=w*(prj.y-f_p.proj.y);
        }
    }
    //for each marker, project
    vector<cv::Point2f> m_points(4);
    for(const auto &m:markermap_info){
        se3 pose_g2m;
        for(int i=0;i<6;i++) pose_g2m[i]=sol(m.second.sol_idx+i);
        //get the 3d points
        vector<cv::Point3f> m_p3d(4);
        getMarker3dpoints(pose_g2m,_params.markerSize,m_p3d);
        //project them in each frame
        for(auto f_p:m.second.frame_projections){
            float *rt_t=frame_PrjMatrix[f_p.frame_sol_idx].ptr<float>(0);//get camera projection matrix
            for(int c=0;c<4;c++){//project the four points
                auto prj=fast__project__(rt_t,(float*)&m_p3d[c]);
                err(f_p.errIdx+c*2)  =/*_weight_markers* */(prj.x-f_p.proj[c].x);
                err(f_p.errIdx+c*2+1)=/*_weight_markers* */(prj.y-f_p.proj[c].y);
            }
        }
    }
}


void GlobalOptimizerSimple::jacobian(const SparseLevMarq<float>::eVector &sol, Eigen::SparseMatrix<float> &Jac){
    ScopeTimer timer("GlobalOptimizerSimple::jacobian");

    Jac.resize(_errVectorSize,sol.size());
    std::vector<se3> frame_pose;//precomputed poses for fast calculation
    std::vector<cv::Mat> frame_PrjMatrix;//precomputed poses for fast calculation
    for(const auto &f:_params.used_frames){
        auto fp=getFramePose(sol,f);
        frame_pose.push_back(fp);
        frame_PrjMatrix.push_back( (LPm*fp.convert()));
    }
    triplets.clear();
    triplets.reserve(_errVectorSize*9);
    //for each point, project
    for(const auto &p:pmap_info){
        cv::Point3f p3d(sol(p.second.sol_idx),sol(p.second.sol_idx+1),sol(p.second.sol_idx+2));
        //were it projects
        for(auto &f_p:p.second.frame_projections){                
            auto pinfo=project_and_derive(p3d,frame_pose[f_p.frame_sol_idx],cameraMatrix);
             auto f_sp=frame_solpos_fsi[f_p.frame_sol_idx];//staring column of the frame params
            //add derivatives of frame (rx,ry...tz), if not fixed
            if (!fixed_frames_fsi[f_p.frame_sol_idx] )//(!_params.fixed_frames.count(f_p.frame_idx)){
                for(int i=0;i<6;i++){
                    if( pinfo.dx[i]!=0)
                        triplets.push_back( Eigen::Triplet<float> (f_p.errIdx,f_sp+i,pinfo.dx[i]));
                    if( pinfo.dy[i]!=0)
                        triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+1,f_sp+i,pinfo.dy[i]));
                }
            //add derivatives of point x,y,z
            for(int i=0;i<3;i++){
                if ( pinfo.dx[6+i]!=0)
                triplets.push_back( Eigen::Triplet<float> (f_p.errIdx,p.second.sol_idx+i,pinfo.dx[6+i]));
                if ( pinfo.dy[6+i]!=0)
                triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+1,p.second.sol_idx+i,pinfo.dy[6+i]));
            }
        }
    }


    for(const auto &m:markermap_info){
        se3 pose_g2m;
        for(int i=0;i<6;i++) pose_g2m[i]=sol(m.second.sol_idx+i);
        vector<cv::Point3f> m_p3d(4);
        getMarker3dpoints(pose_g2m,_params.markerSize,m_p3d);
        //find marker repoj error derivatives wrt camera locations
        for(auto f_p:m.second.frame_projections){
            if (!fixed_frames_fsi[f_p.frame_sol_idx] ){//(!_params.fixed_frames.count(f_p.frame_idx)){
//test
               auto f_sp=frame_solpos_fsi[f_p.frame_sol_idx];//staring column of the frame params
                for(int c=0;c<4;c++){
                    auto pinfo=project_and_derive(m_p3d[c],frame_pose[f_p.frame_sol_idx],cameraMatrix,true);
                    for(int i=0;i<6;i++){
                        if( pinfo.dx[i]!=0)
                            triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+c*2,f_sp+i,pinfo.dx[i]));
                        if( pinfo.dy[i]!=0)
                            triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+c*2+1,f_sp+i,pinfo.dy[i]));
                    }
                }
            }
        }

        // find error derivatives wrt rxry..
        float delta=1e-3;
        float invdelta2=1./(2*delta);
        for(int d=0;d<6;d++){//for each dimension rx,ry,...
            se3 pose_g2m_p(pose_g2m),pose_g2m_m(pose_g2m);
            pose_g2m_p[d]+=delta;
            pose_g2m_m[d]-=delta;
            vector<cv::Point3f> m_p3d_p(4),m_p3d_m(4);
            getMarker3dpoints(pose_g2m_p,_params.markerSize,m_p3d_p);
            getMarker3dpoints(pose_g2m_m,_params.markerSize,m_p3d_m);


            for(auto f_p:m.second.frame_projections){
                float *rt_t=frame_PrjMatrix[f_p.frame_sol_idx].ptr<float>(0);//get camera projection matrix
                //project them in each frame
                for(int c=0;c<4;c++){//project the four points
                    auto p_p=fast__project__(rt_t,(float*)&m_p3d_p[c]);
                    auto p_m=fast__project__(rt_t,(float*)&m_p3d_m[c]);
                    auto err1=invdelta2*(p_p.x-p_m.x);
                    auto err2=invdelta2*(p_p.y-p_m.y);
                    if ( err1!=0)
                        triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+c*2,m.second.sol_idx+d,err1));
                    if ( err2!=0)
                        triplets.push_back( Eigen::Triplet<float> (f_p.errIdx+c*2+1,m.second.sol_idx+d,err2));
                }
            }
         }
     }

    Jac.setFromTriplets(triplets.begin(),triplets.end());
    //cout<<Jac<<endl;
}


void GlobalOptimizerSimple::optimize( )throw(std::exception){


    //compute the relative weight of points and marker points

    double totalmarkers=global_w_markers;
    if ( markermap_info.size()==0) totalmarkers=0;
    if (pmap_info.size()==0)totalmarkers=1;

//    if ( markermap_info.size()!=0)
//        _weight_markers = totalmarkers /  double(markermap_info.size()*4) ;
//    if(pmap_info.size()!=0)
//        _weight_points = (1.0f-totalmarkers)/double(pmap_info.size());

    _weight_markers=1;
    _weight_points=1;

    _debug_msg_( "weight markers="<<_weight_markers<<" _weight_points="<<_weight_points);


    _errVectorSize=0;
    for(const auto &p:pmap_info)  _errVectorSize+=2*p.second.frame_projections.size();
    for(const auto &m:markermap_info) _errVectorSize+=8*m.second.frame_projections.size();
    float minErr=_params.minErrPerPixel*_errVectorSize;
    float minStepErr=_params.minStepErr*_errVectorSize;
    SparseLevMarq<float>::Params sl_params (_params.nIters,minErr,minStepErr,1e-3);
    sl_params.use_omp=false;
    sl_params.verbose=_params.verbose;
    optimizer.setParams(sl_params);
    if (_params.verbose)cerr<<"minErr="<<minErr<<" minStepErr="<<minStepErr<<endl;


  //  optimizer.setStepCallBackFunc( std::bind(&GlobalOptimizerSimple::StepCallBackFunc,this,std::placeholders::_1));
    if (1)  optimizer.solve( _solution,std::bind(&GlobalOptimizerSimple::error,this,std::placeholders::_1,std::placeholders::_2), std::bind(&GlobalOptimizerSimple::jacobian,this,std::placeholders::_1,std::placeholders::_2) );
    else
        optimizer.solve( _solution,std::bind(&GlobalOptimizerSimple::error,this,std::placeholders::_1,std::placeholders::_2));
}
void GlobalOptimizerSimple::StepCallBackFunc( const SparseLevMarq<float>::eVector  &sol){
    auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };
     auto separe=[](uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);};

    auto getMap=[&]( Eigen::SparseMatrix<float>  &J){
        std::map<uint64_t,float> sJmap;
            for (int k=0; k<J.outerSize(); ++k)
              for ( Eigen::SparseMatrix<float> ::InnerIterator it(J,k); it; ++it)
                  sJmap.insert( {join(it.row(),it.col()),it.value()});
            return sJmap;
    };



    Eigen::SparseMatrix<float> J2(_errVectorSize,sol.size());
    jacobian(sol,J2);

    Eigen::SparseMatrix<float> sJ(_errVectorSize,sol.size());
    optimizer.calcDerivates(sol,sJ ,std::bind(&GlobalOptimizerSimple::error,this,std::placeholders::_1,std::placeholders::_2));

    cout<<endl;
    auto sJmap=getMap(sJ);
    auto sJ2map=getMap(J2);
    //find big differencs
    for(auto j:sJmap){
        if (sJ2map.count(j.first)==0 ){
            auto rc=separe(j.first);
            cerr<<"error in="<<rc.first<<" "<<rc.second<<":"<< j.second<<endl;
        }
        else{
            if (fabs(sJ2map[j.first]-j.second)>1e-1 ){
                auto rc=separe(j.first);
                cerr<<"diff in="<<rc.first<<" "<<rc.second<<" : jac="<<sJ2map[j.first]<<" calderv="<<j.second<<endl;
            }
        }
    }
cout<<endl;
    //    cout<<".................A"<<endl;
//    for (int k=0; k<sJ.outerSize(); ++k)
//      for ( Eigen::SparseMatrix<float> ::InnerIterator it(sJ,k); it; ++it)
//          if (it.row()==0||it.row()==1)
//              cout<<it.row()<<" "<<it.col()<<":"<<it.value()<<" ";
//    cout<<endl;


//    jacobian(sol,J2);
//    cout<<".................B"<<endl;
//    for (int k=0; k<J2.outerSize(); ++k)
//      for ( Eigen::SparseMatrix<float> ::InnerIterator it(J2,k); it; ++it)
//          if (it.row()==0||it.row()==1)
//              cout<<it.row()<<" "<<it.col()<<":"<<it.value()<<" ";
//    cout<<endl;
cout<<endl;
 }

void GlobalOptimizerSimple::getResults(Slam &w)throw(std::exception){
    for(const auto &p:pmap_info){
        if (w.TheMap.map_points.is(p.first))
            w.TheMap.map_points[p.first].setCoordinates(cv::Point3f (_solution(p.second.sol_idx),_solution(p.second.sol_idx+1),_solution(p.second.sol_idx+2)));
    }

    for(auto &f:_params.used_frames){
        if (w.TheFrameSet.is(f)){
            w.TheFrameSet[f].pose_f2g= getFramePose( _solution,f);
        }
    }
    //for the markers used (update their location)
    for(const auto &m:markermap_info){
        se3 pose_g2m;
        for(int i=0;i<6;i++) pose_g2m[i]=_solution(m.second.sol_idx+i);
   //     _debug_msg_(" marker "<<m.first<<" "<<w.TheMap.map_markers[ m.first].pose_g2m<<" "<<pose_g2m);
        w.TheMap.map_markers[ m.first].pose_g2m=pose_g2m;

    }
}

se3 GlobalOptimizerSimple::getFramePose( const SparseLevMarq<float>::eVector &sol,uint32_t frame_idx){
    if (_params.fixed_frames.count(frame_idx))
        return fixedframe_poses[frame_idx];
    se3 pose;
    auto s_idx=frame_solpos[frame_idx];
    memcpy(&pose,sol.data()+s_idx,6*sizeof(float) );
    return pose;
}



void GlobalOptimizerSimple::saveToStream_impl(std::ostream &str){

}
void GlobalOptimizerSimple::readFromStream_impl(std::istream &str){


}
}
