#include "sparselevmarq.h"
#include "globaloptimizer.h"
#include <omp.h>
namespace ucoslam{

//creates the solution vector from the input data
//solution vector is
//fx fy cx cy k1 k2 p1 p2 k3 | marker_0(rx ry rz tx ty tz) marker_1 .. | x0 y0 z0 ... xn yn zn |  frame_0(rx ry rz tx ty tz) ....
void GlobalOptimizer::convert(  Map &map,  FrameSet &fset, ImageParams &ip, GlobalOptimizer::Params &p,SparseLevMarq<double>::eVector &sol)throw (std::runtime_error){


    //fill the optimizing_frames set if empty
    if (p.optimizing_frames.size()==0)
        for(auto f:fset) p.optimizing_frames.insert(f.first);
    else{//or... check that all the optimizing frames selected are in the map
        if (p.optimizing_frames.size()!=0)
            for(auto f:p.optimizing_frames)
                if (!fset.is(f)) throw std::runtime_error("GlobalOptimizer_convert Frame not found");
    }

    cout<<"frames:";
    for(auto f:p.optimizing_frames)cout<<f<<" ";cout<<endl;

    p.n_frames=p.optimizing_frames.size();


    //---------------------------------------
    //create the set of markers to be optimized
    std::set<uint32_t> opt_markers;
    for(auto f:p.optimizing_frames)
        for(auto m:fset[f].markers){
            if (map.map_markers.find(m.id)!=map.map_markers.end())
                opt_markers.insert(m.id);
        }

    //now, set the position of each one in the solution vector
    uint32_t sol_idx=9;
    p.so_markers=sol_idx; //start of markers
    for(auto m:opt_markers){
        p.opt_markers.insert(make_pair(m, sol_idx));
        cout<<"mp:"<<m<<" "<< sol_idx<<endl;
        sol_idx+=6;
    }

    //---------------------------------------
    //go with the points to be optimized
    p.so_points=sol_idx;//start of points index in solution vector
    for(const auto &pt:map.map_points){
            p.opt_points.insert(make_pair(pt.first,sol_idx));
            sol_idx+=3;
        }
    cout<<"points="<<map.size()<<endl;

    //------------------------------------------
    //The frames
    p.so_frames=sol_idx;
    for(auto f:p.optimizing_frames){
        p.opt_frames.insert(make_pair(f,sol_idx));
        cout<<"f:"<<f<<" "<<sol_idx<<endl;
        sol_idx+=6;
    }
    //used to compute the weight assigned to markers and points.
    int nPointErrorMeasures=0;
    int nMarkersErrorMeasures=0;
    //----------- fill frame_points_sol_proj
    uint32_t err_idx=0;
    for(auto f:p.opt_frames){
        auto psp_points=make_pair(std::vector<uint32_t>(),std::vector<cv::Point2f>());
        Frame &frame=fset[f.first];
        p.frame_starterr[f.first]=err_idx;
        //for each point the frame sees, the location of the 3d point in the solution vector and its 2d projection
        for(size_t i=0;i<frame.ids.size();i++){
            if (frame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                assert(p.opt_points.find(frame.ids[i])!=p.opt_points.end());

                psp_points.first.push_back(p.opt_points[frame.ids[i]]);
                psp_points.second.push_back(frame.kpts[i].pt);
                err_idx+=2;//each point adds two error measures
                nPointErrorMeasures+=2;
            }
        }
        p.frame_points_sol_proj[f.first]=psp_points;

        //now, the markers
        auto psp_markers=make_pair(std::vector<uint32_t>(),std::vector<cv::Point2f>());
         //for each marker the frame sees, the location of the 3d point in the solution vector and its 2d projection
        for(auto marker : frame.markers ){
            if ( map.map_markers.find(marker.id)!=map.map_markers.end()){
                assert(p.opt_markers.find(marker.id)!=p.opt_markers.end());
                psp_markers.first.push_back(p.opt_markers[marker.id]);
                psp_markers.second.insert(psp_markers.second.end(),marker.begin(),marker.end() );
                p.frame_marker_errorstart[f.first][marker.id]=err_idx;//CHECK!!!
//                cout<<"ffss:"<<f.first<<" "<<marker.id<<" "<<err_idx<<endl;
                err_idx+=8;//each marker adds eight error measures (2 each corner)
                p.marker_frames[marker.id].push_back(f.first);//
                nMarkersErrorMeasures+=8;
            }
        }
         p.frame_makers_sol_proj[f.first]=psp_markers;
    }

    p.errv_size=err_idx;
    //---------------------
    //move data to solution vector
    assert(ip.Distorsion.total()==5);
    assert(ip.Distorsion.type()==CV_32F);
    assert(ip.CameraMatrix.type()==CV_32F);
    //first camera params
    sol.resize(sol_idx);
    //check all elements are written
    for(int i=0;i<sol.size();i++) sol(i)=std::numeric_limits<double>::quiet_NaN();

    sol(0)= ip.CameraMatrix.at<float>(0,0);//fx
    sol(1)= ip.CameraMatrix.at<float>(1,1);//fy
    sol(2)= ip.CameraMatrix.at<float>(0,2);//cx
    sol(3)= ip.CameraMatrix.at<float>(1,2);//cy
    for(int i=0;i<5;i++) sol(4+i)= ip.Distorsion.ptr<float>(0)[i];

//    //now  markers
    for(auto m:p.opt_markers){
        assert(map.map_markers.find(m.first)!=map.map_markers.end());
        se3 pose=map.map_markers[m.first].pose_g2m;
        cout<<"mm:"<<m.first<<" "<<m.second<<endl;
        for(int i=0;i<6;i++) sol(m.second++)=pose[i];
    }
    //now, go with the points
    for(auto pid:p.opt_points){
        assert(map.map_points.find(pid.first)!=map.map_points.end());
        se3 pose=map.map_points[pid.first].pose;
       // cout<<"pp:"<<pid.first<<" "<<pid.second<<endl;
        for(int i=3;i<6;i++)  sol(pid.second++)=pose[i];//copy only the t element (r is the direction and is not optimized here)
    }
    //go with frames
    for(auto f:p.opt_frames){
        assert(fset.find(f.first)!=fset.end());
        se3 pose=fset[f.first].pose_f2g;
        cout<<"ff:"<<f.first<<" "<<f.second<<endl;
        for(int i=0;i<6;i++) sol(f.second++)=pose[i];
    }
//    cerr<<"sol("<<sol.size()<<"):";
//    for(int i=0;i<sol.size();i++){cerr<<sol(i)<<" ";
//    }cerr<<endl;

    p._w_m =p._w_p=0;
    if ( nMarkersErrorMeasures!=0)
        p._w_m = p.w_markers /  double(nMarkersErrorMeasures) ;

    if(nPointErrorMeasures!=0)
    p._w_p = (1.d-p.w_markers)/double(nPointErrorMeasures);
    //normalize the sum
    double sum=p._w_m+p._w_p;
      p._w_m/=sum;
    p._w_p/=sum;

    cout<<"wmarker="<<p._w_m<<" wpoints="<<p._w_p<<endl;

//    cin.ignore();
//    for(int i=0;i<sol.size();i++){
//        assert( !std::isnan(sol(i)));
//    }
}

//extracs data to map and fset from solution vector
void GlobalOptimizer::convert(const SparseLevMarq<double>::eVector &sol, Map &map,FrameSet &fset,ImageParams &ip, const GlobalOptimizer::Params &p){


    //---------------------
    //move data to solution vector
    ip.Distorsion.create(1,5,CV_32F);
    ip.CameraMatrix=cv::Mat::eye(3,3,CV_32F);
    //first camera params
    const double *sptr=sol.data();
    ip.CameraMatrix.at<float>(0,0)=*sptr++;//fx
    ip.CameraMatrix.at<float>(1,1)=*sptr++;//fy
    ip.CameraMatrix.at<float>(0,2)=*sptr++;//cx
    ip.CameraMatrix.at<float>(1,2)=*sptr++;//cy
    for(int i=0;i<5;i++)  ip.Distorsion.ptr<float>(0)[i]=*sptr++;

    //now  markers
    for(auto m:p.opt_markers)
        map.map_markers[m.first].pose_g2m=se3 (sol(m.second),sol(m.second+1),sol(m.second+2),sol(m.second+3),sol(m.second+4),sol(m.second+5));
    //now, go with the points
    for(auto pid:p.opt_points)
        for(int i=0;i<3;i++) map.map_points[pid.first].pose[i+3]=sol(pid.second+i);
    //go with frames
    for(auto f:p.opt_frames){cout<<sol(f.second+3)<<" "<<sol(f.second+4)<<" "<<sol(f.second+5)<<endl;
        fset[f.first].pose_f2g=se3 (sol(f.second),sol(f.second+1),sol(f.second+2),sol(f.second+3),sol(f.second+4),sol(f.second+5));
}
}

void GlobalOptimizer::optimize(Map &map, FrameSet &fset, ImageParams &ip, float markerSize, const Params &p)throw(std::exception){
    _params=p;
    //_ip=ip;
    _markerSize=markerSize;
    _fset=&fset;
    SparseLevMarq<double>::eVector solution;

    convert(map,fset,ip,_params,solution);

    SparseLevMarq<double> optimizer;
    SparseLevMarq<double>::Params oparams(p.nIters,0.1,1,1e-5);
    oparams.cal_dev_parallel=false;
    oparams.verbose=_params.verbose;
     optimizer.setParams(oparams);
    //optimizer.solve( solution,std::bind(&GlobalOptimizer::error,this,std::placeholders::_1,std::placeholders::_2), std::bind(&GlobalOptimizer::jacobian,this,std::placeholders::_1,std::placeholders::_2) );
     optimizer.solve( solution,std::bind(&GlobalOptimizer::error,this,std::placeholders::_1,std::placeholders::_2));
    convert(solution,map,fset,ip,_params);


}




void GlobalOptimizer::error(const SparseLevMarq<double>::eVector &sol,SparseLevMarq<double>::eVector &err){


    err.resize(_params.errv_size);
    //extract the camera params
    cv::Mat cameraMatrix=cv::Mat::eye(3,3,CV_32F);
    cv::Mat distorsion(1,5,CV_32F);
    const double *sptr=sol.data();
    cameraMatrix.at<float>(0,0)=*sptr++;//fx
    cameraMatrix.at<float>(1,1)=*sptr++;//fy
    cameraMatrix.at<float>(0,2)=*sptr++;//cx
    cameraMatrix.at<float>(1,2)=*sptr++;//cy
    for(int i=0;i<5;i++)  distorsion.ptr<float>(0)[i]=*sptr++;


    std::vector<std::vector< Eigen::Triplet<double> > >omp_triplets(omp_get_max_threads());


    vector<cv::Point3f> marker_points = { cv::Point3f ( -_markerSize/2., _markerSize/2.,0 ),cv::Point3f ( _markerSize/2., _markerSize /2.,0 ),
                                   cv::Point3f ( _markerSize/2., -_markerSize/2.,0 ),cv::Point3f ( -_markerSize/2., -_markerSize/2.,0 )  };

    //for each frame, project the corresponding points and markers
    //#pragma omp parallel for
    for(auto frame:_params.opt_frames){

      //  cout<<"###################frame:"<<frame.first<<" np:"<< _params.frame_points_sol_proj[frame.first].first.size()<<" nm:"<< _params.frame_makers_sol_proj[frame.first].first.size()<<endl;
          vector<cv::Point3f> p3d;
        p3d.reserve(_params.frame_points_sol_proj[frame.first].first.size());
        cv::Mat jac;
        vector<cv::Point2f> p2d_reprj;
        se3 frame_pose;
        auto & omptvector= omp_triplets[omp_get_thread_num()];
        uint32_t err_idx=_params.frame_starterr[frame.first];


        //extract frame pose
        for(int i=0;i<6;i++) frame_pose[i]=sol(frame.second+i);

        if (_params.opt_points.size()>0 && _params.frame_points_sol_proj[frame.first].first.size()>0){
            //-----------------------------------------------------------
            //---------- 3D points
            //-----------------------------------------------------------
            //extract the 3d points from solution vector
            for(auto p3d_pos:_params.frame_points_sol_proj[frame.first].first)
                p3d.push_back(cv::Point3f( sol(p3d_pos),sol(p3d_pos+1),sol(p3d_pos+2)));

            //project the points
             cv::projectPoints(p3d,frame_pose.getRvec(),frame_pose.getTvec(),cameraMatrix,distorsion,p2d_reprj,jac);
             assert(jac.type()==CV_64F);
            //now, fill in the error vector
            const auto &p2d= _params.frame_points_sol_proj[frame.first].second;
            for(size_t p=0;p<p2d.size();p++){
                err(err_idx++)=_params._w_p*(p2d_reprj[p].x-p2d[p].x);
                err(err_idx++)=_params._w_p*(p2d_reprj[p].y-p2d[p].y);
            }
            // ------------ jacobian
            //jacobian of camera params and frame positions
            {
                uint32_t err_idx_jac=_params.frame_starterr[frame.first];
                for(size_t p=0;p<p2d.size()*2;p++,err_idx_jac++){
                    if (!_params.isFrameFixed(frame.first))
                        for(int rt=0;rt<6;rt++){
                            omptvector.push_back( Eigen::Triplet<double>(err_idx_jac,frame.second+rt,_params._w_p*jac.ptr<double>(p)[rt]) );
                        }
                    if (!_params.fixIntrinsics)
                        for(size_t cp=0;cp<9;cp++){
                            omptvector.push_back( Eigen::Triplet<double>(err_idx_jac,cp,_params._w_p*jac.ptr<double>(p)[6+cp]) );
                        }
                }

            }
            //--jacobian of 3d points
            {
                uint32_t err_idx_p3d=_params.frame_starterr[frame.first];
                double ux,uy,uz,vx,vy,vz;//partial derivatives of the 3d point
                auto &points_col=_params.frame_points_sol_proj[frame.first].first;
                for(size_t p=0;p<p2d.size();p++){
                    point_part_derv(cameraMatrix,distorsion,frame_pose,p3d[p],ux,uy,uz,vx,vy,vz);
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p],_params._w_p*ux) );
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p]+1,_params._w_p*uy) );
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p]+2,_params._w_p*uz) );
                    err_idx_p3d++;
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p],_params._w_p*vx) );
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p]+1,_params._w_p*vy) );
                    omptvector.push_back( Eigen::Triplet<double>(err_idx_p3d,points_col[p]+2,_params._w_p*vz) );
                    err_idx_p3d++;
                }

            }
        }

        //-----------------------------------------
        //                  MARKERS
        //-----------------------------------------
        if(_params.opt_markers.size()>0 && _params.frame_makers_sol_proj[frame.first].first.size()>0){
            vector<cv::Point3f> marker_points_all;marker_points_all.reserve(_params.frame_makers_sol_proj[frame.first].first.size()*4);
            for(auto marker_pos:_params.frame_makers_sol_proj[frame.first].first){
                se3 pose_g2m( sol(marker_pos),sol(marker_pos+1),sol(marker_pos+2),sol(marker_pos+3),sol(marker_pos+4),sol(marker_pos+5));
                //   cout<<"pose="<<pose_g2m<<endl;
                //multiply the 3d points to be in global coordinates and add to the end of vector
                mult(pose_g2m,marker_points,marker_points_all);
            }
            auto err_idx_start_markers=err_idx;//start of errors for markers
            //reprjection of the 3d points
            cv::Mat jac;
             cv::projectPoints(marker_points_all,frame_pose.getRvec(),frame_pose.getTvec(),cameraMatrix,distorsion,p2d_reprj,jac);
             assert(jac.type()==CV_64F);
            //add errors
            auto & marker_prj=_params.frame_makers_sol_proj[frame.first].second;
             assert(marker_prj.size()==p2d_reprj.size());
            for(size_t p=0;p<p2d_reprj.size();p++){
                err(err_idx++)= _params._w_m*( p2d_reprj[p].x-marker_prj[p].x);
                err(err_idx++)= _params._w_m*(p2d_reprj[p].y-marker_prj[p].y);
                // cout<<marker_points_all[p]<<" "<< p2d_reprj[p]<<" "<<marker_prj[p]<<endl;
            }
            {
                auto row_err=err_idx_start_markers;
                for(size_t p=0;p<p2d_reprj.size()*2;p++,row_err++){
                    if (!_params.isFrameFixed(frame.first))
                        for(int rt=0;rt<6;rt++)
                            omptvector.push_back( Eigen::Triplet<double>(row_err,frame.second+rt,_params._w_m*jac.ptr<double>(p)[rt]) );
                    if (!_params.fixIntrinsics)
                        for(size_t cp=0;cp<9;cp++)
                            omptvector.push_back( Eigen::Triplet<double>(row_err,cp,_params._w_m*jac.ptr<double>(p)[6+cp]) );
                }
            }

            //now, need to compute the jacobian of markers, we will do it in the jacobian function for effiency reasons
        }


    }
    join(omp_triplets,triplets,true);

//    for(auto t:triplets)
//        cout<<t.row()<<" "<<t.col()<<" "<<t.value()<<endl;



}


void GlobalOptimizer::jacobian(const SparseLevMarq<double>::eVector &sol, Eigen::SparseMatrix<double> &Jac){
    //extract the camera params
    cv::Mat cameraMatrix=cv::Mat::eye(3,3,CV_32F);
    cv::Mat distorsion(1,5,CV_32F);
    const double *sptr=sol.data();
    cameraMatrix.at<float>(0,0)=*sptr++;//fx
    cameraMatrix.at<float>(1,1)=*sptr++;//fy
    cameraMatrix.at<float>(0,2)=*sptr++;//cx
    cameraMatrix.at<float>(1,2)=*sptr++;//cy
    for(int i=0;i<5;i++)  distorsion.ptr<float>(0)[i]=*sptr++;

    if (_params.opt_frames.size()!=0){
        //now, we only need to compute the jacobian of the markers
        vector<cv::Point3f> marker_points = { cv::Point3f ( -_markerSize/2., _markerSize/2.,0 ),cv::Point3f ( _markerSize/2., _markerSize /2.,0 ),
                                              cv::Point3f ( _markerSize/2., -_markerSize/2.,0 ),cv::Point3f ( -_markerSize/2., -_markerSize/2.,0 )  };

        std::vector<std::vector< Eigen::Triplet<double> > >omp_triplets(omp_get_max_threads());

        //get all frame poses
        std::map<uint32_t,se3> frame_poses;
        for(auto frame:_params.opt_frames)
            frame_poses[frame.first]=se3( sol(frame.second) ,sol(frame.second+1),sol(frame.second+2),sol(frame.second+3),sol(frame.second+4),sol(frame.second+5));

        double der_epsilon=1e-3;
        //for each marker, compute derivative in each axis in all visible frames
        //#pragma omp parallel for
        for(auto marker:_params.opt_markers){

            auto & omptvector= omp_triplets[omp_get_thread_num()];

            for(int rt=0;rt<6;rt++){
                //get the altered vector adding and substracting epison
                SparseLevMarq<double>::eVector rt_p=sol.middleRows(marker.second,6);
                rt_p(rt)+=der_epsilon;
                SparseLevMarq<double>::eVector rt_n=sol.middleRows(marker.second,6);
                rt_n(rt)-=der_epsilon;

                vector<cv::Point3f> obj_points_p_n;
                mult(se3(rt_p(0),rt_p(1),rt_p(2),rt_p(3),rt_p(4),rt_p(5)),marker_points,obj_points_p_n);
                mult(se3(rt_n(0),rt_n(1),rt_n(2),rt_n(3),rt_n(4),rt_n(5)),marker_points,obj_points_p_n);

                for( auto mf:_params.marker_frames[marker.first]){
                    //get the frame location and project
                    vector<cv::Point2f> img_points_p_n;
                    cv::projectPoints(obj_points_p_n,frame_poses[mf].getRvec() ,frame_poses[mf].getTvec(),cameraMatrix,distorsion,img_points_p_n);

                    //makes the difference to compute the derivate
                    auto errorRow=  _params.frame_marker_errorstart[mf][marker.first];
                    auto error_Col=marker.second+rt;
                    for(int p=0;p<4;p++){
                        double derv_errx=(img_points_p_n[p].x-img_points_p_n[p+4].x)/(2.*der_epsilon);
                        omptvector.push_back(Eigen::Triplet<double>( errorRow++,error_Col,_params._w_m*derv_errx));
                         double derv_erry=(img_points_p_n[p].y-img_points_p_n[p+4].y)/(2.*der_epsilon);
                        omptvector.push_back(Eigen::Triplet<double>( errorRow++,error_Col,_params._w_m*derv_erry));

                    }

                }

            }

        }

        join(omp_triplets,triplets);
    }

    Jac.setFromTriplets(triplets.begin(),triplets.end());
}


void GlobalOptimizer::mult(se3 pose_g2m,const vector<cv::Point3f> &vpoints_in,vector<cv::Point3f> &vpoints_out){
    cv::Mat rt44=pose_g2m;
    assert(rt44.type()==CV_32F);
    float *m=rt44.ptr<float>(0);
    for(const auto &p:vpoints_in){
        cv::Point3f pres;
        pres.x=p.x*m[0]+p.y*m[1]+p.z*m[2]+m[3];
        pres.y=p.x*m[4]+p.y*m[5]+p.z*m[6]+m[7];
        pres.z=p.x*m[8]+p.y*m[9]+p.z*m[10]+m[11];
        vpoints_out.push_back(pres);
    }
}


void GlobalOptimizer::point_part_derv(const cv::Mat &CameraMatrix,const cv::Mat &Distorsion,se3 &pose,cv::Point3f p3d,double &ux,double &uy,double &uz,double &vx,double &vy,double &vz){
    cv::Mat rt44 =pose;
    float *m=rt44.ptr<float>(0);
    double fx= CameraMatrix.at<float>(0,0);
    double fy= CameraMatrix.at<float>(1,1);
    double k1= Distorsion.ptr<float>(0)[0];
    double k2= Distorsion.ptr<float>(0)[1];
    double p1= Distorsion.ptr<float>(0)[2];
    double p2= Distorsion.ptr<float>(0)[3];
    double k3= Distorsion.ptr<float>(0)[4];
    double x= m[0]*p3d.x + m[1]*p3d.y+ m[2]*p3d.z +m[3];
    double y= m[4]*p3d.x + m[5]*p3d.y+ m[6]*p3d.z +m[7];
    double z= m[8]*p3d.x + m[9]*p3d.y+ m[10]*p3d.z +m[11];
    double x2=x*x;
    double y2=y*y;
    double z2=z*z;
    double A= (x2/z2) + (y2/z2);
    double R2=x2+y2;
    double U1=A * (2.*k2 + 3.* A*k3) ;
    double U2=(1 + A*(k1 + A *(k2 + A*k3)));
    double U3=A*(2.*k2 + 3.* A*k3);
    double U4=-2.* R2* (k1 + U1)*x;
    double U5=-2.* R2* (k1 + U1)*y;
    double yp1=y*p1;
    double xp1=x*p1;
    double xp2=x*p2;
    double yp2=y*p2;
    double y2p2=y2*p2;
    double xy=x*y;
    double z4=z2*z2;
    double z3=z*z2;

    ux= (fx/(z4))*
            (m[8]*U4 + 2.*((k1*m[0] - 3.* m[8]* p2 + m[0]*U1)*x2
            +(k1*m[4] + U3*m[4] - 2.*m[8]*p1)*xy - m[8]*y2p2)* z
            -(m[8]*U2*x - 2.*(m[4]*xp1 + 3*m[0]*xp2 + m[0]*yp1 + m[4]*yp2)) *z2 + m[0]*U2*z3);


    uy= (fx/(z4))*
            (m[9]*U4 + 2.*((k1*m[1] - 3.* m[9]* p2 + m[1]*U1)*x2 +
            (k1*m[5] + U3*m[5] - 2.*m[9]*p1)*xy - m[9]*y2p2)* z -
            (m[9]*U2*x - 2.*(m[5]*xp1 + 3*m[1]*xp2 + m[1]*yp1 + m[5]*yp2)) *z2 + m[1]*U2*z3);

    uz= (fx/(z4))*
            (m[10]*U4 + 2.*((k1*m[2] - 3.* m[10]* p2 + m[2]*U1)*x2 +
            (k1*m[6] + U3*m[6] - 2.*m[10]*p1)*xy - m[10]*y2p2)* z -
            (m[10]*U2*x - 2.*(m[6]*xp1 + 3*m[2]*xp2 + m[2]*yp1 + m[6]*yp2)) *z2 + m[2]*U2*z3);

    vx= (fy/z4)*
            ( m[8] *U5+ 2 *((k1 + U1)*y*(m[0]*x + m[4]*y) - m[8] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
            (m[8]*U2*y - 2.* (m[0]*xp1 + m[4]*xp2 + 3.* m[4]* yp1 + m[0]*yp2))*z2 + m[4]*U2*z3);

    vy= (fy/z4)*
            ( m[9] *U5 + 2 *((k1 + U1)*y*(m[1]*x + m[5]*y) - m[9] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
            (m[9]*U2*y - 2.* (m[1]*xp1 + m[5]*xp2 + 3.* m[5]* yp1 + m[1]*yp2))*z2 + m[5]*U2*z3);

    vz= (fy/z4)*
            (m[10]* U5 + 2 *((k1 + U1)*y*(m[2]*x + m[6]*y) - m[10] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
            (m[10]*U2*y - 2.* (m[2]*xp1 + m[6]*xp2 + 3.* m[6]* yp1 + m[2]*yp2))*z2 + m[6]*U2*z3);

}

void GlobalOptimizer::join(const std::vector<std::vector< Eigen::Triplet<double> > > &in, std::vector< Eigen::Triplet<double> > &out,bool clearOutput){
    int s=0;
    if (clearOutput)out.clear();
    for(const auto &e:in)s+=e.size();
    int idx=out.size();
    out.resize(out.size()+s);
    for(const auto &e:in) {
        memcpy(&out[idx],&e[0],e.size()*sizeof(Eigen::Triplet<double>));
        idx+=e.size();
    }
}

}
