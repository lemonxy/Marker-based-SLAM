#include "globaloptimizer_g2o.h"
#include "stuff/utils.h"
#include "stuff/timers.h"
#include <chrono>
#include "basic_types/marker.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/base_binary_edge.h"

#include <iostream>

#include "proj_info.h"
//#define ORIGINAL_G2O_SBA

namespace ucoslam{
typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>                               Vector8D;

class  MarkerEdge: public  g2o::BaseBinaryEdge<8, Vector8D, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
    g2o::Vector3D points[4];

    uint32_t marker_id,frame_id;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double fx, fy, cx, cy;

  MarkerEdge(double size,uint32_t markerid,uint32_t frameid){
      marker_id=frameid;
      frame_id=frameid;
      _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
      auto pointsA=ucoslam::Marker::get3DPointsLocalRefSystem(size);
      for(int i=0;i<4;i++){
          auto &p=pointsA[i];
          points[i]=g2o::Vector3D(p.x,p.y,p.z);
      }
  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}

  inline void computeError()  {
      //marker
      const g2o::VertexSE3Expmap* g2m= static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);//marker
    //camera
      const g2o::VertexSE3Expmap*  c2g = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);//camera

      //std::cout<<" ------------------------"<<std::endl;
    //first, move the points to the global reference system, and then to the camera
    auto Transform_C2M=c2g->estimate()*  g2m->estimate();
    //std::cout<<"g2m="<<g2m->estimate()<<" c2g="<<c2g->estimate()<< " Transform_C2M="<<Transform_C2M<<std::endl;
    g2o::Vector3D points2[4];
    for(int i=0;i<4;i++){
        points2[i]=Transform_C2M.map(points[i]);//3d rigid transform
  //      std::cout<<"p="<<points2[i]<<std::endl;
    }


    //now, project
     _error.resize(8);
    Vector8D obs(_measurement);
//    std::cout<<"obs="<<obs<<std::endl;
    assert(_error.size()==obs.size());
    int idx=0;
    ///测量-预测
    for(int i=0;i<4;i++){
        float projx=( points2[i](0)/points2[i](2)) *fx +cx;
        _error(idx)=obs(idx)-projx;
         idx++;
        float projy=( points2[i](1)/points2[i](2)) *fy +cy;
        _error(idx)=obs(idx)-projy;
         idx++;
    }
//     std::cout<<"E(="<<marker_id<<"-"<< frame_id<<"):"<<_error.transpose()<<std::endl;
  }

  virtual void  linearizeOplus()
  {
      g2o::VertexSE3Expmap *mTcw = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
      g2o::VertexSE3Expmap *mTwm = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
      auto Tcw = mTcw->estimate();
      auto Twm = mTwm->estimate();
      Eigen::Matrix3d Rcw = Tcw.rotation().toRotationMatrix();
      Eigen::Vector3d tcw = Tcw.translation();
      g2o::SE3Quat Tcm(mTcw->estimate() * mTwm->estimate());

      for (int i = 0; i < 4; ++i)
      {
          Eigen::Vector3d cam_xyz = Tcm.map(points[i]);
          double x = cam_xyz(0);
          double y = cam_xyz(1);
          double z = cam_xyz(2);
          double z_2 = z * z;
          Eigen::Matrix<double,2,3> block1;///公共矩阵块
          block1(0,0) = -fx/z; block1(0,1) = 0; block1(0,2) = fx*x/z_2;
          block1(1,0) = 0; block1(1,1) = -fy/z; block1(1,2) = fy*y/z_2;

          Eigen::Vector3d xyz = Twm.map(points[i]);
          Eigen::Matrix3d skew_xyz;
          skew_xyz(0,0) = 0; skew_xyz(0,1) = -xyz(2); skew_xyz(0,2) = xyz(1);
          skew_xyz(1,0) = xyz(2); skew_xyz(1,1) = 0; skew_xyz(1,2) = -xyz(0);
          skew_xyz(2,0) = -xyz(1); skew_xyz(2,1) = xyz(0); skew_xyz(2,2) = 0;

          Eigen::Matrix<double,3,3> block2;///对Twm求导的矩阵块
          block2 = -Rcw*skew_xyz;

          ///旋转在前，平移在后
          Eigen::Matrix<double,2,6> tmp1;
          tmp1.block<2,3>(0,0) = block1 * block2;
          tmp1.block<2,3>(0,3) = block1 * Rcw;
          Eigen::Matrix<double,3,3> block3;///对Tcw求导的矩阵块
          block3(0,0) = 0; block3(0,1) = -cam_xyz(2); block3(0,2) = cam_xyz(1);
          block3(1,0) = cam_xyz(2); block3(1,1) = 0; block3(1,2) = -cam_xyz(0);
          block3(2,0) = -cam_xyz(1); block3(2,1) = cam_xyz(0); block3(2,2) = 0;
          block3 = -1 * block3;

          ///旋转在前，平移在后
          Eigen::Matrix<double,2,6> tmp2;
          tmp2.block<2,3>(0,0) = block1 * block3;
          tmp2.block<2,3>(0,3) = block1;

          ///对Twm求导
          _jacobianOplusXi(i*2,0) = tmp1(0,0);
          _jacobianOplusXi(i*2,1) = tmp1(0,1);
          _jacobianOplusXi(i*2,2) = tmp1(0,2);
          _jacobianOplusXi(i*2,3) = tmp1(0,3);
          _jacobianOplusXi(i*2,4) = tmp1(0,4);
          _jacobianOplusXi(i*2,5) = tmp1(0,5);

          _jacobianOplusXi(i*2+1,0) = tmp1(1,0);
          _jacobianOplusXi(i*2+1,1) = tmp1(1,1);
          _jacobianOplusXi(i*2+1,2) = tmp1(1,2);
          _jacobianOplusXi(i*2+1,3) = tmp1(1,3);
          _jacobianOplusXi(i*2+1,4) = tmp1(1,4);
          _jacobianOplusXi(i*2+1,5) = tmp1(1,5);

          ///对Tcw求导
          _jacobianOplusXj(i*2,0) = tmp2(0,0);
          _jacobianOplusXj(i*2,1) = tmp2(0,1);
          _jacobianOplusXj(i*2,2) = tmp2(0,2);
          _jacobianOplusXj(i*2,3) = tmp2(0,3);
          _jacobianOplusXj(i*2,4) = tmp2(0,4);
          _jacobianOplusXj(i*2,5) = tmp2(0,5);

          _jacobianOplusXj(i*2+1,0) = tmp2(1,0);
          _jacobianOplusXj(i*2+1,1) = tmp2(1,1);
          _jacobianOplusXj(i*2+1,2) = tmp2(1,2);
          _jacobianOplusXj(i*2+1,3) = tmp2(1,3);
          _jacobianOplusXj(i*2+1,4) = tmp2(1,4);
          _jacobianOplusXj(i*2+1,5) = tmp2(1,5);
      }
  }

};

class  EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ(){}

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}

  inline void computeError()  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }


  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void  linearizeOplus() {

      g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    Eigen::Matrix<double,2,3> tmp;
    tmp(0,0) = fx;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*fx;

    tmp(1,0) = 0;
    tmp(1,1) = fy;
    tmp(1,2) = -y/z*fy;

    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
    _jacobianOplusXj(0,2) = y/z *fx;
    _jacobianOplusXj(0,3) = -1./z *fx;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2 *fx;

    _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
    _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
    _jacobianOplusXj(1,2) = -x/z *fy;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *fy;
    _jacobianOplusXj(1,5) = y/z_2 *fy;

//    cout<<_jacobianOplusXi<<endl;
//    cout<<endl;
//    cout<<_jacobianOplusXj<<endl;

//    auto toCvMat=[](const g2o::SE3Quat &SE3)
//    {
//        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
//        cv::Mat cvMat(4,4,CV_32F);
//        for(int i=0;i<4;i++)
//            for(int j=0; j<4; j++)
//                cvMat.at<float>(i,j)=eigMat(i,j);

//        return cvMat.clone();

//    };

//    cv::Mat mpose=toCvMat(T);
//    ucoslam::se3 rt=mpose;
//    cv::Mat camp=cv::Mat::eye(3,3,CV_32F);
//    camp.at<float>(0,0)=fx;
//    camp.at<float>(1,1)=fy;
//    camp.at<float>(0,2)=cx;
//    camp.at<float>(1,2)=cy;
//    proj_info  pi=project_and_derive(cv::Point3f(xyz(0),xyz(1),xyz(2)), rt,camp,true) ;
//    cout<<"ehere"<<endl;

  }



  inline Eigen::Vector2d  cam_project(const Eigen::Vector3d & trans_xyz) const{
    return Eigen::Vector2d  ( (trans_xyz(0)/trans_xyz(2)) *fx + cx, (trans_xyz(1)/trans_xyz(2))*fy + cy);
  }
  double fx, fy, cx, cy;

};


class  EdgeStereoSE3ProjectXYZ: public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZ(){}


  inline void computeError()  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
  }

 inline  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


 virtual void linearizeOplus(){
     g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
     g2o::SE3Quat T(vj->estimate());
     g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
     Eigen::Vector3d xyz = vi->estimate();
     Eigen::Vector3d xyz_trans = T.map(xyz);

     const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

     double x = xyz_trans[0];
     double y = xyz_trans[1];
     double z = xyz_trans[2];
     double z_2 = z*z;

     _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
     _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
     _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

     _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
     _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
     _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

     _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
     _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
     _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

     _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
     _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
     _jacobianOplusXj(0,2) = y/z *fx;
     _jacobianOplusXj(0,3) = -1./z *fx;
     _jacobianOplusXj(0,4) = 0;
     _jacobianOplusXj(0,5) = x/z_2 *fx;

     _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
     _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
     _jacobianOplusXj(1,2) = -x/z *fy;
     _jacobianOplusXj(1,3) = 0;
     _jacobianOplusXj(1,4) = -1./z *fy;
     _jacobianOplusXj(1,5) = y/z_2 *fy;

     _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
     _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
     _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
     _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
     _jacobianOplusXj(2,4) = 0;
     _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;}

 inline Eigen::Vector3d cam_project(const Eigen::Vector3d & trans_xyz, const float &bf) const{
     const float invz = 1.0f/trans_xyz[2];
     Eigen::Vector3d res;
     res[0] = trans_xyz[0]*invz*fx + cx;
     res[1] = trans_xyz[1]*invz*fy + cy;
     res[2] = res[0] - bf*invz;
     return res;
 }

  double fx, fy, cx, cy, bf;
};

vector<MarkerEdge*> g_vMarkerEdge;

void GlobalOptimizerG2O::setParams(std::shared_ptr<Map> map, const ParamSet &p )throw(std::exception){

    auto  toSE3Quat=[](const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    };


    g_vMarkerEdge.clear();

    _params=p;
    _InvScaleFactors.reserve(map->keyframes.front().scaleFactors.size());
    for(auto f:map->keyframes.front().scaleFactors) _InvScaleFactors.push_back(1./f);

    ///Step1：找局部关键帧组为有效关键帧
    //Find the frames and points that will be used
    isFixedFrame.resize(map->keyframes.capacity());
    usedFramesIdOpt.resize(map->keyframes.capacity());
    usedPointsIdOpt.resize(map->map_points.capacity());
    for(auto &v:usedFramesIdOpt)v=INVALID_IDX;
    for(auto &v:usedPointsIdOpt)v=INVALID_IDX;
    for(auto &v:isFixedFrame)v=UNFIXED;


    ///Step2: 找出有效关键帧（局部优化即相邻关键帧组），和固定关键帧，有效Marker
    uint32_t optiD=0;//optimizers vertex id
    if (_params.used_frames.size()==0){     //if non indicated, use all
        for(auto f:map->keyframes)
            usedFramesIdOpt[f.idx]=optiD++;
    }
    else {//else, set the required ones
        for(auto f:_params.used_frames)
            usedFramesIdOpt[f]=optiD++;
    }


    if(_params.fixFirstFrame ){
        if ( usedFramesIdOpt[map->keyframes.front().idx]!=INVALID_IDX )
            isFixedFrame[map->keyframes.front().idx]=FIXED_WITHPOINTS;
    }

    //add also the fixed ones (in case they are not in the used_frames map
    for(auto f:_params.fixed_frames) {
        if (usedFramesIdOpt[f]!=INVALID_IDX)//add it if not yet
            isFixedFrame[f]=FIXED_WITHPOINTS;
    }

    usedMapPoints.clear();

    //now, go thru points assigning ids
    for( uint32_t f=0;f<usedFramesIdOpt.size();f++){

        if ( usedFramesIdOpt[f]==INVALID_IDX)continue;
        if (  isFixedFrame[f]==FIXED_WITHOUTPOINTS)continue;

        for(auto point_id:map->keyframes[f].ids){
            if ( point_id==INVALID_IDX) continue;
            if (usedPointsIdOpt[point_id]==INVALID_IDX){
                if( (map->map_points[point_id].frames.size()<2 && !map->map_points[point_id].isStereo) || map->map_points[point_id].isBad)
                    usedPointsIdOpt[point_id]=INVALID_VISITED_IDX;
                else{
                    usedPointsIdOpt[point_id]=optiD++;
                    assert( std::find(usedMapPoints.begin(),usedMapPoints.end(),point_id)== usedMapPoints.end());
                    usedMapPoints.push_back(point_id);
                    for( const auto &f_info:map->map_points[point_id].frames)
                        if (usedFramesIdOpt[f_info.first]==INVALID_IDX){
                            usedFramesIdOpt[f_info.first]=optiD++;
                            isFixedFrame[f_info.first]=FIXED_WITHOUTPOINTS;
                        }
                }
            }
        }
        //now, the markers

        ///有效关键帧的地图Marker为有效Marker
        for(auto marker:map->keyframes[f].und_markers){
            if ( usedMarkersIdOp.count(marker.id)!=0 ) continue;//marker alreday added
            auto &map_marker=map->map_markers[marker.id];
            if (!map_marker.pose_g2m.isValid()) continue;//invalid pose yet
                usedMarkersIdOp[marker.id]=optiD++;//set a id for the optimizer
                //add all its frames if not yet
                for(auto frames_id:map_marker.frames)
                    if (usedFramesIdOpt[frames_id]==INVALID_IDX){
                        usedFramesIdOpt[frames_id]=optiD++;
                        isFixedFrame[frames_id]=FIXED_WITHOUTPOINTS;
                    }

        }

    }

    //finally, add
    _debug_msg_("Total points "<<map->map_points.size()<<" used= "<<usedMapPoints.size());
    _debug_msg_("Total markers "<<map->map_markers.size()<<" used= "<<usedMarkersIdOp.size());

    Optimizer=std::make_shared<g2o::SparseOptimizer>();
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    Optimizer->setAlgorithm(solver);

    bool bRobust=true;


uint32_t totalVars=0;


    ///////////////////////////////////////
    /// Add KeyFrames as vertices of the graph
    ///////////////////////////////////////

    ///Step3: 设置顶点
    ///Step3.1,设置帧的顶点：只有局部关键帧参与优化，第一帧需要固定不优化
    for( uint32_t fid=0;fid<usedFramesIdOpt.size();fid++){
        if (usedFramesIdOpt[fid]!=INVALID_IDX){
            g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(toSE3Quat(map->keyframes[fid].pose_f2g));
            vSE3->setId(usedFramesIdOpt.at(fid));

            if (isFixedFrame[fid]) {
                vSE3->setFixed(true);
            }

            totalVars+=6;
            Optimizer->addVertex(vSE3);
        }
    }



#ifdef ORIGINAL_G2O_SBA
    g2o::CameraParameters* camera = new g2o::CameraParameters( (fx+fy)*0.5, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    Optimizer->addParameter( camera );
#endif
    ///////////////////////////////////////
    /// ADD MAP POINTS AND LINKS TO KEYFRAMES
    ///////////////////////////////////////

    const float thHuber2D = sqrt(5.99);
    const float thHuberStereo = sqrt(7.815);

    point_edges_frameId.resize(map->map_points.capacity());

    ///Step3.2： 设置顶点，ORB特征点
    for(auto mp_id:usedMapPoints){
        MapPoint &mp=map->map_points[mp_id];
        float PointConfidence=mp.getConfidence();


        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        cv::Point3f p3d=mp.getCoordinates();
        Eigen::Matrix<double,3,1> vp3d;
        vp3d << p3d.x , p3d.y ,p3d.z;
        vPoint->setEstimate(vp3d);
        vPoint->setId(usedPointsIdOpt.at(mp_id));
        vPoint->setMarginalized(true);
        Optimizer->addVertex(vPoint);

        point_edges_frameId[mp_id].reserve(mp.frames.size());

        //SET EDGES
        int nEdges=0;
        for( const auto &f_info:mp.frames)
        {
            if ( usedFramesIdOpt.at(f_info.first)==INVALID_IDX) continue;
            nEdges++;
            const auto &Frame=map->keyframes[f_info.first];
            const cv::KeyPoint &kpUn =Frame.und_kpts[f_info.second];
            float depth=Frame.depth[f_info.second];


            if ( depth<=0){

                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;


                EdgeSE3ProjectXYZ* e = new  EdgeSE3ProjectXYZ();
                e->fx = Frame.imageParams.fx();
                e->fy = Frame.imageParams.fy();
                e->cx =  Frame.imageParams.cx();
                e->cy =  Frame.imageParams.cy();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedPointsIdOpt.at(mp_id))));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(f_info.first))));
                e->setMeasurement(obs);
                //try a different confidence based on the robustness (n times seen)



                e->setInformation(Eigen::Matrix2d::Identity()*_InvScaleFactors[kpUn.octave]*PointConfidence);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }
                Optimizer->addEdge(e);
                point_edges_frameId[mp_id].push_back(edge_frameId_stereo((void*)e,f_info.first,false));
                totalNEdges++;
            }
            else{//stereo observation
                Eigen::Matrix<double,3,1> obs;
                //compute the right proyection difference
                float mbf=Frame.imageParams.bl*Frame.imageParams.fx();
                const float kp_ur = kpUn.pt.x - mbf/depth;
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedPointsIdOpt.at(mp_id))));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(f_info.first))));
                e->setMeasurement(obs);
                e->setInformation(Eigen::Matrix3d::Identity()*double(_InvScaleFactors[kpUn.octave]*PointConfidence));


                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberStereo);

                e->fx = Frame.imageParams.fx();
                e->fy = Frame.imageParams.fy();
                e->cx =  Frame.imageParams.cx();
                e->cy =  Frame.imageParams.cy();
                e->bf = mbf;

                Optimizer->addEdge(e);
                point_edges_frameId[mp_id].push_back(edge_frameId_stereo((void*)e,f_info.first,true));
                totalNEdges++;
            }
        }
        //assert(nEdges>=2 &&);
    }

    ///////////////////////////////////////
    ///  ADD MARKERS as vertices and connection
    ///////////////////////////////////////

    ///Step3.3: 设置Marker顶点和边
    for(auto m:usedMarkersIdOp){
        ucoslam::Marker &marker= map->map_markers.at(m.first);
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(toSE3Quat( marker.pose_g2m.convert()));
        vSE3->setId(m.second);
       // vSE3->setMarginalized(true);
        Optimizer->addVertex(vSE3);
        totalVars+=6;


        //Now add the links between markers and frames
        for(auto fid:marker.frames){
            assert(usedFramesIdOpt[fid]!=INVALID_IDX);
            Frame &frame=map->keyframes.at(fid);
            MarkerEdge* e = new MarkerEdge(marker.size,marker.id,fid);
            Eigen::Matrix<double,8,1> obs;
            auto mcorners=frame.getMarker(marker.id);
            for(int i=0;i<4;i++){
                obs(i*2)=mcorners[i].x;
                obs(i*2+1)=mcorners[i].y;
            }
            e->setMeasurement(obs);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSE3));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(fid))));
            e->fx=frame.imageParams.fx();
            e->fy=frame.imageParams.fy();
            e->cx=frame.imageParams.cx();
            e->cy=frame.imageParams.cy();
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(15.51);


            e->setInformation( Eigen::Matrix< double, 8, 8 >::Identity());
            Optimizer->addEdge(e);
            totalNEdges++;
            g_vMarkerEdge.emplace_back(e);

        }
    }

    cout<<"TOTA NVARS="<<            totalVars<<endl;

}

///局部优化
void GlobalOptimizerG2O::optimize(std::shared_ptr<Map> map, const ParamSet &p )throw(std::exception) {
    ScopedTimerEvents timer("GlobalOptSimple");
    setParams(map,p);
    timer.add("Setparams");
    optimize();
    timer.add("Optmize");
    getResults(map);
    timer.add("getResults");

}





void GlobalOptimizerG2O::optimize( )throw(std::exception){


    ScopeTimer Timer("G2o Opt");
    Optimizer->initializeOptimization();
    Optimizer->setVerbose( _params.verbose);
    Optimizer->optimize(_params.nIters,1);
    bool bDoMore=false;
    if(bDoMore)
    {
        for( auto &mp_id:usedMapPoints)
        {
            for(auto e_fix:point_edges_frameId.at (mp_id)){
                EdgeSE3ProjectXYZ*e=((EdgeSE3ProjectXYZ*)e_fix.first);
                if( e->chi2()>6 || !e->isDepthPositive())
                    e->setLevel(1);
                e->setRobustKernel(0);

            }
        }
        // Optimize again without the outliers

        Optimizer->initializeOptimization();
        Optimizer->optimize(_params.nIters);

    }

    for (size_t i = 0; i < g_vMarkerEdge.size(); ++i)
    {
        MarkerEdge *e = g_vMarkerEdge[i];
        const float chi2 = e->chi2();
//        cout<<"chi2 = "<<chi2<<endl;
    }


}

void GlobalOptimizerG2O::getResults(std::shared_ptr<Map>  map)throw(std::exception){

    auto toCvMat=[](const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=eigMat(i,j);

        return cvMat.clone();

    };
    ///Keyframes
    for(uint32_t fid=0;fid<usedFramesIdOpt.size();fid++)
    {
        if (usedFramesIdOpt[fid]==INVALID_IDX ) continue;
        if (isFixedFrame[fid])continue;

        Frame &frame=map->keyframes[fid];
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(Optimizer->vertex(usedFramesIdOpt[fid]));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        frame.pose_f2g= toCvMat(SE3quat);
    }

    _badAssociations.clear();
    _badAssociations.reserve(usedMapPoints.size());
    //Points
    for( auto &mp_id:usedMapPoints)
    {
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(Optimizer->vertex( usedPointsIdOpt[mp_id]));
        auto est=vPoint->estimate();
        MapPoint &mp=map->map_points[mp_id];
        assert(!mp.isBad);
        mp.setCoordinates(cv::Point3f( est(0),est(1),est(2)));

        //check each projection
        for(auto e_fix:point_edges_frameId.at (mp.id)){
            bool isBad=false;
            if(e_fix.isStereo){
                if( ((EdgeStereoSE3ProjectXYZ*)e_fix.first)->chi2()>7.815 || !((EdgeStereoSE3ProjectXYZ*)e_fix.first)->isDepthPositive())
                    isBad=true;
            }
            else{
                if( ((EdgeSE3ProjectXYZ*)e_fix.first)->chi2()>6)  isBad=true;
            }
            //check no point is behind a camera
            if(!isBad) {
                cv::Point3f pincam=map->keyframes[e_fix.second].pose_f2g* mp.pose;
                if( pincam.z<0) isBad=true;
            }
            if(isBad)
                _badAssociations.push_back(std::make_pair(mp.id,e_fix.second));
        }
    }

    ///Markers
    for(auto &marker_idop:usedMarkersIdOp){
        g2o::VertexSE3Expmap* markerpose = static_cast<g2o::VertexSE3Expmap*>(Optimizer->vertex( marker_idop.second));
        g2o::SE3Quat SE3quat = markerpose->estimate();
        map->map_markers[marker_idop.first].pose_g2m= toCvMat(SE3quat);
    }

    //now, update normals and other values of the mappoints

    for( auto &mp_id:usedMapPoints)
        map->updatePointInfo(mp_id);
//    //take the points in these frames, that have a single projection and thus have not been optimized, their position must need an update
//     for(size_t i=0;i<usedPointsIdOpt.size();i++){
//        if (usedPointsIdOpt[i]==INVALID_VISITED_IDX){
//            auto &Mp=map->map_points[i];
//            if ( !Mp.isBad){
//                assert(Mp.frames.size()==1 && Mp.isStereo);
//                //recompute depth
//                const Frame &frame=map->keyframes[ Mp.frames.begin()->first];
//                auto p3d=frame.get3dStereoPoint(frame.depth[Mp.frames.begin()->second]);
//                //trasnform the point to the new ref system
//                auto g2f=frame.pose_f2g.inv();
//                p3d=g2f*p3d;
//                Mp.setCoordinates(p3d);
//                //update normal
//                 auto normal=frame.getCameraCenter()-p3d;
//                 Mp.updateNormals({normal});
//             }
//        }
//    }
#ifdef ORIGINAL_G2O_SBA

    float fxy=    w.TheImageParams.CameraMatrix.at<float>(0,0)+w.TheImageParams.CameraMatrix.at<float>(1,1);
    fxy/=2.;
    w.TheImageParams.CameraMatrix.at<float>(0,0)=fxy;
    w.TheImageParams.CameraMatrix.at<float>(1,1)=fxy;
#endif
}




}
