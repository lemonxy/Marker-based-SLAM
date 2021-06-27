#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include "stuff/debug.h"
#include "stuff/utils.h"
#include "stuff/timers.h"
#include "basic_types/marker.h"
namespace  ucoslam {

typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>                               Vector8D;

class  MarkerEdgeOnlyProject: public  g2o::BaseBinaryEdge<8, Vector8D, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
    g2o::Vector3D points[4];

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double fx, fy, cx, cy;

  MarkerEdgeOnlyProject(double size){
      _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
      auto pointsA=ucoslam::Marker::get3DPointsLocalRefSystem(size);
      for(int i=0;i<4;i++){
          auto &p=pointsA[i];
          points[i]=g2o::Vector3D(p.x,p.y,p.z);
      }
  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}
inline Vector8D getError(){
 computeError();
 return _error;
}

  inline void computeError()  {
      //marker
      const g2o::VertexSE3Expmap* g2m= static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);//marker
    //camera
      const g2o::VertexSE3Expmap*  c2g = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);//camera

   // std::cout<<"g2m="<<g2m->estimate()<<" c2g="<<c2g->estimate()<<std::endl;
    //first, move the points to the global reference system, and then to the camera
    auto Transform_C2M=c2g->estimate()*  g2m->estimate();
    g2o::Vector3D points2[4];
     for(int i=0;i<4;i++){
        points2[i]=Transform_C2M.map(points[i]);//3d rigid transform
     }

    //now, project
     _error.resize(8);
    Vector8D obs(_measurement);
  //  std::cout<<obs<<std::endl;
    assert(_error.size()==obs.size());
    int idx=0;
    for(int i=0;i<4;i++){
        float projx=( points2[i](0)/points2[i](2)) *fx +cx;
        _error(idx)=obs(idx)-projx;
         idx++;
        float projy=( points2[i](1)/points2[i](2)) *fy +cy;
        _error(idx)=obs(idx)-projy;
         idx++;
    }
 //   std::cout<<"E="<<_error.transpose()<<std::endl;
  }

};

class  EdgeStereoSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZOnlyPose(){}


  void computeError()  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus(){
      g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
      Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

      double x = xyz_trans[0];
      double y = xyz_trans[1];
      double invz = 1.0/xyz_trans[2];
      double invz_2 = invz*invz;

      _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
      _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
      _jacobianOplusXi(0,2) = y*invz *fx;
      _jacobianOplusXi(0,3) = -invz *fx;
      _jacobianOplusXi(0,4) = 0;
      _jacobianOplusXi(0,5) = x*invz_2 *fx;

      _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
      _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
      _jacobianOplusXi(1,2) = -x*invz *fy;
      _jacobianOplusXi(1,3) = 0;
      _jacobianOplusXi(1,4) = -invz *fy;
      _jacobianOplusXi(1,5) = y*invz_2 *fy;

      _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
      _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
      _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
      _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
      _jacobianOplusXi(2,4) = 0;
      _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
    }


  Eigen::Vector3d cam_project(const Eigen::Vector3d & trans_xyz) const{
      const float invz = 1.0f/trans_xyz[2];
      Eigen::Vector3d res;
      res[0] = trans_xyz[0]*invz*fx + cx;
      res[1] = trans_xyz[1]*invz*fy + cy;
      res[2] = res[0] - bf*invz;
      return res;

  }

  Eigen::Vector3d Xw;
  double fx, fy, cx, cy, bf;
};

class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is){return false;}

    bool write(std::ostream& os) const{return false;}

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus(){
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;

        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;
    }
    Eigen::Vector2d project2d(const  Eigen::Vector3d& v)const  {
        Eigen::Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const{
        Eigen::Vector2d proj = project2d(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Eigen::Vector3d Xw;
    double fx, fy, cx, cy;
};



void g2o_poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d, se3 &pose_io,
                        std::vector<bool> &vBadMatches, const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor, std::vector<std::pair<ucoslam::Marker,aruco::Marker> > *marker_poses ){


    ScopedTimerEvents Timer("poseEstimation g2o_poseEstimation");


    auto  toSE3Quat=[](const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    };
    auto toCvMat=[](const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=eigMat(i,j);

        return cvMat.clone();

    };
    vBadMatches.resize( p3d.size());

    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    optimizer.setAlgorithm(solver);

    //Step1， 设置顶点：当前帧的位姿
    g2o::VertexSE3Expmap * G2oVertexCamera = new g2o::VertexSE3Expmap();
    G2oVertexCamera->setEstimate(toSE3Quat(pose_io.convert()));
    G2oVertexCamera->setId(0);
    G2oVertexCamera->setFixed(false);
    optimizer.addVertex(G2oVertexCamera);


    //Step2， 设置顶点：ORB特征点的位姿 Set MapPoint vertices

    std::vector< EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    std::vector<size_t> vnIndexEdgeMono;

    vpEdgesMono.reserve(p3d.size());
    vnIndexEdgeMono.reserve(p3d.size());

    const float deltaMono = sqrt(5.991);
    //const float deltaStereo = sqrt(7.815);


    float fx=CameraParams.at<float>(0,0);
    float fy=CameraParams.at<float>(1,1);
    float cx=CameraParams.at<float>(0,2);
    float cy=CameraParams.at<float>(1,2);

    for(size_t i=0; i<p3d.size(); i++)
    {
        vBadMatches[i]=false;
        Eigen::Matrix<double,2,1> obs;
        obs << p2d[i].pt.x , p2d[i].pt.y;

        EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(G2oVertexCamera));
        e->setMeasurement(obs);

        const float invSigma2 = invScaleFactor[p2d[i].octave];
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        e->fx = fx;
        e->fy = fy;
        e->cx = cx;
        e->cy = cy;
        e->Xw[0] = p3d[i].x;
        e->Xw[1] = p3d[i].y;
        e->Xw[2] = p3d[i].z;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vnIndexEdgeMono.push_back(i);
    }


    vector<MarkerEdgeOnlyProject*> vMarkerEdge;
    int videx=1;
    ///Step3, 设置顶点：Marker位姿（不优化）
    if (marker_poses!=0){
        for(auto mpose:*marker_poses){
            //add first the markers
            g2o::VertexSE3Expmap * G2oVertexMarker= new g2o::VertexSE3Expmap();
            G2oVertexMarker->setEstimate(toSE3Quat(mpose.first.pose_g2m.convert()));
            G2oVertexMarker->setId(videx++);
            G2oVertexMarker->setFixed(true);
            optimizer.addVertex(G2oVertexMarker);
            //now, the edge

            MarkerEdgeOnlyProject* e = new MarkerEdgeOnlyProject(mpose.first.size);
            Eigen::Matrix<double,8,1> obs;
            auto mcorners=mpose.second;
            for(int i=0;i<4;i++){
                obs(i*2)=mcorners[i].x;
                obs(i*2+1)=mcorners[i].y;
            }
            e->setMeasurement(obs);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(G2oVertexMarker));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( G2oVertexCamera));
            e->fx=fx;
            e->fy=fy;
            e->cx=cx;
            e->cy=cy;

            e->setInformation( Eigen::Matrix< double, 8, 8 >::Identity());
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(15.51);


            optimizer.addEdge(e);
            vMarkerEdge.emplace_back(e);
          }
    }



    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono=5.991;
    //const float chiMarker=15.51;
    std::vector<int> its={10,5,5,5};

    ///KeyPoints
    for(size_t it=0; it<its.size(); it++)
    {

        G2oVertexCamera->setEstimate(toSE3Quat(pose_io.convert()));
        optimizer.initializeOptimization(0);
        optimizer.setVerbose(debug::Debug::getLevel()>=10);
        optimizer.optimize(its[it],1);

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(vBadMatches[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono )
            {
                vBadMatches[idx]=true;
                e->setLevel(1);
            }
            else
            {
                vBadMatches[idx]=false;
                e->setLevel(0);
            }
        }

        if(optimizer.edges().size()<10)
            break;
    }

    for (int i = 0; i < vMarkerEdge.size(); ++i) {
        MarkerEdgeOnlyProject *e = vMarkerEdge[i];
        const float chi2 = e->chi2();
//        cout<<"Tracking chi2 = "<<chi2<<endl;
    }
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    pose_io=  toCvMat(SE3quat_recov);





//    double chiSum=0;
//    Se3Transform T;T=pose_io;
//    for(size_t p=0;p<p3d.size();p++){
//        cv::Point3f p3res=T*p3d[p];
//        if (p3res.z> 0) {
//            p3res.z=1./p3res.z;
//            cv::Point2f p2dProj( p3res.x *fx *p3res.z + cx,p3res.y *fy *p3res.z + cy);
//            cv::Point2f  err=p2d[p].pt- p2dProj ;
//            chiSum+=invScaleFactor[p2d[p].octave] * (err.x*err.x+ err.y*err.y);
//        }
//    }

//    cout<<"Pose Est chi2 sum="<<chiSum<<endl;
}







}
