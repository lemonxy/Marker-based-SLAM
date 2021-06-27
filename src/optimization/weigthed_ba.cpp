#include "weigthed_ba.h"
#include "levmarq.h"
#include <opencv2/calib3d.hpp>
#include "../debug.h"
#include "proj_info.h"
namespace ucoslam{
void WeightedBA::optimize(se3 &pose_io, const std::vector<cv::Point3f>  &points, const std::vector<cv::Point2f> & points_2d,
                           const std::vector<float>  &points_weight,
                           const std::vector<cv::Point3f>  &marker_corners, const std::vector<cv::Point2f> & marker_corners_2d,
                           const ImageParams &ip, const Params &p )throw(std::exception){

    std::vector<float> points_weight_norm;
    float _weight_markers=0,_weight_points=0;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _jac_matrix;

    auto error_fast= [& ] (const LevMarq<double>::eVector  &sol, LevMarq<double>::eVector &err){
        //take the 3d points and project them
        //compute the error function

        _jac_matrix.resize((points.size()+marker_corners.size())*2,6);
        err.resize((points.size()+marker_corners.size())*2);
        se3 pose(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        //project the points of the map
        int idx=0,i=0;
        if (points.size()>0){
            for(auto p:points){
                auto proj_info=project_and_derive(p,pose,ip.CameraMatrix,false);
                err(idx)= /*points_weight_norm[i]**/( proj_info.x- points_2d[i].x);
                err(idx+1)= /*points_weight_norm[i]**/( proj_info.y- points_2d[i].y);
                //copy jacobian data
                for(int j=0;j<6;j++){
                    _jac_matrix(idx,j)= /*points_weight_norm[i]**/ proj_info.dx[j];
                    _jac_matrix(idx+1,j)=/*points_weight_norm[i]**/ proj_info.dy[j];
                }
                idx+=2;
                i++;
            }
        }
        //        //now, the marker corners
        if(marker_corners.size()>0){
            i=0;
            for(auto p:marker_corners){
                auto proj_info=project_and_derive(p,pose,ip.CameraMatrix,false);
                err(idx)= /*_weight_markers**/( proj_info.x- marker_corners_2d[i].x);
                err(idx+1)= /*_weight_markers**/( proj_info.y- marker_corners_2d[i].y);
                //copy jacobian data
                for(int j=0;j<6;j++){
                    _jac_matrix(idx,j)= /*_weight_markers**/ proj_info.dx[j];
                    _jac_matrix(idx+1,j)=/*_weight_markers**/ proj_info.dy[j];
                }
                idx+=2;
                i++;
            }
        }
//            cv::projectPoints(marker_corners,r,t,ip.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),proj,mJac);
//            for(size_t i=0;i<marker_corners.size();i++){
//                err(idx)= _weight_markers*( proj[i].x- marker_corners_2d[i].x);
//                err(idx+1)= _weight_markers*( proj[i].y- marker_corners_2d[i].y);
//                //copy jacobian data
//                for(int j=0;j<6;j++){
//                    _jac_matrix(idx,j)=_weight_markers*mJac.at<double>(i*2,j);
//                    _jac_matrix(idx+1,j)=_weight_markers*mJac.at<double>((i*2)+1,j);
//                }

//                idx+=2;
//            }
//        }
    };
    auto jacobian_f= [&](const LevMarq<double>::eVector  &solution,  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &jacobian) {
        (void)solution;
        jacobian=_jac_matrix;
    };



    //normalize to sum one the points weight
    if (points_weight.size()!=0){//if there are points weights, do normalizatin
        float weight_sum=0;
        for(auto w:points_weight) weight_sum+=w;
        float inv_sum=1./weight_sum;
        points_weight_norm.reserve(points.size()+marker_corners.size());
        for(auto w:points_weight) points_weight_norm.push_back( inv_sum*w);
    }
    else {//if empty, set a constant value
        float inv_sum=1./double(points.size());
        for(size_t i=0;i<points.size();i++)
            points_weight_norm.push_back( inv_sum);
    }


    //compute the relative weight of points and marker points
    if ( marker_corners.size()!=0)
        _weight_markers = p.w_markers /  double(marker_corners.size()) ;
    if(points.size()!=0)
        _weight_points = (1.0f-p.w_markers)/double(points.size());
    //normalize the sum
    double sum=_weight_markers+_weight_points;
    _weight_markers/=sum;
    _weight_points/=sum;
    //reset the weights to have them already computed
    for(auto &w:points_weight_norm) w*=_weight_points;


    //cout<<"wmarker="<<_weight_markers<<" wpoints="<<_weight_points<<endl;
    //_weight_markers_permarker=_weight_markers/double(marker_corners.size());

    LevMarq<double> solver;
    LevMarq<double>::eVector sol(6);//the pose
    for(int i=0;i<6;i++) sol(i)=pose_io[i];
    solver.setParams(p.nIters,1e-5,1e-8);
    solver.verbose()=p.verbose;
  if(0)  solver.solve(sol,std::bind( error_fast,std::placeholders::_1,std::placeholders::_2) );
  else  solver.solve(sol,std::bind( error_fast,std::placeholders::_1,std::placeholders::_2),std::bind(jacobian_f,std::placeholders::_1,std::placeholders::_2) );


    for(int i=0;i<6;i++) pose_io[i]=sol(i);
}


}
