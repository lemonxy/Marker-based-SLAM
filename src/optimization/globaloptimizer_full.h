#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_FULL_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_FULL_H_
#include "slam.h"
#include "sparselevmarq.h"
#include <unordered_map>
#include "globaloptimizer.h"

#include "proj_info.h" //to remove from here
namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class GlobalOptimizerFull: public GlobalOptimizer{
public:

    void setParams(Slam &w,const ParamSet &p=ParamSet() )throw(std::exception);
    void optimize()throw(std::exception) ;
    void getResults(Slam &w)throw(std::exception);

    void optimize(Slam &w,const ParamSet &p=ParamSet() )throw(std::exception) ;
    string getName()const{return "simple";}
      void saveToStream_impl(std::ostream &str);
      void readFromStream_impl(std::istream &str);

private:
    void convert(Slam &sl, ParamSet &p, SparseLevMarq<float>::eVector &solution);
    void error(const SparseLevMarq<float>::eVector &sol,SparseLevMarq<float>::eVector &err);
    void jacobian(const SparseLevMarq<float>::eVector &sol, Eigen::SparseMatrix<float> &);
    se3 getFramePose(const SparseLevMarq<float>::eVector &sol,uint32_t frame_idx);
    void StepCallBackFunc( const SparseLevMarq<float>::eVector  &);


    void getMarker3dpoints(const se3 &pose_g2m, float markerSize, vector<cv::Point3f> &corners);

    inline void mult_matrix_point(float *rt_4x4,float *point_4_in,float *point_4_out){
            point_4_out[0]=rt_4x4[0]*point_4_in[0]+rt_4x4[1]*point_4_in[1]+rt_4x4[2]*point_4_in[2]+ rt_4x4[3];
            point_4_out[1]=rt_4x4[4]*point_4_in[0]+rt_4x4[5]*point_4_in[1]+rt_4x4[6]*point_4_in[2]+ rt_4x4[7];
            point_4_out[2]=rt_4x4[8]*point_4_in[0]+rt_4x4[9]*point_4_in[1]+rt_4x4[10]*point_4_in[2]+ rt_4x4[11];
    };

    ParamSet _params;
    std::unordered_map<uint32_t,uint32_t> point_pos;

    std::unordered_map<uint32_t,uint32_t> frame_solpos;//slow, access by the frame index
    std::vector<uint32_t> frame_solpos_fsi; //given a frame_sol_idx index (see prj_info), indicates the frame initial location in the solution vector
    std::vector<bool> fixed_frames_fsi; //given a frame_sol_idx index (see prj_info), indicates the frame initial location in the solution vector
    uint32_t _errVectorSize;



    struct point_prj_info{
        point_prj_info(uint32_t fidx,const cv::Point2f &p,uint32_t fs=0){
            frame_idx=fidx;
            proj=p;
            frame_sol_idx=fs;
        }

        uint32_t frame_idx;
        cv::Point2f proj;
        uint32_t frame_sol_idx;//internal index to speed up access to current pose
        int64_t errIdx=-1;//starting row position in the error vector
    };


    struct point_info{
        uint32_t sol_idx;//position in the solution vector
        vector<point_prj_info> frame_projections;//frame_idx in which it projects and its projection
    };

    struct marker_prj_info{
        marker_prj_info(uint32_t Frame_idx,const std::vector<cv::Point2f> Prj){frame_idx=Frame_idx;proj=Prj;}
        uint32_t frame_idx;
        vector<cv::Point2f> proj;
        uint32_t frame_sol_idx=0;//internal index to speed up access to current pose
        int64_t errIdx=-1;//starting row position in the error vector
    };
    struct marker_info{
        uint32_t sol_idx;//position in the solution vector
        vector<marker_prj_info> frame_projections;//frame_idx in which it projects and its projection
    };

    std::unordered_map<uint32_t,point_info> pmap_info;
    std::unordered_map<uint32_t,marker_info> markermap_info;
    std::map<uint32_t,se3> fixedframe_poses;
    cv::Mat cameraMatrix;
  //  float fx,fy,cx,cy;
    SparseLevMarq<float>::eVector  _solution;
    cv::Mat LPm;//precomputed left part of projection matrix
    SparseLevMarq<float> optimizer;
    std::vector< Eigen::Triplet<float> > triplets;

    double global_w_markers=0.25;
    double _weight_markers,_weight_points;

};
}
#endif
