#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_UNDH_
#define _UCOSLAM_GLOBAL_OPTIMIZER_UNDH_
#include "basic_types.h"
#include "exports.h"
#include "sparselevmarq.h"
#include <set>
namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations, assuming points are undistorted and making no camera parameters optimization
 */
class GlobalOptimizer_Und{


public:
    struct Params{
        friend class GlobalOptimizer_Und;
        Params(){}
        std::set<uint32_t> optimizing_frames;//set of frames points to be optimized. If emtpy, all of them
        std::set<uint32_t> fixed_markers;//set of markers to set fixed(not optimized)
        std::set<uint32_t> fixed_frames;//set of frames to set fixed
        int nIters=100;//number of iterations
         bool verbose=false;
        double w_markers=0.25;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points

    private:
        //for internal usage only
        int so_points;//start of points in the solution vector
        int so_markers;//start of markers in the solution vector
        int so_frames;//
        int n_frames;
        std::map<uint32_t,uint32_t> opt_markers;//for each marker to be optimized, its starting position in the solution vector
        std::map<uint32_t,uint32_t> opt_points;//for each point to be optimized, its starting position in the solution vector
        std::map<uint32_t,uint32_t> opt_frames;//for each frame to be optimized, its starting position in the solution vector

        //for each frame to be optimized we store information about the 3d points projecting in it,
        //we store a pair of vectors. The first one is the set of positions of the points in the solution vector
        //the seconc vector correspond to the 2d projections in the frame of these 3d points
        std::map<uint32_t,std::pair< std::vector<uint32_t>,std::vector<cv::Point2f> > >frame_points_sol_proj;
        //the same for markers
        std::map<uint32_t,std::pair< std::vector<uint32_t>,std::vector<cv::Point2f> > >frame_makers_sol_proj;

        //frame_marker_errorstart
        std::map<uint32_t,std::map<uint32_t,uint32_t > >frame_marker_errorstart;

        //fore ach marker, the set of frames it projects in
        std::map<uint32_t,std::vector<uint32_t> > marker_frames;

        //starting posision of errors for each frame
        std::map<uint32_t,uint32_t> frame_starterr;
        uint32_t errv_size;

        bool isFrameFixed(uint32_t fidx)const{return fixed_frames.find(fidx)!=fixed_frames.end();}
        float _w_m,_w_p;//final weight assigned to markers and points
    };


    void optimize(Map &map,FrameSet &fset,ImageParams &ip,  float markerSize,const Params &p=Params())throw(std::exception);



private:
    Params _params;

    void convert(  Map &map,  FrameSet &fset, ImageParams &ip, GlobalOptimizer_Und::Params &p,SparseLevMarq<double>::eVector &sol)throw (std::runtime_error);
    void convert(const SparseLevMarq<double>::eVector &sol, Map &map,FrameSet &fset,ImageParams &ip, const GlobalOptimizer_Und::Params &p);
    void error(const SparseLevMarq<double>::eVector &sol,SparseLevMarq<double>::eVector &err);
    void jacobian(const SparseLevMarq<double>::eVector &sol, Eigen::SparseMatrix<double> &);


    std::vector< Eigen::Triplet<double> > triplets;
   //ImageParams _ip;

    float _markerSize;
    FrameSet *_fset;

    //derivatives of repj error w.r.t. 3d point
    void point_part_derv(const cv::Mat &CameraMatrix,const cv::Mat &Distorsion,se3 &pose,cv::Point3f p3d,double &ux,double &uy,double &uz,double &vx,double &vy,double &vz);
    void  mult(se3 pose_g2m,const vector<cv::Point3f> &vpoints_in,vector<cv::Point3f> &vpoints_out);
    void join(const std::vector<std::vector< Eigen::Triplet<double> > > &in, std::vector< Eigen::Triplet<double> > &out,bool clearOutput=false);
    void undistort(const vector<cv::Point2f> &pin,vector<cv::Point2f> &pout,const ImageParams &ip);

    struct proj_info{
        float x,y;
        float dx[9];//dx_rx,dx_ry,dx_rz,dx_tx,dx_ty,dx_tz,dx_X,dx_Y,dx_Z
        float dy[9];//dy_rx,dy_ry,dy_rz,dy_tx,dy_ty,dy_tz,dy_X,dy_Y,dy_Z
    };
    void project(const vector<cv::Point3f> &points,const cv::Mat &camera_matrix,const cv::Mat &Rvec,const cv::Mat &Tvec, std::vector<proj_info> &pinfo);
};

}
#endif
