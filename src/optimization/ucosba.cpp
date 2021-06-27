#include "ucosba.h"
#include <fstream>
#include "sparselevmarq.h"
#include  "../utils.h"
#include "proj_info.h"
#include "../io_utils.h"
namespace  ucosba {



struct proj_info{
    double x,y;
    double dx[9];//dx_rx,dx_ry,dx_rz,dx_tx,dx_ty,dx_tz,dx_X,dx_Y,dx_Z
    double dy[9];//dy_rx,dy_ry,dy_rz,dy_tx,dy_ty,dy_tz,dy_X,dy_Y,dy_Z
    friend std::ostream & operator<<(std::ostream &str,const proj_info &p){
        str<<p.x<<" "<<p.y<<"|";
        for(int i=0;i<9;i++)    str<<p.dx[i]<<" ";
        str<<"|";
        for(int i=0;i<9;i++)    str<<p.dy[i]<<" ";
        return str;

    }
};
inline  Point2f fast__project__(const Se3Transform & K,const  Point3f &p3d) {
    float Z_inv=1./ (p3d.x*K.data[8]+p3d.y*K.data[9]+p3d.z*K.data[10]+K.data[11]);
    return  Point2f (Z_inv*(p3d.x*K.data[0]+p3d.y*K.data[1]+p3d.z*K.data[2]+K.data[3]),Z_inv*(p3d.x*K.data[4]+p3d.y*K.data[5]+p3d.z*K.data[6]+K.data[7]) );
}
inline  cv::Point2f fast__project__(const float * data,const  cv::Point3f &p3d) {
    float Z_inv=1./ (p3d.x* data[8]+p3d.y* data[9]+p3d.z* data[10]+ data[11]);
    return  cv::Point2f (Z_inv*(p3d.x* data[0]+p3d.y* data[1]+p3d.z* data[2]+ data[3]),Z_inv*(p3d.x* data[4]+p3d.y* data[5]+p3d.z* data[6]+data[7]) );
}

inline proj_info  project_and_derive_cv(const  Point3f  &point_, const vector<float> &rt,const CameraParams &camera_params,bool calcDervXYZ=true) {

    cv::Point3f point(point_.x,point_.y,point_.z);
    vector<cv::Point3f> p3d(1);
    p3d[0]=point;
    ucoslam::se3 rt3(rt[0],rt[1],rt[2],rt[3],rt[4],rt[5]);

    vector<cv::Point2f > p2d;
    cv::Mat jac;
    float ci[9]={camera_params.fx,0,camera_params.cx,0,camera_params.fy,camera_params.cy,0,0,1};
    cv::Mat CamInt(3,3,CV_32F,ci);
    cv::projectPoints(p3d,rt3.getRvec(),rt3.getTvec(),CamInt,cv::Mat (),p2d,jac);
    proj_info pi;
    pi.x=p2d[0].x;
    pi.y=p2d[0].y;
    for(int i=0;i<6;i++){
        pi.dx[i]= jac.at<double>(0,i);
        pi.dy[i]= jac.at<double>(1,i);
    }

    if (calcDervXYZ){
        cv::Mat LPm=cv::Mat::eye(4,4,CV_32F);
        LPm.at<float>(0,0)=camera_params.fx;
        LPm.at<float>(0,2)=camera_params.cx;
        LPm.at<float>(1,1)=camera_params.fy;
        LPm.at<float>(1,2)=camera_params.cy;
        cv::Mat K=LPm*rt3.convert();//projection matrix
        float* Kptr=K.ptr<float>(0);

        float delta=1e-3;
        float invdelta2=1./(2.f*delta);
        cv::Point3f deltas[3]={cv::Point3f(delta,0,0),cv::Point3f(0,delta,0),cv::Point3f(0,0,delta)};
        for(int d=0;d<3;d++){
            auto pp=point+deltas[d];
            auto pm=point-deltas[d];
            auto pdif= fast__project__( Kptr,pp)-   fast__project__( Kptr,pm);
            pi.dx[6+d]=pdif.x*invdelta2;
            pi.dy[6+d]=pdif.y*invdelta2;
        }
    }


    //        auto pi2=project_and_derive_org(point,rt,camera_params);
    //        for(int i=0;i<6;i++){
    //            assert( fabs(pi.dx[i]-pi2.dx[i])<1e-3);
    //            assert( fabs(pi.dy[i]-pi2.dy[i])<1e-3);
    //        }
    //        if ( calcDervXYZ)
    //            for(int i=6;i<9;i++){
    //                assert( fabs(pi.dx[i]-pi2.dx[i])<1);
    //                assert( fabs(pi.dy[i]-pi2.dy[i])<1);
    //            }

    return pi;
}
inline proj_info  project_and_derive(const  Point3f  &point, const std::vector<float> &rt,const  CameraParams &camera_params,bool calcDervXYZ=true) {
    double fx=camera_params.fx;
    double cx=camera_params.cx;
    double fy=camera_params.fy;
    double cy=camera_params.cy;
    double rx=rt[0];
    double ry=rt[1];
    double rz=rt[2];
    double tx=rt[3];
    double ty=rt[4];
    double tz=rt[5];


    double nsqa=rx*rx + ry*ry + rz*rz;
    double a=std::sqrt(nsqa);
    double i_a=a?1./a:0;
    double rnx=rx*i_a;
    double rny=ry*i_a;
    double rnz=rz*i_a;
    double cos_a=cos(a);
    double sin_a=sin(a);
    double _1_cos_a=1.-cos_a;
    double r11=cos_a+rnx*rnx*_1_cos_a;
    double r12=rnx*rny*_1_cos_a- rnz*sin_a;
    double r13=rny*sin_a + rnx*rnz*_1_cos_a;
    double r21=rnz*sin_a +rnx*rny*_1_cos_a;
    double r22= cos_a+rny*rny*_1_cos_a;
    double r23= -rnx*sin_a+ rny*rnz*_1_cos_a;
    double r31= -rny*sin_a + rnx*rnz*_1_cos_a;
    double r32= rnx*sin_a + rny*rnz*_1_cos_a;
    double r33=cos_a+rnz*rnz*_1_cos_a;
    double _cos_a_1=cos_a-1;
    double rx2=rx*rx;
    double ry2=ry*ry;
    double rz2=rz*rz;


    proj_info pinf;
    const double &X=point.x;
    const double &Y=point.y;
    const double &Z=point.z;

    auto &pout=pinf;
    double Xc=X*r11 + Y*r12 + Z*r13 + tx;
    double Yc=X*r21 + Y*r22 + Z*r23 + ty;
    double Zc=X*r31+Y*r32 + Z*r33 +tz;
    pout.x= (Xc*fx/Zc)+cx;//projections
    pout.y= (Yc*fy/Zc)+cy;//projections


    //dx,dy wrt rx ry rz
    double q1=(nsqa*tz+rz*(rx*X+ry*Y+rz*Z)-(-nsqa*Z+rz*(rx*X+ry*Y+rz*Z))*cos_a+a*(-ry*X+rx*Y)*sin_a);
    double q2=(rx*X+ry*Y+rz*Z);
    double q3=(1/(nsqa*q1*q1));
    double q4=(-q2*rz-nsqa*tz+(q2*rz-nsqa*Z)*cos_a-a*(-ry*X+rx*Y)*sin_a);
    if (q1>1e-6){
        pout.dx[0]=q3*fx*(q4*(2*q2*rx2-nsqa*(2*rx*X+ry*Y+rz*Z)+(-2*q2*rx2+nsqa*(ry*Y+rz*Z+rx*(2*X+rz*Y-ry*Z)))*cos_a+a*rx*(nsqa*X-rx2*X-rz*Y+ry*Z-rx*(ry*Y+rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
        pout.dx[1]=q3*fx*(q4*(rx*(2*q2*ry-nsqa*Y)+(-2*rx2*ry*X+nsqa*ry*(rz*Y-ry*Z)+rx*(nsqa*Y-2*ry*(ry*Y+rz*Z)))*cos_a-a*(nsqa*(-ry*X+Z)+ry*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
        pout.dx[2]=q3*fx*((q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*((-nsqa+2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(rx*(2*q2*rz-nsqa*Z)+(-2*rx2*rz*X+nsqa*rz*(rz*Y-ry*Z)+rx*(nsqa*Z-2*rz*(ry*Y+rz*Z)))*cos_a+a*(nsqa*(rz*X+Y)-rz*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a));
        pout.dy[0]=q3*fy*(-q1*(ry*(2*q2*rx-nsqa*X)+(-2*q2*rx*ry+nsqa*(ry*X-rx*rz*X+rx2*Z))*cos_a+a*(rx*(-rx*ry*X+rz*X+nsqa*Y-ry2*Y)+(nsqa-rx*(rx+ry*rz))*Z)*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
        pout.dy[1]=q3*fy*(-q1*(2*q2*ry2-nsqa*(rx*X+2*ry*Y+rz*Z)+(-2*q2*ry2+nsqa*(rx*X-ry*rz*X+2*ry*Y+rx*ry*Z+rz*Z))*cos_a+a*ry*(rz*X+nsqa*Y-ry2*Y-ry*rz*Z-rx*(ry*X+Z))*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
        pout.dy[2]=q3*fy*((q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(-(nsqa-2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(ry*(2*q2*rz-nsqa*Z)+(rx*rz*(-2*ry*X+nsqa*Z)+nsqa*(-rz2*X+ry*Z)-2*ry*rz*(ry*Y+rz*Z))*cos_a-a*(nsqa*(X-rz*Y)+rz*(-rz*X+ry2*Y+ry*rz*Z+rx*(ry*X+Z)))*sin_a));
    }
    else{
        pout.dx[0]=pout.dx[1]=pout.dx[2]=pout.dy[0]=pout.dy[1]=pout.dy[2]=0;
    }



    double a1=(tz+Z*(rnz*rnz* _1_cos_a+cos_a)+Y *(rny*rnz*_1_cos_a+rnx*sin_a)+X *(rnx*rnz*_1_cos_a-rny*sin_a)) ;
    double a2=rnx*rnx*_1_cos_a+cos_a;
    double a3=rnx*rnz*_1_cos_a;
    double a4=rny*sin_a;
    double a5=rnx*rny*_1_cos_a;
    double a6=rnz*Y*sin_a;
    double a7=(ty+a5*X+rny*rny*Y+rny*rnz*Z+(Y-rny*rny*Y-rny* rnz*Z)*cos_a+(rnz*X-rnx*Z)*sin_a);
    double a8=(-a6+tx+a2*X+a5*Y+(a3+a4)*Z);
    double _inv_a1_2=1./(a1*a1);




    //dx_tx
    pout.dx[3]= fx/a1;
    //dx_ty
    pout.dx[4]=0;
    //dx_tz
    pout.dx[5]= -((a8*fx)*_inv_a1_2);
    if (calcDervXYZ){
        //dx_X_errVectorSize
        pout.dx[6]= ((a1*a2-(a3-a4)*a8)*fx)*_inv_a1_2;
        //dx_Y
        pout.dx[7]= (fx*(-a8*(rny*rnz-rny*rnz*cos_a+rnx*sin_a)+a1*(a5-rnz*sin_a)))*_inv_a1_2;
        //dx_Z
        pout.dx[8]= (fx*(a1*(a3+a4)-a8*(rnz*rnz+cos_a-rnz*rnz*cos_a))) *_inv_a1_2;
    }

    pout.dy[3]=0;
    pout.dy[4]= fy/a1;
    pout.dy[5]= -((a7*fy)*_inv_a1_2);

    if (calcDervXYZ){
        pout.dy[6]= (fy*(-(a3-a4)*a7+a1*(a5+rnz*sin_a))) *_inv_a1_2;
        pout.dy[7]= (fy*(a1*(rny*rny+cos_a-rny*rny* cos_a)-a7*(rny*rnz-rny*rnz *cos_a+rnx*sin_a)))*_inv_a1_2;
        pout.dy[8]= (fy*(-a7*(rnz*rnz+cos_a-rnz*rnz*cos_a)-a1* (rny*rnz*(_cos_a_1)+rnx*sin_a)))*_inv_a1_2;

    }
    return pinf;
}



inline  Point2f undistort(const  Point2f &p2d,const CameraParams &cameraParams ){
    assert(false);
return p2d;
//    std::vector<cv::Point2f> pin(1),pout;
//    pin[0].x=p2d.x;
//    pin[0].y=p2d.y;
//    cv::undistortPoints(pin,pout,)
//    //take the normalized value
//    Point2f norm;
//    norm.x=( p2d.x-cameraParams.cx)/cameraParams.fx;
//    norm.y=( p2d.y-cameraParams.cy)/cameraParams.fy;
//    //now, remove distortion
}



inline  Point2f project(const  Point3f &p3d,const CameraParams &cameraParams,const Se3Transform &RT){

    const float *rt=RT.data;

    //project point first
    float x= p3d.x* rt[0]+p3d.y* rt[1]+p3d.z* rt[2]+rt[3];
    float y= p3d.x* rt[4]+p3d.y* rt[5]+p3d.z* rt[6]+rt[7];
    float z= p3d.x* rt[8]+p3d.y* rt[9]+p3d.z* rt[10]+rt[11];

    float xx=x/z;
    float yy=y/z;

    if (cameraParams.hasDistortion()){//apply distortion //        k1,k2,p1,p2[,k3
        const float *k=cameraParams.dist;
        float xx2=xx*xx;
        float yy2=yy*yy;
        float r2=xx2+yy2;
        float r4=r2*r2;
        float comm=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
        float xxx = xx * comm + 2*k[2] *xx2+ k[3]*(r2+2*xx2);
        float yyy= yy*comm+ k[2]*(r2+2*yy2)+2*k[3]*xx*yy;
        xx=xxx;
        yy=yyy;
    }
    return  Point2f((xx*cameraParams.fx)+cameraParams.cx,(yy*cameraParams.fy)+cameraParams.cy );
}



double SBA_Data::computeChi2(){
    //project all points
    double Chi2Sum=0;
    for(PointInfo &pinfo: pointSetInfo){
        for(PointProjection &pprj:pinfo.pointProjections){
            Point2f proj= project(pinfo.p3d,camParams,cameraSetInfo[pprj.camIndex].pose);
            float errX=proj.x-pprj.p2d.x;
            float errY=proj.y-pprj.p2d.y;
            Chi2Sum+=pprj.chi2= pprj.weight* (errX*errX+errY*errY);
        }
    }
    return Chi2Sum;
}

void BasicOptimizer::fromSolutionToSBAData(const ucoslam::SparseLevMarq<double>::eVector &sol,SBA_Data &BAD){
    //for each camera, restore its pose
    for(auto &csi: BAD.cameraSetInfo)
        if (!csi.fixed)
            csi.pose.fromRTVec(   sol(csi.sol_index),sol(csi.sol_index+1),sol(csi.sol_index+2),sol(csi.sol_index+3),sol(csi.sol_index+4),sol(csi.sol_index+5));

    //the same for the points
    for(PointInfo &pi:BAD.pointSetInfo)
        if (!pi.fixed){
            pi.p3d.x=sol( pi.sol_index);
            pi.p3d.y=sol( pi.sol_index+1);
            pi.p3d.z=sol( pi.sol_index+2);
        }
}

Se3Transform getProjectionMatrix(CameraParams &cam,const Se3Transform &RT){
//precomputed projection matrix lhs
//  |fx 0   cx |   |1 0 0 0|
//  |0  fy  cy | * |0 1 0 0|
//  |0  0   1  |   |0 0 1 0|
Se3Transform   LPm;

float m[16]={cam.fx,0,cam.cx,0,
             0,cam.fy,cam.cy,0,
             0,0,1,0,
             0,0,0,1};
memcpy(LPm.data,m,16*sizeof(float));
return LPm*RT;
}

constexpr float epsilon=1e-3;
constexpr float epsilon_inv=1000;

constexpr float epsilon_2=2*1e-3;
constexpr float epsilon_2_inv=1./(2*1e-3);
//returns the projection matrices for rx,ry,...,tz
std::vector<std::pair<Se3Transform,Se3Transform> > computeDerivativeProjectionMatrices(CameraParams &cam, Se3Transform &T){
    std::vector<std::pair<Se3Transform,Se3Transform> > DervM(6);

    vector<float> rt=T.toRTVec();
    Se3Transform M;
    for(int i=0;i<6;i++)
    {

        float org=rt[i];
        rt[i]=org+epsilon;
        M.fromRTVec(rt);
        DervM[i].first= getProjectionMatrix(cam,M);
        rt[i]=org-epsilon;
        M.fromRTVec(rt);
        DervM[i].second= getProjectionMatrix(cam,M);

        rt[i]=org;
    }
    return DervM;
}

proj_info project_and_derive_2(const std::vector<std::pair<Se3Transform,Se3Transform> > &DervM,Se3Transform &Korg,Point3f p3d){
    proj_info pi;
    Point2f proj=fast__project__ (Korg,p3d);
    pi.x=proj.x;
    pi.y=proj.y;

    for(int i=0;i<6;i++){
        Point2f p1=fast__project__ (DervM[i].first,p3d);
        pi.dx[i]=double(p1.x-proj.x)*epsilon_inv;
        pi.dy[i]=double(p1.y-proj.y)*epsilon_inv;
    }
    //finally, compute for X
    Point3f p3dp=p3d;p3dp.x+=epsilon;
    Point2f p1=fast__project__ (Korg,p3dp);
    pi.dx[6]=double(p1.x-proj.x)*epsilon_inv;
    pi.dy[6]=double(p1.y-proj.y)*epsilon_inv;

    p3dp=p3d;p3dp.y+=epsilon;
    p1=fast__project__ (Korg,p3dp);
    pi.dx[7]=double(p1.x-proj.x)*epsilon_inv;
    pi.dy[7]=double(p1.y-proj.y)*epsilon_inv;

    p3dp=p3d;p3dp.z+=epsilon;
    p1=fast__project__ (Korg,p3dp);
    pi.dx[8]=double(p1.x-proj.x)*epsilon_inv;
    pi.dy[8]=double(p1.y-proj.y)*epsilon_inv;

    return pi;
}
std::vector<pair<double,double> > computeDerivativs( const std::vector<std::pair<Se3Transform,Se3Transform> > &DervM,Se3Transform &Korg,Point3f p3d){
    std::vector<pair<double,double> > ders(9);
    //first ones
    for(int i=0;i<6;i++){
        Point2f p1=fast__project__ (DervM[i].first,p3d);
        Point2f p2=fast__project__ (Korg,p3d);
        ders[i].first=double(p1.x-p2.x)/epsilon;
        ders[i].second=double(p1.y-p2.y)/epsilon;
    }
    //finally, compute for X
    Point3f p3dp(p3d.x+epsilon,p3d.y,p3d.z);
    Point3f p3dn(p3d.x-epsilon,p3d.y,p3d.z);
    Point2f p1=fast__project__ (Korg,p3dp);
    Point2f p2=fast__project__ (Korg,p3dn);
    ders[6].first=double(p1.x-p2.x)*epsilon_2_inv;
    ders[6].second=double(p1.y-p2.y)*epsilon_2_inv;
    //Y
    p3dp=Point3f(p3d.x,p3d.y+epsilon,p3d.z);
    p3dn=Point3f(p3d.x,p3d.y-epsilon,p3d.z);
    p1=fast__project__ (Korg,p3dp);
    p2=fast__project__ (Korg,p3dn);
    ders[7].first=double(p1.x-p2.x)*epsilon_2_inv;
    ders[7].second=double(p1.y-p2.y)*epsilon_2_inv;

    //Z
    p3dp=Point3f(p3d.x,p3d.y,p3d.z+epsilon);
    p3dn=Point3f(p3d.x,p3d.y,p3d.z-epsilon);
    p1=fast__project__ (Korg,p3dp);
    p2=fast__project__ (Korg,p3dn);
    ders[8].first=double(p1.x-p2.x)*epsilon_2_inv;
    ders[8].second=double(p1.y-p2.y)*epsilon_2_inv;
    return ders;
}


void BasicOptimizer::jacobian(const ucoslam::SparseLevMarq<double>::eVector &sol, Eigen::SparseMatrix<double> &jacobian){
    triplets.clear();
    jacobian.resize(_errorSize,sol.size());
    //copy solution to SBA_Data
    fromSolutionToSBAData(sol,*_data);
    //start computation of projection errors
    uint32_t err_idx=0;
    for(uint32_t cam=0;cam<cam_points.size();cam++){
        CameraInfo &camInfo=_data->cameraSetInfo[ cam ];
        auto DervMatrices=computeDerivativeProjectionMatrices(_data->camParams, camInfo.pose);
        Se3Transform K=getProjectionMatrix(_data->camParams,camInfo.pose);

        for(const PointProjection2 &pp2 :cam_points[cam] ){
            const PointInfo&pinfo=_data->pointSetInfo[ pp2.pointIndex ];

            //now compute derivatives
            auto pp=project_and_derive_2(DervMatrices,K,pinfo.p3d);

            if(!camInfo.fixed){//not a fixed camera
                for(int i=0;i<6;i++){
                    triplets.push_back(Eigen::Triplet<double>( err_idx,  camInfo.sol_index+i ,pp.dx[i]) );
                    triplets.push_back(Eigen::Triplet<double>( err_idx+1,camInfo.sol_index+i ,pp.dy[i]) );
                    assert(err_idx+1<_errorSize);
                    assert(camInfo.sol_index+i<sol.size());
                }
            }
            //now, for x,y,z
            if (!pinfo.fixed) {
                for(int i=0;i<3;i++){
                    triplets.push_back(Eigen::Triplet<double>( err_idx,  pinfo.sol_index+i ,pp.dx[6+i]) );
                    triplets.push_back(Eigen::Triplet<double>( err_idx+1,pinfo.sol_index+i ,pp.dy[6+i]) );
                    assert(err_idx+1<_errorSize);
                    assert(pinfo.sol_index+i<sol.size());
                }
            }

            err_idx+=2;
        }
    }
    jacobian.setFromTriplets(triplets.begin(),triplets.end());

}

void BasicOptimizer::callback(const ucoslam::SparseLevMarq<double>::eVector &sol) {
    auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };
     auto separe=[](uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);};

    auto getMap=[&]( Eigen::SparseMatrix<double>  &J){
        std::map<uint64_t,double> sJmap;
            for (int k=0; k<J.outerSize(); ++k)
              for ( Eigen::SparseMatrix<double> ::InnerIterator it(J,k); it; ++it)
                  sJmap.insert( {join(it.row(),it.col()),it.value()});
            return sJmap;
    };

    auto saveToFile=[](const std::map<uint64_t,double> &Map,string path ){
        ofstream file(path,std::ios::binary);
        ucoslam::toStream__kv(Map,file);
    };
    auto readFromFile=[](string path ){
        std::map<uint64_t,double> Map;
        ifstream file(path,std::ios::binary);
        ucoslam::fromStream__kv(Map,file);
        return Map;
    };


    std::map<uint64_t,double> correctMap;
    if (1){
        Eigen::SparseMatrix<double> correctJac(_errorSize,sol.size());
        optimizer.calcDerivates(sol,correctJac ,std::bind(&BasicOptimizer::error,this,std::placeholders::_1,std::placeholders::_2));
        correctMap=getMap(correctJac);
        saveToFile(correctMap,"smap.bin");

    }
    else{
        correctMap=readFromFile("smap.bin");
    }

    Eigen::SparseMatrix<double> customJac(_errorSize,sol.size());
    jacobian(sol,customJac);
    std::map<uint64_t,double> customMap=getMap(customJac);
    //find big differencs
    for(auto j:correctMap){
        if (customMap.count(j.first)==0 ){
            auto rc=separe(j.first);
            cerr<<"error in="<<rc.first<<" "<<rc.second<<":"<< j.second<<endl;
        }
        else{
            if (fabs(customMap[j.first]-j.second)>1e-1 ){
                auto rc=separe(j.first);
                cerr<<"diff in="<<rc.first<<" "<<rc.second<<" : custom="<<customMap[j.first]<<" org="<<j.second<<endl;
                if (rc.second>startCamPoseSols)
                 cerr<<"   "<< (rc.second-startCamPoseSols)/6<< " "<<(rc.second-startCamPoseSols)%6<<endl;
            }
        }
    }
    cout<<endl;


}

void  BasicOptimizer::error(const ucoslam::SparseLevMarq<double>::eVector &sol,ucoslam::SparseLevMarq<double>::eVector &err){
    //copy solution to SBA_Data
    fromSolutionToSBAData(sol,*_data);
    //start computation of projection errors
    err.resize(_errorSize);
    //now, let's project
    uint32_t err_idx=0;
    for(uint32_t cam=0;cam<cam_points.size();cam++){
        Se3Transform K=getProjectionMatrix(_data->camParams,_data->cameraSetInfo[ cam ].pose);
        for(const PointProjection2 &pp2 :cam_points[cam] ){
            const PointInfo&pinfo=_data->pointSetInfo[ pp2.pointIndex ];
            Point2f reprj= fast__project__(K,pinfo.p3d);
            double errX= reprj.x-pp2.p2d.x ;
            double errY= reprj.y-pp2.p2d.y ;
            double SqErr=(errX*errX+ errY*errY);
            double robust_weight= getHubberMonoWeight(SqErr,pp2.weight);
            err(err_idx++)=robust_weight*errX;
            err(err_idx++)=robust_weight*errY;
        }
    }
}

//void  BasicOptimizer::error(const ucoslam::SparseLevMarq<double>::eVector &sol,ucoslam::SparseLevMarq<double>::eVector &err){
//    //copy solution to SBA_Data
//    fromSolutionToSBAData(sol,*_data);
//    //start computation of projection errors
//    err.resize(_errorSize);
//    //now, let's project
//    float fx=_data->camParams.fx;
//    float fy=_data->camParams.fy;
//    float cx=_data->camParams.cx;
//    float cy=_data->camParams.cy;
//    uint32_t err_idx=0;
//    for(PointInfo &pi:_data->pointSetInfo){
//        for(PointProjection prjts:pi.pointProjections){
//            //move the point to camera coordinates
//            Point3f pcam= _data->cameraSetInfo[prjts.camIndex].pose*pi.p3d;
//            if (pcam.z>0){
//                pcam.z=1./pcam.z;
//                //project
//                Point2f reprj((pcam.x*pcam.z*fx)+cx,(pcam.y*pcam.z*fy)+cy );
//                double errX= prjts.p2d.x- reprj.x;
//                double errY= prjts.p2d.y- reprj.y;
//                double SqErr=(errX*errX+ errY*errY);
//                double robust_weight= getHubberMonoWeight(SqErr,prjts.weight);
//                err(err_idx++)=robust_weight*errX;
//                err(err_idx++)=robust_weight*errY;
//            }
//            else{
//                err(err_idx++)=0;
//                err(err_idx++)=0;
//            }
//        }
//    }
//}

void BasicOptimizer::optimize(SBA_Data &data, OptimizationParams params)
{
    _params=params;
    _data=&data;
    //create the solution vector with non fixed points and cameras
    std::vector<float> vsol;
    vsol.reserve(data.pointSetInfo.size()*3+data.cameraSetInfo.size()*3);
    //first add the points
    for(PointInfo &pi:data.pointSetInfo){
        if (!pi.fixed){
            pi.sol_index=vsol.size();//store its position in the solution vector
            vsol.push_back(pi.p3d.x);
            vsol.push_back(pi.p3d.y);
            vsol.push_back(pi.p3d.z);
        }
    }

    startCamPoseSols=vsol.size();
    cout<<"startCamPoseSols="<<startCamPoseSols<<endl;
    for(CameraInfo &cpose:data.cameraSetInfo){
        if (!cpose.fixed) {
            cpose.sol_index=vsol.size();//store position in solution vector
            auto rt=cpose.pose.toRTVec();
            for(int i=0;i<6;i++) vsol.push_back(rt[i]);
        }
    }
    //now, let us compute the error size
    _errorSize=0;
    for(PointInfo &pinfo: data.pointSetInfo)
        _errorSize+=2*pinfo.pointProjections.size();

    //finally, remove distortion if required
    if (data.camParams.hasDistortion())
        for(PointInfo &pi:data.pointSetInfo)
            for(PointProjection &prjts:pi.pointProjections)
                prjts.p2d=undistort(prjts.p2d,data.camParams);
    for(int i=0;i<5;i++)
        data.camParams.dist[i]=0;



//now, lets compute the camera-points info
    cam_points.resize(data.cameraSetInfo.size());
    for(size_t i=0;i<data.pointSetInfo.size();i++){
        for(const PointProjection &prjts:data.pointSetInfo[i].pointProjections){
            PointProjection2 pp2;
            pp2.p2d=prjts.p2d;
            pp2.camIndex=prjts.camIndex;
            pp2.pointIndex=i;
            pp2.weight=prjts.weight;
            cam_points[prjts.camIndex ].push_back(pp2);
        }
    }





    //start the optimization
    solution.resize(vsol.size());
    for(size_t i=0;i<vsol.size();i++)        solution(i)=vsol[i];

    ucoslam::SparseLevMarq<double>::Params SL_params;
    SL_params.maxIters=_params.maxIterations;
    SL_params.minError=_params.minGlobalChi2;
    SL_params.min_step_error_diff=_params.minStepChi2;
    SL_params.use_omp=false;
    SL_params.tau=1e-3;

   // optimizer.setStepCallBackFunc(std::bind(&BasicOptimizer::callback,this,std::placeholders::_1));
    ucoslam::SparseLevMarq<double>::eVector err;

    error(solution,err);
    cout<<"CHI2 start="<<       err.cwiseProduct(err).sum()<<endl;

    Eigen::SparseMatrix<double> customJac(_errorSize,solution.size());
    jacobian(solution,customJac);
   // callback(solution);
     SL_params.verbose=true;
    optimizer.setParams(SL_params);

    //optimizer.solve( solution,std::bind(&BasicOptimizer::error,this,std::placeholders::_1,std::placeholders::_2));
    optimizer.solve( solution,std::bind(&BasicOptimizer::error,this,std::placeholders::_1,std::placeholders::_2),std::bind(&BasicOptimizer::jacobian,this,std::placeholders::_1,std::placeholders::_2));

}



template< typename T>
void   toStream__ts ( const  std::vector<T> &v,std::ostream &str ) {
    uint32_t s=v.size();
    str.write ( ( char* ) &s,sizeof ( s) );
    for(size_t i=0;i<v.size();i++) v[i].toStream( str);
}

template< typename T>
void   fromStream__ts (    std::vector<T> &v,std::istream &str ) {
    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    v.resize(s);
    for(size_t i=0;i<v.size();i++) v[i].fromStream( str);
}

void Se3Transform::toStream(std::ostream &str)const{
    str.write((char*)data,16*sizeof(float));

}


void Se3Transform::fromStream(std::istream &str){
    str.read((char*)data,16*sizeof(float));
}
void CameraInfo::toStream(std::ostream &str) const{

    str.write((char*)&fixed, sizeof(fixed));
    str.write((char*)&id, sizeof(id));
    str.write((char*)&sol_index, sizeof(sol_index));
}

void CameraInfo::fromStream(std::istream &str)
{
    str.read((char*)&fixed, sizeof(fixed));
    str.read((char*)&id, sizeof(id));
    str.read((char*)&sol_index, sizeof(sol_index));

}

void PointProjection::toStream(std::ostream &str)const{
    str.write((char*)&camIndex,sizeof(camIndex));
    str.write((char*)&p2d,sizeof(p2d));
    str.write((char*)&chi2,sizeof(chi2));
    str.write((char*)&weight,sizeof(weight));

}
void PointProjection::fromStream(std::istream &str){
    str.read((char*)&camIndex,sizeof(camIndex));
    str.read((char*)&p2d,sizeof(p2d));
    str.read((char*)&chi2,sizeof(chi2));
    str.read((char*)&weight,sizeof(weight));
}

void PointInfo::toStream(std::ostream &str)const{
    str.write((char*)&p3d,sizeof(p3d));
    str.write((char*)&fixed,sizeof(fixed));
    str.write((char*)&id,sizeof(id));
    str.write((char*)&sol_index,sizeof(sol_index));


    toStream__ts(pointProjections,str);
}
void PointInfo::fromStream(std::istream &str){
    str.read((char*)&p3d,sizeof(p3d));
    str.read((char*)&fixed,sizeof(fixed));
    str.read((char*)&id,sizeof(id));
    str.read((char*)&sol_index,sizeof(sol_index));
    fromStream__ts(pointProjections,str);
}



void SBA_Data::saveToFile(std::string path)const{
    std::ofstream file(path,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for writing");
    uint64_t sig=23123;
    file.write((char*)&sig,sizeof(sig));
    toStream(file);
}
void SBA_Data::readFromFile(std::string path){

    std::ifstream file(path,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for reading");
    uint64_t sig=23123;
    file.read((char*)&sig,sizeof(sig));
    if (sig!=23123)throw std::runtime_error("Invalid file type");
    fromStream(file);
}

void CameraParams::toStream(std::ostream &str)const{


    str.write((char*)&width,sizeof(width));
    str.write((char*)&height,sizeof(height));
    str.write((char*)&fx,sizeof(fx));
    str.write((char*)&fy,sizeof(fy));
    str.write((char*)&cx,sizeof(cx));
    str.write((char*)&cy,sizeof(cy));
    for(int i=0;i<5;i++)
        str.write((char*)&dist[i],sizeof(dist[i]));

}
void CameraParams::fromStream(std::istream &str){
    str.read((char*)&width,sizeof(width));
    str.read((char*)&height,sizeof(height));
    str.read((char*)&fx,sizeof(fx));
    str.read((char*)&fy,sizeof(fy));
    str.read((char*)&cx,sizeof(cx));
    str.read((char*)&cy,sizeof(cy));
    for(int i=0;i<5;i++)
        str.read((char*)&dist[i],sizeof(dist[i]));

}

void SBA_Data::toStream(std::ostream &str)const{


    camParams.toStream(str);
    toStream__ts(cameraSetInfo,str);
    toStream__ts(pointSetInfo,str);

}
void SBA_Data::fromStream(std::istream &str){

    camParams.fromStream(str);
    fromStream__ts(cameraSetInfo,str);
    fromStream__ts(pointSetInfo,str);

}
std::vector<float> Se3Transform::toRTVec()const{
    cv::Mat M(4,4,CV_32F,(float*)data),Rvec;
    cv::Rodrigues(M.rowRange(0,3).colRange(0,3),Rvec);
    assert(Rvec.type()==CV_32F);
    float *rv=Rvec.ptr<float>(0);
    return {rv[0],rv[1],rv[2], data[3],data[7],data[11] };
}



void Se3Transform::fromRTVec(float rx,float ry,float rz,float tx,float ty,float tz){
    float nsqa=rx*rx + ry*ry + rz*rz;
    float a=std::sqrt(nsqa);
    float i_a=a?1./a:0;
    float rnx=rx*i_a;
    float rny=ry*i_a;
    float rnz=rz*i_a;
    float cos_a=cos(a);
    float sin_a=sin(a);
    float _1_cos_a=1.-cos_a;
    data[0] =cos_a+rnx*rnx*_1_cos_a;
    data[1]=rnx*rny*_1_cos_a- rnz*sin_a;
    data[2]=rny*sin_a + rnx*rnz*_1_cos_a;
    data[3]=tx;
    data[4]=rnz*sin_a +rnx*rny*_1_cos_a;
    data[5]=cos_a+rny*rny*_1_cos_a;
    data[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
    data[7]=ty;
    data[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
    data[9]= rnx*sin_a + rny*rnz*_1_cos_a;
    data[10]=cos_a+rnz*rnz*_1_cos_a;
    data[11]=tz;
    data[12]=data[13]=data[14]=0;
    data[15]=1;
}
  std::ostream &operator<<(std::ostream &str,const Se3Transform &ci){
      for(int i=0;i<16;i++)      str<<ci.data[i]<<" ";
       return str;
  }
  Se3Transform Se3Transform::operator *(const Se3Transform &m)const{
      Se3Transform res;
      for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
              res(i,j)=0;
              for(int k=0;k<4;k++)
                  res(i,j)+=  (*this)(i,k)*m(k,j);
          }
      }
       return res;
  }

}
