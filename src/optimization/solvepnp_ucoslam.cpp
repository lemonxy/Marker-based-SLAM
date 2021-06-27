#include "levmarq.h"
#include <opencv2/core.hpp>
#include "stuff/timers.h"
#include "stuff/se3.h"
#include "stuff/se3transform.h"
#include "basic_types/marker.h"
#include "aruco/marker.h"
namespace ucoslam {
inline double hubber(double e,double _delta){
double dsqr = _delta * _delta;
 if (e <= dsqr) { // inlier
   return  e;
 } else { // outlier
   double sqrte = sqrt(e); // absolut value of the error
   return  2*sqrte*_delta - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
 }
}

inline double hubberMono(double e){
    if (e <= 5.991) { // inlier
      return  e;
    } else  // outlier
       return  4.895303872*sqrt(e) - 5.991; // rho(e)   = 2 * delta * e^(1/2) - delta^2
}

inline double getHubberMonoWeight(double SqErr ){
    if (SqErr==0) return 1;
     return sqrt(hubberMono(  SqErr)/ SqErr);
}

 void ucoslam_poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d, se3 &pose_io, std::vector<bool> &vBadMatches,
                    const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor,  std::vector<std::pair<ucoslam::Marker,aruco::Marker> > *marker_poses=0)
{
    ScopedTimerEvents Timer("ucoslam_poseEstimation");




    vBadMatches.resize(p3d.size());
    for(size_t i=0;i<vBadMatches.size();i++)vBadMatches[i]=false;

    float fx=CameraParams.at<float>(0,0);
    float fy=CameraParams.at<float>(1,1);
    float cx=CameraParams.at<float>(0,2);
    float cy=CameraParams.at<float>(1,2);

    const float chi2Mono=5.991;


    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> jacobian;

    auto  calcDerivates=[&](const LevMarq<double>::eVector & sol ,  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &sJ ){
        sJ.resize(p3d.size()*2,6);

        double rx=sol(0);
        double ry=sol(1);
        double rz=sol(2);
        double tx=sol(3);
        double ty=sol(4);
        double tz=sol(5);
        double nsqa=rx*rx + ry*ry + rz*rz;
        double a=std::sqrt(nsqa);
        double i_a=a?1./a:0;
        double rnx=rx*i_a;
        double rny=ry*i_a;
        double rnz=rz*i_a;
        double cos_a=cos(a);
        double sin_a=sin(a);
        double _1_cos_a=1.-cos_a;
        double rx2=rx*rx;
        double ry2=ry*ry;
        double rz2=rz*rz;

        uint32_t errIdx=0;

        for(size_t i=0;i<p3d.size();i++,errIdx+=2){

            if (vBadMatches[i]) {//do not use it
                for(int i=0;i< 6;i++) sJ(errIdx,i)=sJ(errIdx+1,i)=0;
                continue;
            }

            const double &X=p3d[i].x;
            const double &Y=p3d[i].y;
            const double &Z=p3d[i].z;



            //dx,dy wrt rx ry rz
            double q1=(nsqa*tz+rz*(rx*X+ry*Y+rz*Z)-(-nsqa*Z+rz*(rx*X+ry*Y+rz*Z))*cos_a+a*(-ry*X+rx*Y)*sin_a);
            double q2=(rx*X+ry*Y+rz*Z);
            double q3=(1/(nsqa*q1*q1));
            double q4=(-q2*rz-nsqa*tz+(q2*rz-nsqa*Z)*cos_a-a*(-ry*X+rx*Y)*sin_a);
            if (q1>1e-6){

                sJ(errIdx,0)=q3*fx*(q4*(2*q2*rx2-nsqa*(2*rx*X+ry*Y+rz*Z)+(-2*q2*rx2+nsqa*(ry*Y+rz*Z+rx*(2*X+rz*Y-ry*Z)))*cos_a+a*rx*(nsqa*X-rx2*X-rz*Y+ry*Z-rx*(ry*Y+rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
                sJ(errIdx,1)=q3*fx*(q4*(rx*(2*q2*ry-nsqa*Y)+(-2*rx2*ry*X+nsqa*ry*(rz*Y-ry*Z)+rx*(nsqa*Y-2*ry*(ry*Y+rz*Z)))*cos_a-a*(nsqa*(-ry*X+Z)+ry*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
                sJ(errIdx,2)=q3*fx*((q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*((-nsqa+2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(rx*(2*q2*rz-nsqa*Z)+(-2*rx2*rz*X+nsqa*rz*(rz*Y-ry*Z)+rx*(nsqa*Z-2*rz*(ry*Y+rz*Z)))*cos_a+a*(nsqa*(rz*X+Y)-rz*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a));
                sJ(errIdx+1,0)=q3*fy*(-q1*(ry*(2*q2*rx-nsqa*X)+(-2*q2*rx*ry+nsqa*(ry*X-rx*rz*X+rx2*Z))*cos_a+a*(rx*(-rx*ry*X+rz*X+nsqa*Y-ry2*Y)+(nsqa-rx*(rx+ry*rz))*Z)*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
                sJ(errIdx+1,1)=q3*fy*(-q1*(2*q2*ry2-nsqa*(rx*X+2*ry*Y+rz*Z)+(-2*q2*ry2+nsqa*(rx*X-ry*rz*X+2*ry*Y+rx*ry*Z+rz*Z))*cos_a+a*ry*(rz*X+nsqa*Y-ry2*Y-ry*rz*Z-rx*(ry*X+Z))*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
                sJ(errIdx+1,2)=q3*fy*((q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(-(nsqa-2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(ry*(2*q2*rz-nsqa*Z)+(rx*rz*(-2*ry*X+nsqa*Z)+nsqa*(-rz2*X+ry*Z)-2*ry*rz*(ry*Y+rz*Z))*cos_a-a*(nsqa*(X-rz*Y)+rz*(-rz*X+ry2*Y+ry*rz*Z+rx*(ry*X+Z)))*sin_a));
            }
            else{
                sJ(errIdx,0)=sJ(errIdx,1)=sJ(errIdx,2)=sJ(errIdx+1,0)=sJ(errIdx+1,1)=sJ(errIdx+1,2)=0;
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
            sJ(errIdx,3)= fx/a1;
            //dx_ty
            sJ(errIdx,4)=0;
            //dx_tz
            sJ(errIdx,5)= -((a8*fx)*_inv_a1_2);

            sJ(errIdx+1,3)=0;
            sJ(errIdx+1,4)= fy/a1;
            sJ(errIdx+1,5)= -((a7*fy)*_inv_a1_2);
        }

    };

    auto  calcDerivates2=[&](const LevMarq<double>::eVector & sol ,  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &sJ ){
        sJ.resize(p3d.size()*2,6);

        double rx=sol(0);
        double ry=sol(1);
        double rz=sol(2);
        double tx=sol(3);
        double ty=sol(4);
        double tz=sol(5);
        double nsqa=rx*rx + ry*ry + rz*rz;
        double a=std::sqrt(nsqa);
        double i_a=a?1./a:0;
        double rnx=rx*i_a;
        double rny=ry*i_a;
        double rnz=rz*i_a;
        double cos_a=cos(a);
        double sin_a=sin(a);
        double _1_cos_a=1.-cos_a;
        double rt_44[16];
        rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
        rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[3]=tx;
        rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt_44[5]=cos_a+rny*rny*_1_cos_a;
        rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
        rt_44[7]=ty;
        rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
        rt_44[11]=tz;

        uint32_t errIdx=0;

        for(size_t i=0;i<p3d.size();i++){
            if (vBadMatches[i]) {//do not use it
                for(int i=0;i< 6;i++) sJ(errIdx,i)=sJ(errIdx+1,i)=0;
                errIdx+=2;
                continue;
            }
            //move the point
            auto &pglob=p3d[i];

            double x=pglob.x*rt_44[0]+pglob.y*rt_44[1]+pglob.z*rt_44[2]+rt_44[3];
            double y=pglob.x*rt_44[4]+pglob.y*rt_44[5]+pglob.z*rt_44[6]+rt_44[7];
            double z=pglob.x*rt_44[8]+pglob.y*rt_44[9]+pglob.z*rt_44[10]+rt_44[11];
            const double z_2 = z*z;



            sJ(errIdx,0) =  x*y/z_2 *fx;
            sJ(errIdx,1)= -(1+(x*x/z_2)) *fx;
            sJ(errIdx,2) = y/z *fx;
            sJ(errIdx,3) = -1./z *fx;
            sJ(errIdx,4) = 0;
            sJ(errIdx,5) = x/z_2 *fx;
            errIdx++;
            sJ(errIdx,0) = (1+y*y/z_2) *fy;
            sJ(errIdx,1) = -x*y/z_2 *fy;
            sJ(errIdx,2) = -x/z *fy;
            sJ(errIdx,3) = 0;
            sJ(errIdx,4) = -1./z *fy;
            sJ(errIdx,5) = y/z_2 *fy;
            errIdx++;

        }

    };
    auto poseEstimationError=[&](const LevMarq<double>::eVector &sol,LevMarq<double>::eVector &errV){

        errV.resize(p3d.size()*2);
        jacobian.resize( p3d.size()*2,6);
        uint32_t errIdx=0;
        se3 pse3(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        Se3Transform T;
        T=pse3 ;
        for(size_t p=0;p<p3d.size();p++,errIdx+=2){
            cv::Point3f p3res=T*p3d[p];
            if (p3res.z> 0) {
                p3res.z=1./p3res.z;
                cv::Point2f p2dProj( p3res.x *fx *p3res.z + cx,p3res.y *fy *p3res.z + cy);
                cv::Point2f  err=p2d[p].pt-p2dProj;
                float robust_weight= getHubberMonoWeight((err.x*err.x+ err.y*err.y)*invScaleFactor[p2d[p].octave]  );
                 errV(errIdx)=robust_weight *err.x;
                errV(errIdx+1)=robust_weight*err.y;
            }
            else{
                errV(errIdx)=0;
                errV(errIdx+1)=0;
            }
        }

    };


    LevMarq<double>::eVector sol(6);
    for(int i=0;i<6;i++)  sol(i)=pose_io(i);
     Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> derv1,derv2;

    LevMarq<double> optimizer;
    optimizer.setParams(10,0.1,0.1,0.1);
    optimizer.verbose()=debug::Debug::getLevel()>=10;

    LevMarq<double>::eVector err;


       for(int it=0;it<5;it++){
        vBadMatches=std::vector<bool>(p3d.size(),false);
        optimizer.solve(sol,std::bind( poseEstimationError,std::placeholders::_1,std::placeholders::_2) ,std::bind(calcDerivates2,std::placeholders::_1,std::placeholders::_2));
        for(int i=0;i<6;i++)  pose_io(i)=sol(i);
        Se3Transform T;
        T=pose_io ;
        //now check the chi2 and find bad matches
        for(size_t p=0;p<p3d.size();p++){
            cv::Point3f p3res=T*p3d[p];
            if (p3res.z> 0) {
                p3res.z=1./p3res.z;
                cv::Point2f p2dProj( p3res.x *fx *p3res.z + cx,p3res.y *fy *p3res.z + cy);
                cv::Point2f  err=p2d[p].pt- p2dProj ;
                float chi2=  invScaleFactor[p2d[p].octave] * (err.x*err.x+ err.y*err.y);
                if (chi2>chi2Mono) vBadMatches[p]=true;
            }
            else vBadMatches[p]=true;
        }
    }

}

}
