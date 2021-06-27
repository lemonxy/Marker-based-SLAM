#ifndef _UCOSLAM_POINTPROJECTOR_H_
#define _UCOSLAM_POINTPROJECTOR_H_
#include <opencv2/core/core.hpp>
#include "stuff/se3.h"
#include "stuff/utils.h"
namespace ucoslam{

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


inline proj_info  project_and_derive(const cv::Point3f  &point, const se3 &rt,const cv::Mat &camera_params,bool calcDervXYZ=true) {
    assert(camera_params.type()==CV_32F);
    double fx=camera_params.at<float>(0,0);
    double cx=camera_params.at<float>(0,2);
    double fy=camera_params.at<float>(1,1);
    double cy=camera_params.at<float>(1,2);
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




inline vector<proj_info>  project_and_derive(const vector<cv::Point3f>  &points, const se3 &rt,const cv::Mat &camera_params,bool calcDervXYZ=true) {
    assert(camera_params.type()==CV_32F);
    double fx=camera_params.at<float>(0,0);
    double cx=camera_params.at<float>(0,2);
    double fy=camera_params.at<float>(1,1);
    double cy=camera_params.at<float>(1,2);
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

    vector<proj_info> vpinf(points.size());

    for(size_t i=0;i<points.size();i++){
        auto &pout=  vpinf[i];
        const auto &point=points[i];
        const double &X=point.x;
        const double &Y=point.y;
        const double &Z=point.z;

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
    }

    //check that results are identical to individual calls

//    for(size_t i=0;i<points.size();i++){
//        auto pinf2=project_and_derive(points[i],rt,camera_params,calcDervXYZ);
//        for(int d=0;d<6;d++){
//            assert( fabs(vpinf[i].dx[d]-pinf2.dx[d])<=1e-3 );
//            assert( fabs(vpinf[i].dy[d]-pinf2.dy[d])<=1e-3 );
//        }
//        if ( calcDervXYZ)
//            for(int d=6;d<9;d++){
//                assert( fabs(vpinf[i].dx[d]-pinf2.dx[d])<=1e-3 );
//                assert( fabs(vpinf[i].dy[d]-pinf2.dy[d])<=1e-3 );
//            }
//    }
    return vpinf;
}

//    inline proj_info  project_and_derive(const cv::Point3f  &point, const se3 &rt,const cv::Mat &camera_params) {
//        assert(camera_params.type()==CV_32F);
//        double fx=camera_params.at<float>(0,0);
//        double cx=camera_params.at<float>(0,2);
//        double fy=camera_params.at<float>(1,1);
//        double cy=camera_params.at<float>(1,2);

//        double rx=rt[0];
//        double ry=rt[1];
//        double rz=rt[2];
//        double tx=rt[3];
//        double ty=rt[4];
//        double tz=rt[5];


//        double nsqa=rx*rx + ry*ry + rz*rz;
//        double a=std::sqrt(nsqa);
//        double i_a=a?1./a:0;
//        double rnx=rx*i_a;
//        double rny=ry*i_a;
//        double rnz=rz*i_a;
//        double cos_a=cos(a);
//        double sin_a=sin(a);
//        double _1_cos_a=1.-cos_a;
//        double r11=cos_a+rnx*rnx*_1_cos_a;
//        double r12=rnx*rny*_1_cos_a- rnz*sin_a;
//        double r13=rny*sin_a + rnx*rnz*_1_cos_a;
//        double r21=rnz*sin_a +rnx*rny*_1_cos_a;
//        double r22= cos_a+rny*rny*_1_cos_a;
//        double r23= -rnx*sin_a+ rny*rnz*_1_cos_a;
//        double r31= -rny*sin_a + rnx*rnz*_1_cos_a;
//        double r32= rnx*sin_a + rny*rnz*_1_cos_a;
//        double r33=cos_a+rnz*rnz*_1_cos_a;
//        double _cos_a_1=cos_a-1;
//        double rx2=rx*rx;
//        double ry2=ry*ry;
//        double rz2=rz*rz;


//        proj_info pinf;
//        const double &X=point.x;
//        const double &Y=point.y;
//        const double &Z=point.z;

//        auto &pout=pinf;
//        double Xc=X*r11 + Y*r12 + Z*r13 + tx;
//        double Yc=X*r21 + Y*r22 + Z*r23 + ty;
//        double Zc=X*r31+Y*r32 + Z*r33 +tz;
//        pout.x= (Xc*fx/Zc)+cx;//projections
//        pout.y= (Yc*fy/Zc)+cy;//projections


//        //dx,dy wrt rx ry rz
//        double q1=(nsqa*tz+rz*(rx*X+ry*Y+rz*Z)-(-nsqa*Z+rz*(rx*X+ry*Y+rz*Z))*cos_a+a*(-ry*X+rx*Y)*sin_a);
//        double q2=(rx*X+ry*Y+rz*Z);
//        double q3=(1/(nsqa*q1*q1));
//        double q4=(-q2*rz-nsqa*tz+(q2*rz-nsqa*Z)*cos_a-a*(-ry*X+rx*Y)*sin_a);
//        pout.dx[0]=q3*fx*(q4*(2*q2*rx2-nsqa*(2*rx*X+ry*Y+rz*Z)+(-2*q2*rx2+nsqa*(ry*Y+rz*Z+rx*(2*X+rz*Y-ry*Z)))*cos_a+a*rx*(nsqa*X-rx2*X-rz*Y+ry*Z-rx*(ry*Y+rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
//        pout.dx[1]=q3*fx*(q4*(rx*(2*q2*ry-nsqa*Y)+(-2*rx2*ry*X+nsqa*ry*(rz*Y-ry*Z)+rx*(nsqa*Y-2*ry*(ry*Y+rz*Z)))*cos_a-a*(nsqa*(-ry*X+Z)+ry*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
//        pout.dx[2]=q3*fx*((q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*((-nsqa+2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(rx*(2*q2*rz-nsqa*Z)+(-2*rx2*rz*X+nsqa*rz*(rz*Y-ry*Z)+rx*(nsqa*Z-2*rz*(ry*Y+rz*Z)))*cos_a+a*(nsqa*(rz*X+Y)-rz*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a));
//        pout.dy[0]=q3*fy*(-q1*(ry*(2*q2*rx-nsqa*X)+(-2*q2*rx*ry+nsqa*(ry*X-rx*rz*X+rx2*Z))*cos_a+a*(rx*(-rx*ry*X+rz*X+nsqa*Y-ry2*Y)+(nsqa-rx*(rx+ry*rz))*Z)*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
//        pout.dy[1]=q3*fy*(-q1*(2*q2*ry2-nsqa*(rx*X+2*ry*Y+rz*Z)+(-2*q2*ry2+nsqa*(rx*X-ry*rz*X+2*ry*Y+rx*ry*Z+rz*Z))*cos_a+a*ry*(rz*X+nsqa*Y-ry2*Y-ry*rz*Z-rx*(ry*X+Z))*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
//        pout.dy[2]=q3*fy*((q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(-(nsqa-2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(ry*(2*q2*rz-nsqa*Z)+(rx*rz*(-2*ry*X+nsqa*Z)+nsqa*(-rz2*X+ry*Z)-2*ry*rz*(ry*Y+rz*Z))*cos_a-a*(nsqa*(X-rz*Y)+rz*(-rz*X+ry2*Y+ry*rz*Z+rx*(ry*X+Z)))*sin_a));



//        double a1=(tz+Z*(rnz*rnz* _1_cos_a+cos_a)+Y *(rny*rnz*_1_cos_a+rnx*sin_a)+X *(rnx*rnz*_1_cos_a-rny*sin_a)) ;
//        double a2=rnx*rnx*_1_cos_a+cos_a;
//        double a3=rnx*rnz*_1_cos_a;
//        double a4=rny*sin_a;
//        double a5=rnx*rny*_1_cos_a;
//        double a6=rnz*Y*sin_a;
//        double a7=(ty+a5*X+rny*rny*Y+rny*rnz*Z+(Y-rny*rny*Y-rny* rnz*Z)*cos_a+(rnz*X-rnx*Z)*sin_a);
//        double a8=(-a6+tx+a2*X+a5*Y+(a3+a4)*Z);
//        double _inv_a1_2=1./(a1*a1);




//        //dx_tx
//        pout.dx[3]= fx/a1;
//        //dx_ty
//        pout.dx[4]=0;
//        //dx_tz
//        pout.dx[5]= -((a8*fx)*_inv_a1_2);
//        //dx_X_errVectorSize
//        pout.dx[6]= ((a1*a2-(a3-a4)*a8)*fx)*_inv_a1_2;
//        //dx_Y
//        pout.dx[7]= (fx*(-a8*(rny*rnz-rny*rnz*cos_a+rnx*sin_a)+a1*(a5-rnz*sin_a)))*_inv_a1_2;
//        //dx_Z
//        pout.dx[8]= (fx*(a1*(a3+a4)-a8*(rnz*rnz+cos_a-rnz*rnz*cos_a))) *_inv_a1_2;

//        pout.dy[3]=0;
//        pout.dy[4]= fy/a1;
//        pout.dy[5]= -((a7*fy)*_inv_a1_2);
//        pout.dy[6]= (fy*(-(a3-a4)*a7+a1*(a5+rnz*sin_a))) *_inv_a1_2;
//        pout.dy[7]= (fy*(a1*(rny*rny+cos_a-rny*rny* cos_a)-a7*(rny*rnz-rny*rnz *cos_a+rnx*sin_a)))*_inv_a1_2;
//        pout.dy[8]= (fy*(-a7*(rnz*rnz+cos_a-rnz*rnz*cos_a)-a1* (rny*rnz*(_cos_a_1)+rnx*sin_a)))*_inv_a1_2;


//        return pout;
//    }

//    inline proj_info  project_and_derive_fast(const cv::Point3f  &point, const se3 &rt,const cv::Mat &camera_params,bool calcDervXYZ=true) {
//        //create the projection matrix

//        cv::Mat LPm=cv::Mat::eye(4,4,CV_32F);
//        LPm.at<float>(0,0)=camera_params.at<float>(0,0);
//        LPm.at<float>(0,2)=camera_params.at<float>(0,2);
//        LPm.at<float>(1,1)=camera_params.at<float>(1,1);
//        LPm.at<float>(1,2)=camera_params.at<float>(1,2);
//        cv::Mat K=LPm*rt.convert();//projection matrix
//        float* Kptr=K.ptr<float>(0);
//        auto p=fast__project__( Kptr,(float*)&point);
//        proj_info pi;
//        pi.x=p.x;
//        pi.y=p.y;
//        if (calcDervXYZ){
//            //now, dervxy wrt XYZ
//            float delta=1e-3;
//            float invdelta2=1./(2.f*delta);
//            cv::Point3f deltas[3]={cv::Point3f(delta,0,0),cv::Point3f(0,delta,0),cv::Point3f(0,0,delta)};
//            for(int d=0;d<3;d++){
//                auto pp=point+deltas[d];
//                auto pm=point-deltas[d];
//                auto pdif= fast__project__( Kptr,(float*)(&pp))-   fast__project__( Kptr,(float*)(&pm));
//                pi.dx[6+d]=pdif.x*invdelta2;
//                pi.dy[6+d]=pdif.y*invdelta2;
//            }
//        }
//        return pi;

//    }
inline proj_info  project_and_derive_cv(const cv::Point3f  &point, const se3 &rt,const cv::Mat &camera_params,bool calcDervXYZ=true) {

    vector<cv::Point3f> p3d(1);
    p3d[0]=point;

    vector<cv::Point2f > p2d;
    cv::Mat jac;
    cv::projectPoints(p3d,rt.getRvec(),rt.getTvec(),camera_params,cv::Mat (),p2d,jac);
    proj_info pi;
    pi.x=p2d[0].x;
    pi.y=p2d[0].y;
    for(int i=0;i<6;i++){
        pi.dx[i]= jac.at<double>(0,i);
        pi.dy[i]= jac.at<double>(1,i);
    }

    if (calcDervXYZ){
        cv::Mat LPm=cv::Mat::eye(4,4,CV_32F);
        LPm.at<float>(0,0)=camera_params.at<float>(0,0);
        LPm.at<float>(0,2)=camera_params.at<float>(0,2);
        LPm.at<float>(1,1)=camera_params.at<float>(1,1);
        LPm.at<float>(1,2)=camera_params.at<float>(1,2);
        cv::Mat K=LPm*rt.convert();//projection matrix
        float* Kptr=K.ptr<float>(0);

        float delta=1e-3;
        float invdelta2=1./(2.f*delta);
        cv::Point3f deltas[3]={cv::Point3f(delta,0,0),cv::Point3f(0,delta,0),cv::Point3f(0,0,delta)};
        for(int d=0;d<3;d++){
            auto pp=point+deltas[d];
            auto pm=point-deltas[d];
            auto pdif= fast__project__( Kptr,(float*)(&pp))-   fast__project__( Kptr,(float*)(&pm));
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

inline cv::Point2f project(const cv::Point3f &p3d,const cv::Mat &cameraMatrix,const cv::Mat &RT,const cv::Mat & Distortion=cv::Mat()){
    assert(cameraMatrix.type()==CV_32F);
    assert(RT.type()==CV_32F);
    assert( (Distortion.type()==CV_32F &&  Distortion.total()>=5 )|| Distortion.total()==0);

    const float *cm=cameraMatrix.ptr<float>(0);
    const float *rt=RT.ptr<float>(0);
    const float *k=0;
    if ( Distortion.total()!=0) k=Distortion.ptr<float>(0);

    //project point first
    float x= p3d.x* rt[0]+p3d.y* rt[1]+p3d.z* rt[2]+rt[3];
    float y= p3d.x* rt[4]+p3d.y* rt[5]+p3d.z* rt[6]+rt[7];
    float z= p3d.x* rt[8]+p3d.y* rt[9]+p3d.z* rt[10]+rt[11];

    float xx=x/z;
    float yy=y/z;

    if (k!=0){//apply distortion //        k1,k2,p1,p2[,k3
        float r2=xx*xx+yy*yy;
        float r4=r2*r2;
        float comm=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
        float xxx = xx * comm + 2*k[2] *xx*yy+ k[3]*(r2+2*xx*xx);
        float yyy= yy*comm+ k[2]*(r2+2*yy*yy)+2*k[3]*xx*yy;
        xx=xxx;
        yy=yyy;
    }
    return cv::Point2f((xx*cm[0])+cm[2],(yy*cm[4])+cm[5] );
}
}


//inline  Point2f projectNoDistortion(const  Point3f &p3d,const CameraParams &cameraParams,const Se3Transform &RT){

//    const float *rt=RT.data;

//    //project point first
//    float x= p3d.x* rt[0]+p3d.y* rt[1]+p3d.z* rt[2]+rt[3];
//    float y= p3d.x* rt[4]+p3d.y* rt[5]+p3d.z* rt[6]+rt[7];
//    float z= p3d.x* rt[8]+p3d.y* rt[9]+p3d.z* rt[10]+rt[11];

//    float xx=x/z;
//    float yy=y/z;

//    if (cameraParams.hasDistortion()){//apply distortion //        k1,k2,p1,p2[,k3
//        const float *k=cameraParams.dist;
//        float xx2=xx*xx;
//        float yy2=yy*yy;
//        float r2=xx2+yy2;
//        float r4=r2*r2;
//        float comm=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
//        float xxx = xx * comm + 2*k[2] *xx2+ k[3]*(r2+2*xx2);
//        float yyy= yy*comm+ k[2]*(r2+2*yy2)+2*k[3]*xx*yy;
//        xx=xxx;
//        yy=yyy;
//    }
//        else if(Distortion.total()==8){// k1,k2,p1,p2,k3,k4,k5,k6
//            const float *k=Distortion.ptr<float>(0);
//            float xx2=xx*xx;
//            float yy2=yy*yy;
//            float r2=xx2+yy2;
//            float r4=r2*r2;
//            float num=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
//            float den=1+k[5]*r2+k[6]*r4+k[7]*(r4*r2);
//            float comm=num/den;
//            float xxx = xx * comm + 2*k[2] *xx2+ k[3]*(r2+2*xx2);
//            float yyy= yy*comm+ k[2]*(r2+2*yy2)+2*k[3]*xx*yy;
//            xx=xxx;
//            yy=yyy;
//        }
//    return  Point2f((xx*cameraParams.fx)+cameraParams.cx,(yy*cameraParams.fy)+cameraParams.cy );
//}




#endif
