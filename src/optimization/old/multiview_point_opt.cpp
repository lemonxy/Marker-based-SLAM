#include "multiview_point_opt.h"
#include "levmarq.h"
namespace ucoslam{

cv::Point3f  MultiViewPointOpt::optimize(cv::Point3f &p,const std::vector<cv::Point2f> &projs, const std::vector<se3> &poses,cv::Mat &CameraMatrix){


    auto error=[&](const LevMarq<float>::eVector &sol,LevMarq<float>::eVector &err){
        cv::Point3f p(sol(0),sol(1),sol(2));
        err.resize(projs.size()*2);
        int err_idx=0;
        for(int i=0;i<poses.size();i++){
            auto pi=project_and_derive(p,poses[i]);
            err(err_idx++)=projs[i].x-pi.x;
            err(err_idx++)=projs[i].y-pi.y;
        }
    };

    assert(CameraMatrix.type()==CV_32F);
            fx=CameraMatrix.at<float>(0,0);
    cx=CameraMatrix.at<float>(0,2);
    fy=CameraMatrix.at<float>(1,1);
    cy=CameraMatrix.at<float>(1,2);
    LevMarq<float> solver;
    solver.verbose()=true;
    solver.setParams(100,0.1,0.01,0.5);
    LevMarq<float>::eVector sol(3);
    sol(0)=p.x;sol(1)=p.y;sol(2)=p.z;

    solver.solve(sol,std::bind(error,std::placeholders::_1,std::placeholders::_2));

    p.x=sol(0);p.y=sol(1);p.z=sol(2);
    return cv::Point3f(sol(0),sol(1),sol(2));

}


MultiViewPointOpt::proj_info MultiViewPointOpt::project_and_derive(const cv::Point3f  &point, const se3 &rt){

    float rx=rt[0];
    float ry=rt[1];
    float rz=rt[2];
    float tx=rt[3];
    float ty=rt[4];
    float tz=rt[5];


    float nsqa=rx*rx + ry*ry + rz*rz;
    float a=std::sqrt(nsqa);
    float i_a=a?1./a:0;
    float rnx=rx*i_a;
    float rny=ry*i_a;
    float rnz=rz*i_a;
    float cos_a=cos(a);
    float sin_a=sin(a);
    float _1_cos_a=1.-cos_a;
    float r11=cos_a+rnx*rnx*_1_cos_a;
    float r12=rnx*rny*_1_cos_a- rnz*sin_a;
    float r13=rny*sin_a + rnx*rnz*_1_cos_a;
    float r21=rnz*sin_a +rnx*rny*_1_cos_a;
    float r22= cos_a+rny*rny*_1_cos_a;
    float r23= -rnx*sin_a+ rny*rnz*_1_cos_a;
    float r31= -rny*sin_a + rnx*rnz*_1_cos_a;
    float r32= rnx*sin_a + rny*rnz*_1_cos_a;
    float r33=cos_a+rnz*rnz*_1_cos_a;
    float _cos_a_1=cos_a-1;
    float rx2=rx*rx;
    float ry2=ry*ry;
    float rz2=rz*rz;


    proj_info pinf;
    const float &X=point.x;
    const float &Y=point.y;
    const float &Z=point.z;

    auto &pout=pinf;
    float Xc=X*r11 + Y*r12 + Z*r13 + tx;
    float Yc=X*r21 + Y*r22 + Z*r23 + ty;
    float Zc=X*r31+Y*r32 + Z*r33 +tz;
    pout.x= (Xc*fx/Zc)+cx;//projections
    pout.y= (Yc*fy/Zc)+cy;//projections


    //dx,dy wrt rx ry rz
    float q1=(nsqa*tz+rz*(rx*X+ry*Y+rz*Z)-(-nsqa*Z+rz*(rx*X+ry*Y+rz*Z))*cos_a+a*(-ry*X+rx*Y)*sin_a);
    float q2=(rx*X+ry*Y+rz*Z);
    float q3=(1/(nsqa*q1*q1));
    float q4=(-q2*rz-nsqa*tz+(q2*rz-nsqa*Z)*cos_a-a*(-ry*X+rx*Y)*sin_a);
    pout.dx[0]=q3*fx*(q4*(2*q2*rx2-nsqa*(2*rx*X+ry*Y+rz*Z)+(-2*q2*rx2+nsqa*(ry*Y+rz*Z+rx*(2*X+rz*Y-ry*Z)))*cos_a+a*rx*(nsqa*X-rx2*X-rz*Y+ry*Z-rx*(ry*Y+rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
    pout.dx[1]=q3*fx*(q4*(rx*(2*q2*ry-nsqa*Y)+(-2*rx2*ry*X+nsqa*ry*(rz*Y-ry*Z)+rx*(nsqa*Y-2*ry*(ry*Y+rz*Z)))*cos_a-a*(nsqa*(-ry*X+Z)+ry*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a)+(q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
    pout.dx[2]=q3*fx*((q2*rx+nsqa*tx+(-q2*rx+nsqa*X)*cos_a+a*(-rz*Y+ry*Z)*sin_a)*((-nsqa+2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(rx*(2*q2*rz-nsqa*Z)+(-2*rx2*rz*X+nsqa*rz*(rz*Y-ry*Z)+rx*(nsqa*Z-2*rz*(ry*Y+rz*Z)))*cos_a+a*(nsqa*(rz*X+Y)-rz*(rx2*X+rx*ry*Y+rz*Y-ry*Z+rx*rz*Z))*sin_a));
    pout.dy[0]=q3*fy*(-q1*(ry*(2*q2*rx-nsqa*X)+(-2*q2*rx*ry+nsqa*(ry*X-rx*rz*X+rx2*Z))*cos_a+a*(rx*(-rx*ry*X+rz*X+nsqa*Y-ry2*Y)+(nsqa-rx*(rx+ry*rz))*Z)*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*rx-nsqa*X)+(-2*q2*rx*rz+nsqa*(rx*ry*X+rz*X-rx2*Y))*cos_a+a*(-nsqa*Y-rx*(rx*rz*X-rx*Y+ry*(X+rz*Y)-nsqa*Z+rz2*Z))*sin_a));
    pout.dy[1]=q3*fy*(-q1*(2*q2*ry2-nsqa*(rx*X+2*ry*Y+rz*Z)+(-2*q2*ry2+nsqa*(rx*X-ry*rz*X+2*ry*Y+rx*ry*Z+rz*Z))*cos_a+a*ry*(rz*X+nsqa*Y-ry2*Y-ry*rz*Z-rx*(ry*X+Z))*sin_a)+(q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(rz*(2*q2*ry-nsqa*Y)+(-2*q2*ry*rz+nsqa*(ry2*X-rx*ry*Y+rz*Y))*cos_a+a*(nsqa*(X+ry*Z)-ry*(ry*X+rx*rz*X-rx*Y+ry*rz*Y+rz2*Z))*sin_a));
    pout.dy[2]=q3*fy*((q2*ry+nsqa*ty-(q2*ry-nsqa*Y)*cos_a+a*(rz*X-rx*Z)*sin_a)*(-(nsqa-2*rz2)*(rx*X+ry*Y)+2*rz*(-nsqa+rz2)*Z+(-2*q2*rz2+nsqa*(rx*X+ry*rz*X+ry*Y-rx*rz*Y+2*rz*Z))*cos_a+a*rz*(-rx*rz*X+rx*Y-ry*(X+rz*Y)+nsqa*Z-rz2*Z)*sin_a)-q1*(ry*(2*q2*rz-nsqa*Z)+(rx*rz*(-2*ry*X+nsqa*Z)+nsqa*(-rz2*X+ry*Z)-2*ry*rz*(ry*Y+rz*Z))*cos_a-a*(nsqa*(X-rz*Y)+rz*(-rz*X+ry2*Y+ry*rz*Z+rx*(ry*X+Z)))*sin_a));



    float a1=(tz+Z*(rnz*rnz* _1_cos_a+cos_a)+Y *(rny*rnz*_1_cos_a+rnx*sin_a)+X *(rnx*rnz*_1_cos_a-rny*sin_a)) ;
    float a2=rnx*rnx*_1_cos_a+cos_a;
    float a3=rnx*rnz*_1_cos_a;
    float a4=rny*sin_a;
    float a5=rnx*rny*_1_cos_a;
    float a6=rnz*Y*sin_a;
    float a7=(ty+a5*X+rny*rny*Y+rny*rnz*Z+(Y-rny*rny*Y-rny* rnz*Z)*cos_a+(rnz*X-rnx*Z)*sin_a);
    float a8=(-a6+tx+a2*X+a5*Y+(a3+a4)*Z);
    float _inv_a1_2=1./(a1*a1);




    //dx_tx
    pout.dx[3]= fx/a1;
    //dx_ty
    pout.dx[4]=0;
    //dx_tz
    pout.dx[5]= -((a8*fx)*_inv_a1_2);
    //dx_X_errVectorSize
    pout.dx[6]= ((a1*a2-(a3-a4)*a8)*fx)*_inv_a1_2;
    //dx_Y
    pout.dx[7]= (fx*(-a8*(rny*rnz-rny*rnz*cos_a+rnx*sin_a)+a1*(a5-rnz*sin_a)))*_inv_a1_2;
    //dx_Z
    pout.dx[8]= (fx*(a1*(a3+a4)-a8*(rnz*rnz+cos_a-rnz*rnz*cos_a))) *_inv_a1_2;

    pout.dy[3]=0;
    pout.dy[4]= fy/a1;
    pout.dy[5]= -((a7*fy)*_inv_a1_2);
    pout.dy[6]= (fy*(-(a3-a4)*a7+a1*(a5+rnz*sin_a))) *_inv_a1_2;
    pout.dy[7]= (fy*(a1*(rny*rny+cos_a-rny*rny* cos_a)-a7*(rny*rnz-rny*rnz *cos_a+rnx*sin_a)))*_inv_a1_2;
    pout.dy[8]= (fy*(-a7*(rnz*rnz+cos_a-rnz*rnz*cos_a)-a1* (rny*rnz*(_cos_a_1)+rnx*sin_a)))*_inv_a1_2;


    return pinf;
}

}
