#ifndef ucoslam_Se3Transform_H
#define ucoslam_Se3Transform_H
#include "se3.h"
namespace ucoslam{
class Se3Transform:public cv::Mat{
public:
    Se3Transform(){
        create(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                if (i==j) at<float>(i,j)=1;
                else at<float>(i,j)=0;
    }
    Se3Transform(const Se3Transform&R){
        create(4,4,CV_32F);
        memcpy(ptr<float>(0),R.ptr<float>(0),16*sizeof(float));
    }

    void setUnity(){
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                if (i==j) at<float>(i,j)=1;
                else at<float>(i,j)=0;
    }

     cv::Point3f getCenter()const{
        return cv::Point3f(at<float>(0,3),at<float>(1,3),at<float>(2,3));
    }
    inline Se3Transform operator=(const cv::Mat &m){
        assert(m.type()==CV_32F);
        assert(m.cols ==4 && m.rows==4);
        memcpy(ptr<float>(0),m.ptr<float>(0),16*sizeof(float));
        return *this;
    }
    inline Se3Transform & operator=(const Se3Transform &R){
        memcpy(ptr<float>(0),R.ptr<float>(0),16*sizeof(float));
        return *this;
    }

    cv::Mat getTvec()const{
        cv::Mat t(1,3,CV_32F);
        t.ptr<float>(0)[0]=at<float>(0,3);
        t.ptr<float>(0)[1]=at<float>(1,3);
        t.ptr<float>(0)[2]=at<float>(2,3);
        return t;
    }
    cv::Mat getRvec()const{
        cv::Mat Rvec;
        cv::Rodrigues ( rowRange(0,3).colRange(0,3),Rvec );
        return Rvec;
    }

    inline float &operator[](uint32_t idx) {assert(idx<16); return ptr<float>()[idx];}
    inline float operator[](uint32_t idx)const {assert(idx<16); return ptr<float>()[idx];}
    inline float at_(uint32_t idx)const{assert(idx<16); return ptr<float>()[idx];}
    Se3Transform inv()const{

        Se3Transform  Minv;
        Minv[0]=at_(0);
        Minv[1]=at_(4);
        Minv[2]=at_(8);
        Minv[4]=at_(1);
        Minv[5]=at_(5);
        Minv[6]=at_(9);
        Minv[8]=at_(2);
        Minv[9]=at_(6);
        Minv[10]=at_(10);

        Minv[3] = -  ( at_(3)*Minv[0]+at_(7)*Minv[1]+at_(11)*Minv[2]);
        Minv[7] = -  ( at_(3)*Minv[4]+at_(7)*Minv[5]+at_(11)*Minv[6]);
        Minv[11]= -  ( at_(3)*Minv[8]+at_(7)*Minv[9]+at_(11)*Minv[10]);
        Minv[12]=Minv[13]=Minv[14]=0;
        Minv[15]=1;
        return Minv;
    }

    inline cv::Point3f operator*(const cv::Point3f &p)const{
        const float *_ptr=ptr<float> ( 0 );
        return cv::Point3f(  _ptr[0]*p.x +_ptr[1]*p.y +_ptr[2]*p.z+_ptr[3],_ptr[4]*p.x +_ptr[5]*p.y +_ptr[6]*p.z+_ptr[7],_ptr[8]*p.x +_ptr[9]*p.y +_ptr[10]*p.z+_ptr[11]);
    }


    Se3Transform & operator=(const se3&rt){
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
        float *rt_44=ptr<float>(0);
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
        rt_44[12]=rt_44[13]=rt_44[14]=0;
        rt_44[15]=1;
        return *this;
    }

    void toStream(std::ostream &str)const{
        uint32_t sig=928511272;
        str.write((char*)&sig, sizeof(sig));
        str.write((char*)ptr<float>(),16*sizeof(float));
    }
    void fromStream(std::istream &str){
        uint32_t sig;
        str.read((char*)&sig, sizeof(sig));
        if(sig!=928511272) throw std::runtime_error("magic number eror in Se3Transform::fromStream");
        str.read((char*)ptr<float>(),16*sizeof(float));
    }
    float *data(){return ptr<float>(0);}
};

}
#endif
