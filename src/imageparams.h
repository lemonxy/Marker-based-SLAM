#ifndef ucoslam_ImageParmas_H
#define ucoslam_ImageParmas_H
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco/cameraparameters.h"
#include "stuff/io_utils.h"
#include "stuff/hash.h"
namespace ucoslam{
class  ImageParams:public aruco::CameraParameters {
public:

    inline float fx()const{return  CameraMatrix.ptr<float>(0)[0];}
    inline float cx()const{return  CameraMatrix.ptr<float>(0)[2];}
    inline float fy()const{return  CameraMatrix.ptr<float>(0)[4];}
    inline float cy()const{return  CameraMatrix.ptr<float>(0)[5];}
    inline float k1()const{return (Distorsion.total()>=1?Distorsion.ptr<float>(0)[0]:0);}
    inline float k2()const{return (Distorsion.total()>=2? Distorsion.ptr<float>(0)[1]:0);}
    inline float p1()const{return (Distorsion.total()>=3? Distorsion.ptr<float>(0)[2]:0);}
    inline float p2()const{return (Distorsion.total()>=4? Distorsion.ptr<float>(0)[3]:0);}
    inline float k3()const{return (Distorsion.total()>=5?  Distorsion.ptr<float>(0)[4]:0);}



    inline bool isIntoImage(cv::Point2f &p2){
        if ( p2.x>=0 && p2.y>=0 && p2.x<CamSize.width &&  p2.y<CamSize.height) return true;
        return false;
    }

    inline void toStream(std::ostream &str) const ;
    inline void fromStream(std::istream &str) ;

     //apply distortion to a undistorted point
   inline cv::Point2f distortPoint(const cv::Point2f &p) const;
   inline  std::vector<cv::Point2f> distortPoints(const std::vector<cv::Point2f> &p) const;


   //returns a version of this without distortion
    ImageParams undistorted()const{
        ImageParams ip=*this;
        ip.Distorsion.setTo(cv::Scalar::all(0));
        return ip;
    }

    void clear(){
        CameraMatrix=cv::Mat();
        Distorsion=cv::Mat();
        CamSize=cv::Size(-1,-1);
    }

    float bl=0;//stereo camera base line
    float rgb_depthscale=0;//scale to obtain depth from the rgbd values
    bool isStereoCamera()const{return bl!=0;}
    //indicates if a depth point is close or far
    inline bool isClosePoint(float z)const{
        return z<40*bl;
    }

    inline uint64_t getSignature()const;

    friend std::ostream &operator<<(std::ostream &str,const ImageParams &ip){str<<ip.CameraMatrix<<std::endl<<ip.Distorsion<<std::endl<<ip.CamSize<<std::endl<<"bl="<<ip.bl<<" rgb_depthscale="<<ip.rgb_depthscale<<std::endl;return str;}

 };

void ImageParams::toStream(std::ostream &str) const {

    toStream__(CameraMatrix,str);
    toStream__(Distorsion,str);
    str.write((char*)&CamSize,sizeof(CamSize));
    str.write((char*)&bl,sizeof(bl));
    str.write((char*)&rgb_depthscale,sizeof(rgb_depthscale));

}

void ImageParams::fromStream(std::istream &str) {
    fromStream__(CameraMatrix,str);
    fromStream__(Distorsion,str);
    str.read((char*)&CamSize,sizeof(CamSize));
    str.read((char*)&bl,sizeof(bl));
    str.read((char*)&rgb_depthscale,sizeof(rgb_depthscale));

}
std::vector<cv::Point2f> ImageParams::distortPoints(const std::vector<cv::Point2f> &vp) const{

    //for each point, obtain a 3d location in the ray from which it comes from, and then project back the point using distortion parameters

    //obaint the normalized coordinates

    float fx=CameraMatrix.at<float>(0,0);
    float fy=CameraMatrix.at<float>(1,1);
    float cx=CameraMatrix.at<float>(0,2);
    float cy=CameraMatrix.at<float>(1,2);
    std::vector<cv::Point3f> vp3d(vp.size());
    for(size_t i=0;i<vp.size();i++){
        vp3d[i].x= (vp[i].x- cx)/fx;
        vp3d[i].y= (vp[i].y- cy)/fy;
        vp3d[i].z=1;
    }


    //now, project back
    std::vector<cv::Point2f> und;
    cv::projectPoints(vp3d,cv::Mat::zeros(1,3,CV_32F),cv::Mat::zeros(1,3,CV_32F),CameraMatrix, Distorsion,und);
    return und ;
}

cv::Point2f ImageParams::distortPoint(const cv::Point2f &p)const{

    std::vector<cv::Point2f> vp(1);vp[0]=p;
    return distortPoints(vp)[0];

}
uint64_t ImageParams::getSignature()const{
    Hash Sig;

    for(uint64_t i=0;i<CameraMatrix.total();i++) Sig.add(CameraMatrix.ptr<float>(0)[i]);
    for(uint64_t i=0;i<Distorsion.total();i++) Sig.add(Distorsion.ptr<float>(0)[i]);
    Sig.add(CamSize);
    Sig.add(bl);
    Sig.add(rgb_depthscale);
    return Sig;
}

}
#endif
