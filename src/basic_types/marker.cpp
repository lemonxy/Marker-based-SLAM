#include "marker.h"
#include "stuff/utils.h"
using namespace std;
namespace ucoslam{



std::vector<cv::Point3f> Marker::get3DPoints(bool global_ref)const{
    assert(size>0);
    vector<cv::Point3f> marker_points = { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };
    cv::Mat m44=pose_g2m;
    if (global_ref){
        for(auto &p:marker_points)
            p=mult<float>(m44,p);
    }
    return marker_points;
}

void Marker::toStream(std::ostream &str)const {
    str.write((char*)&id,sizeof(id));
    pose_g2m.toStream(str);
    str.write((char*)&size,sizeof(size));
    toStream__(frames,str);
    toStream__(dict_info,str);

}
void Marker::fromStream(std::istream &str) {
    str.read((char*)&id,sizeof(id));
    pose_g2m.fromStream(str);
    str.read((char*)&size,sizeof(size));
    fromStream__(frames,str);
    fromStream__(dict_info,str);

}

std::vector<cv::Point3f> Marker::get3DPointsLocalRefSystem( float size ){
 return { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };

}


vector<cv::Point3f> Marker::get3DPoints(se3 pose_g2m,float size,bool global_ref){
    vector<cv::Point3f> marker_points = { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };
    cv::Mat m44=pose_g2m;
    if (global_ref){
        for(auto &p:marker_points)
            p=mult<float>(m44,p);
    }
    return marker_points;
}

}
