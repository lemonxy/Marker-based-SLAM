#ifndef ucoslam_Marker_H
#define ucoslam_Marker_H
#include <opencv2/core.hpp>
#include <vector>
#include "stuff/se3.h"
#include <set>
namespace ucoslam {

/**A marker
 */

class Marker{
public:
    Marker(){}
    Marker(uint32_t Id,se3 g2m,float Size):id(Id),pose_g2m(g2m),size(Size){}
    uint32_t id;//id
    se3 pose_g2m;//pose  Marker -> Global
    float size=0;//marker size
    std::set<uint32_t> frames;//key frames in which the marker is visible
    std::string dict_info; //information about the dictionary it belongs to

    //returns the 3d points of the marker in the global_ref
    std::vector<cv::Point3f> get3DPoints(bool global_ref=true)const;
    static std::vector<cv::Point3f> get3DPoints(se3 pose_g2m, float size, bool global_ref=true);
    //returns the 3d points in the local reference system
    static std::vector<cv::Point3f> get3DPointsLocalRefSystem( float size );
    //---------------------
    //serialization routines
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;

};
}
#endif
