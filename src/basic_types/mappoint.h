#ifndef ucoslam_MapPoint_H
#define ucoslam_MapPoint_H
#include "ucoslam_exports.h"
#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <cmath>

#include <mutex>
namespace ucoslam {

class Frame;
/**Represents a 3d point of the environment
 */
class UCOSLAM_API MapPoint
{
 public:
    MapPoint();
    MapPoint(const MapPoint &mp);
    MapPoint & operator=(const MapPoint &mp);

    cv::Point3f getCoordinates(void) ;
    void setCoordinates(const cv::Point3f &p);
    cv::Point3f  getNormal(void) ;

    float getMinDistanceInvariance();
    float getMaxDistanceInvariance();
    int predictScale( float  currentDist, float mfLogScaleFactor,int mnScaleLevels);

    //---------------------
    bool isValid()const{return _descRefIdx>=0 && !std::isnan(normal.x);}

    /////////////////////////////////////////////////////////////////////
    void addKeyFrameObservation(const Frame & frame, uint32_t kptIdx);

    void setSeen() ;
    void setVisible(uint64_t fseqId );
    int geNTimesVisible( );
    float getTimesUnseen();
    float getVisibility();


    void toStream(std::ostream &str) ;
    void fromStream(std::istream &str)  ;
    bool operator==(const MapPoint &p)const;

    uint64_t getSignature()const;



    //returns the distance between two descriptors
    static   float getDescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2);
    float getDescDistance( const cv::Mat &dsc2);
    float getDescDistance( MapPoint &mp);
    //gets a copy of the descriptor in the passed mat
   void getDescriptor( cv::Mat &copy);
    //used to reescale the whole map
    void scalePoint(float scaleFactor);

    bool isObservingFrame(uint32_t fidx);
    std::vector<std::pair<uint32_t,uint32_t> > getObservingFrames();
    uint32_t getNumOfObservingFrames();



    //returns a pointer to the extra space
    template<typename T> T* getExtra(uint32_t index=0){return (T*)(&extra_1[0]);}

    void updateNormals(const std::vector<cv::Point3f> &normals);
    void updateBestObservation(const Frame &frame, uint32_t kptIdx);

    float getConfidence();//returns a confidence value for the point according to the number of times it has been seen

    float  getViewCos(const cv::Point3f &camCenter) ;



    uint32_t id=std::numeric_limits<uint32_t>::max();
    cv::Point3f pose=cv::Point3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());//rotation and translation of the point. The translation is the 3d location. The rotation refers to the normal of the patch
    std::map<uint32_t,uint32_t> frames;// Frames in which the point is projected. Key: frame_idx, Value: index in ids of the keypoint where it projects
    bool isStable=false;//indicates if it is an stable point that need no further recheck
    uint32_t creationSeqIdx=std::numeric_limits<uint32_t>::max();//frame index sequence  in which it was created
    bool isBad=false;//excludes this point from optimization
    uint64_t kfSinceAddition=0;
    uint64_t lastTimeSeenFSEQID=0;
    bool isStereo=false;

private:

    cv::Mat _desc; //! Set of descriptors (each row) for the map point
    int _descRefIdx=-1;            //! Index (row) of the reference descriptor, for matching
    cv::Point3f normal=cv::Point3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN()); //! Mean viewing direction
    std::vector<cv::Point3f> vNormals; //! Whole set of viewing directions
    int minOctaveObservation=std::numeric_limits<int>::max();//to save
    uint32_t nTimesSeen=0,nTimesVisible=0;
    float mfMaxDistance,mfMinDistance;
    //extra space for usage
    uint32_t extra_1[128];

    void _addDescriptor(const cv::Mat & desc);

    std::mutex DescMutex,CoordMutex,VisMutex;
};

}

#endif
