#include "mappoint.h"
#include "stuff/io_utils.h"
#include "stuff/hash.h"
#include <bitset>
#include "frame.h"

using namespace std;
namespace ucoslam {

template<typename T>
bool are_ranges_equal(T it1_start,T it1_end,T it2_start,T it2_end){
    auto i1=it1_start;
    auto i2=it2_start;
    while( i1!=it1_end && i2!=it2_end){
        if( !(*i1==*i2) )return false;
        i1++;i2++;
    }
    if (i1!=it1_end || i2!=it2_end)return false;
    return true;
}

MapPoint::MapPoint(){
    memset(extra_1,0,sizeof(extra_1));
}

MapPoint::MapPoint(const MapPoint &mp){
    memset(extra_1,0,sizeof(extra_1));
    *this=mp;
}

MapPoint & MapPoint::operator=(const MapPoint &mp){
    id=mp.id;
    pose=mp.pose;
    frames=mp.frames;
    isStable=mp.isStable;
    creationSeqIdx=mp.creationSeqIdx;
    isBad=mp.isBad;
    _desc=mp._desc.clone();
    _descRefIdx=mp._descRefIdx;
    normal=mp.normal;
    vNormals=mp.vNormals;
    nTimesSeen=mp.nTimesSeen;
    nTimesVisible=mp.nTimesVisible;
    mfMaxDistance=mp.mfMaxDistance;
    mfMinDistance=mp.mfMinDistance;
    isStereo=mp.isStereo;
     memcpy(extra_1,mp.extra_1,sizeof(extra_1));
   return *this;


}

cv::Point3f MapPoint::getCoordinates(void)
{
    std::unique_lock<mutex> lock(CoordMutex);
    cv::Point3f val=pose;
    return val;
}

void MapPoint::setCoordinates(const cv::Point3f &p) {
    unique_lock<mutex> lock(CoordMutex);
    pose=p;
}

cv::Point3f  MapPoint::getNormal(void)  {
    unique_lock<mutex> lock(CoordMutex);
    return normal;
}

 bool MapPoint::operator==(const MapPoint &p)const{
    if(id!=p.id) return false;
    if (!are_ranges_equal( frames.begin(),frames.end(), p.frames.begin(),p.frames.end())) return false;
    if (pose!=p.pose) return false;
    if (_descRefIdx!=p._descRefIdx)return false;
    if (normal!=p.normal)return false;
    if (!are_ranges_equal( vNormals.begin(),vNormals.end(), p.vNormals.begin(),p.vNormals.end())) return false;
    if (_desc.total()!=p._desc.total())return false;
    bool isEqual = (cv::sum(_desc!= p._desc) == cv::Scalar(0,0,0,0));
    if (!isEqual)return false;
    return true;


}
 uint64_t MapPoint::getSignature()const{
     Hash sig;
     sig.add(frames.begin(),frames.end());
     sig.add(pose);
     sig.add(_descRefIdx);
     sig.add(normal);
     sig.add(vNormals.begin(),vNormals.end());
     sig+=_desc;
     return sig;
 }


void MapPoint::fromStream(std::istream &str){
    unique_lock<mutex> lock(CoordMutex);
    unique_lock<mutex> lock2(DescMutex);
    unique_lock<mutex> lock3(VisMutex);
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if (magic!=-123199) throw std::runtime_error("Error in MapPoint::fromSteream");


    str.read((char*)&id,sizeof(id));
    str.read((char*)&creationSeqIdx,sizeof(creationSeqIdx));

    str.read((char*)&pose,sizeof(pose));
    fromStream__(_desc,str);
    str.read((char*)&_descRefIdx,sizeof(_descRefIdx));
    fromStream__kv(frames,str);

    str.read((char*)&normal,sizeof(normal));
    fromStream__(vNormals,str);
    str.read((char*)&extra_1[0],sizeof(extra_1));


    str.read((char*)&nTimesSeen,sizeof(nTimesSeen));
    str.read((char*)&nTimesVisible,sizeof(nTimesVisible));
    str.read((char*)&isStable,sizeof(isStable));
    str.read((char*)&isBad,sizeof(isBad));
    str.read((char*)&mfMaxDistance,sizeof(mfMaxDistance));
    str.read((char*)&mfMinDistance,sizeof(mfMinDistance));
    str.read((char*)&minOctaveObservation,sizeof(minOctaveObservation));
    str.read((char*)&kfSinceAddition,sizeof(kfSinceAddition));
    str.read((char*)&lastTimeSeenFSEQID,sizeof(lastTimeSeenFSEQID));
    str.read((char*)&isStereo,sizeof(isStereo));


}

void MapPoint::toStream(std::ostream &str){

    unique_lock<mutex> lock(CoordMutex);
    unique_lock<mutex> lock2(DescMutex);
    unique_lock<mutex> lock3(VisMutex);
    //let us write a magic number to avoid reading errors
    int magic=-123199;
    str.write((char*)&magic,sizeof(magic));


    str.write((char*)&id,sizeof(id));
    str.write((char*)&creationSeqIdx,sizeof(creationSeqIdx));
    str.write((char*)&pose,sizeof(pose));
    toStream__(_desc,str);
    str.write((char*)&_descRefIdx,sizeof(_descRefIdx));
    toStream__kv(frames,str);

    str.write((char*)&normal,sizeof(normal));
    toStream__(vNormals,str);

    str.write((char*)&extra_1[0],sizeof(extra_1));

    //
    str.write((char*)&nTimesSeen,sizeof(nTimesSeen));
    str.write((char*)&nTimesVisible,sizeof(nTimesVisible));
    str.write((char*)&isStable,sizeof(isStable));
    str.write((char*)&isBad,sizeof(isBad));
    str.write((char*)&mfMaxDistance,sizeof(mfMaxDistance));
    str.write((char*)&mfMinDistance,sizeof(mfMinDistance));
    str.write((char*)&minOctaveObservation,sizeof(minOctaveObservation));
    str.write((char*)&kfSinceAddition,sizeof(kfSinceAddition));
    str.write((char*)&lastTimeSeenFSEQID,sizeof(lastTimeSeenFSEQID));
    str.write((char*)&isStereo,sizeof(isStereo));


}
void MapPoint::scalePoint(float scaleFactor){
    unique_lock<mutex> lock(CoordMutex);
    unique_lock<mutex> lock2(DescMutex);
    pose*=scaleFactor;
    mfMaxDistance *= scaleFactor;
    mfMinDistance *= scaleFactor;
}

float MapPoint::getDescDistance( MapPoint &mp){
    unique_lock<mutex> lock(DescMutex);
    unique_lock<mutex> lock2(mp.DescMutex);
    return getDescDistance(_desc.row(_descRefIdx) ,mp._desc.row(mp._descRefIdx));
}

void MapPoint::getDescriptor(cv::Mat &copy)
{
    unique_lock<mutex> lock(DescMutex);
    _desc.row(_descRefIdx).copyTo(copy);
}

float MapPoint::getDescDistance( const cv::Mat &dsc2){
    unique_lock<mutex> lock(DescMutex);
    return getDescDistance(_desc.row(_descRefIdx),dsc2);
}

float getHammDescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2){
    assert(dsc1.type()==dsc2.type());
    assert(dsc1.rows==dsc2.rows  && dsc2.rows   == 1);
    assert(dsc1.cols==dsc2.cols);
    assert(dsc1.type()==CV_8UC1);
    //   assert(dsc1.total()%8==0);
    // if ( dsc1.total()==32);
    int n8= dsc1.total()/8;
    int hamm=0;
    const uint64_t *ptr1=dsc1.ptr<uint64_t>(0);
    const uint64_t *ptr2=dsc2.ptr<uint64_t>(0);
    for(int i=0;i<n8;i++)
        hamm+=std::bitset<64>(ptr1[i] ^ ptr2[i]).count();

    int extra= dsc1.total() - n8*8;
    if (extra==0) return hamm;

    const uint8_t *uptr1=(uint8_t*) dsc1.ptr<uint64_t>(0)+n8;
    const uint8_t *uptr2=(uint8_t*) dsc2.ptr<uint64_t>(0)+n8;
    for(int i=0;i<extra;i++)
        hamm+=std::bitset<8>(uptr1[i] ^ uptr2[i]).count();

    //finally, the rest
    return hamm;

}
float getL2DescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2){
return cv::norm(dsc1-dsc2);
}
//returns the distance between two descriptors
float MapPoint::getDescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2){
    if (dsc1.type()==CV_8UC1) return getHammDescDistance(dsc1,dsc2);
    else if (dsc1.type()==CV_32F) return getL2DescDistance(dsc1,dsc2);
    throw std::runtime_error("Invalid descriptor");

}
float MapPoint::getMinDistanceInvariance()  {
    unique_lock<mutex> lock(DescMutex);
    return 0.8f*mfMinDistance;
}
float MapPoint::getMaxDistanceInvariance()  {
    unique_lock<mutex> lock(DescMutex);
    return 1.2f*mfMaxDistance;
}
int MapPoint::predictScale( float  currentDist, float mfLogScaleFactor,int mnScaleLevels){
    unique_lock<mutex> lock(DescMutex);
    int nScale = ceil(log(mfMaxDistance/currentDist)/ mfLogScaleFactor);
    if(nScale<0) return 0;
    if(nScale>= mnScaleLevels) return mnScaleLevels-1;
    return nScale;
}

bool MapPoint::isObservingFrame(uint32_t fidx){
    unique_lock<mutex> lock(DescMutex);
    return frames.count(fidx)!=0;
}

vector<pair<uint32_t,uint32_t> > MapPoint::getObservingFrames(){
    unique_lock<mutex> lock(DescMutex);
    vector<pair<uint32_t,uint32_t> > res;res.reserve(frames.size());
    for(auto f:frames)
        res.push_back(f);
    return res;
}
uint32_t MapPoint::getNumOfObservingFrames(){
    unique_lock<mutex> lock(DescMutex);
    return frames.size();
}



void MapPoint::addKeyFrameObservation(const Frame & frame,uint32_t kptIdx){
    assert( creationSeqIdx!=std::numeric_limits<uint32_t>::max());
    assert(!isnan(pose.x));
    unique_lock<mutex> lock(DescMutex);

    //now, select the finner view of the keypoint to select the view limits


    if (minOctaveObservation> frame.und_kpts[kptIdx].octave )
        updateBestObservation(frame,kptIdx);

    assert(frames.count(frame.idx)==0);//there should be no other instace of the point projected in the same frame
    frames.insert(std::make_pair(frame.idx,kptIdx));


    _addDescriptor(frame.desc.row(kptIdx));

    //add viewing dir
    //normalize viewing dir and add to list
    cv::Point3f vdir=frame.getCameraCenter()-  getCoordinates() ;
    vNormals.push_back(vdir/cv::norm(vdir) );

    // Update mean vector
    normal=cv::Point3f(0,0,0);
    for(auto v:vNormals) normal+=v;
    //normalize
    normal*=1./float(vNormals.size());

}

void MapPoint::updateNormals(const vector<cv::Point3f> &normals){
    vNormals=normals;
    //ensure unity
    for(auto &n:vNormals) n*=1./cv::norm(n);
    // Update mean vector
    normal=cv::Point3f(0,0,0);
    for(auto v:vNormals) normal+=v;
    //normalize
    normal*=1./float(vNormals.size());
}

void MapPoint::updateBestObservation(const Frame &frame, uint32_t kptIdx){
        minOctaveObservation= frame.und_kpts[kptIdx].octave;
        const float dist = cv::norm(pose - frame.getCameraCenter());
        const float levelScaleFactor =  frame.scaleFactors[frame.und_kpts[kptIdx].octave];
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/frame.scaleFactors.back();
}

void MapPoint::_addDescriptor(const cv::Mat &desc)
{
   if (_descRefIdx < 0) // Empty
   {
      _desc = desc.clone();
      _descRefIdx = 0;
   }
   else
   {
       cv::vconcat(_desc, desc, _desc);
         //method of Mur-Artal (the one that minimizes distance to all others)
       //float dists[_desc.rows][_desc.rows];
       float * dists = new float[_desc.rows * _desc.rows];
       memset(dists ,0,sizeof(float)*_desc.rows * _desc.rows);
       //compute all distances between the descriptors
       for (int i = 0; i<_desc.rows; i++) {
           for (int j = i + 1; j<_desc.rows; j++) {
               dists[i*_desc.rows + j] = getDescDistance(_desc.row(i), _desc.row(j));
               dists[j*_desc.rows + i] = dists[i*_desc.rows + j];
           }
       }
       //take the one that minimizes the sum of  distances to all
       float minD = std::numeric_limits<float>::max();
       _descRefIdx = -1;
       for (int i = 0; i<_desc.rows; i++) {
           float sumd = 0;
           for (int j = 0; j<_desc.rows; j++) sumd += dists[i*_desc.rows + j];
           if (sumd<minD) {
               minD = sumd;
               _descRefIdx = i;
           }
       }
       delete[] dists;
   }


}
//------------------------------------------------------


void MapPoint::setSeen(){
    unique_lock<mutex> lock(VisMutex);
    if(nTimesSeen<std::numeric_limits<uint32_t>::max())
        nTimesSeen++;
}
void MapPoint::setVisible(uint64_t fseqId){
    unique_lock<mutex> lock(VisMutex);
    lastTimeSeenFSEQID= fseqId;
    if(nTimesVisible<std::numeric_limits<uint32_t>::max())
        nTimesVisible++;
}
int MapPoint::geNTimesVisible( ){
    unique_lock<mutex> lock(VisMutex);
    return nTimesVisible;
}
float MapPoint::getTimesUnseen() {
    unique_lock<mutex> lock(VisMutex);
    if (nTimesVisible==0)return 0;
    return float(nTimesVisible-nTimesSeen)/float(nTimesVisible);
}

float MapPoint::getVisibility() {
    unique_lock<mutex> lock(VisMutex);
    if (nTimesVisible==0)return 0;
    return float(nTimesSeen)/float(nTimesVisible);
}
float MapPoint::getViewCos(const cv::Point3f &camCenter) {
    auto v= camCenter -  pose;
    v/=cv::norm(v);

    return v.dot(normal);
}

float MapPoint::getConfidence(){
return 1.;
    unique_lock<mutex> lock(VisMutex);
    if( nTimesSeen>=10) return 1;
    else if( nTimesSeen>=6) return 0.75;
    else if( nTimesSeen>=3) return 0.5;
    else return 0.25;
}
}
