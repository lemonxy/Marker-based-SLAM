#include "stuff/ferns.h"
namespace ucoslam{

FernTest::FernTest(){}
FernTest::FernTest(cv::Mat &image,cv::Point a,cv::Point b,int thres){setParams(image,a,b,thres);}
void FernTest::setParams(cv::Mat &image,cv::Point a,cv::Point b,int thres){
    //computes the offset of a line
    assert(image.type()==CV_8UC1);
    uint64_t lineSizeBytes=image.ptr<uchar>(1)- image.ptr<uchar>(0);
    off1=  a.x+a.y*lineSizeBytes;
    off2=  b.x+b.y*lineSizeBytes;
    Thres=thres;
}

void Fern::set(cv::Point center){pt=center;}
void Fern::add(cv::Mat &ims,cv::Point a,cv::Point b,int thres){
    assert(FernTests.size()<=32);
    FernTests.push_back(FernTest(ims,a,b,thres));
}


void Fern::toStream(std::ostream &str)const{

    str.write((char*)&pt,sizeof(pt));
    uint32_t s=FernTests.size();
    str.write((char*)&s,sizeof(s));
    str.write((char*)&FernTests[0],FernTests.size()*sizeof(FernTests[0]));

}
void Fern::fromStream(std::istream &str){
    str.read((char*)&pt,sizeof(pt));
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    FernTests.resize(s);
    str.read((char*)&FernTests[0],FernTests.size()*sizeof(FernTests[0]));
}

void FernSet::clear(){
    Ferns.clear();
}

void  FernSet::create(cv::Size ims,int nFerns,int nFernTests,int maxTestdistance){
    _imageSize=ims;
    Ferns.clear();//selected locations  to compute the ferns
    //set the possible points to be evaluated
   std:: vector<cv::Point> offsets;
    for(int x=-maxTestdistance;x<=maxTestdistance;x++)
        for(int y=-maxTestdistance;y<=maxTestdistance;y++)
            offsets.push_back(cv::Point(x,y));
    //now,create the ferns
    std::uniform_int_distribution <int> x_generator(maxTestdistance,ims.width-maxTestdistance);
    std::uniform_int_distribution <int> y_generator(maxTestdistance,ims.height-maxTestdistance);
    std::uniform_int_distribution <int> thres_generator(-250,250);
    std::random_device r;

    cv::Mat dummyImage(ims,CV_8UC1);
    for(int i=0;i<nFerns;i++){
        std::random_shuffle(offsets.begin(),offsets.end());
        Ferns.push_back(Fern());
        auto &fern=Ferns.back();
        fern.set(cv::Point(x_generator(r),y_generator(r)));//set center
        for(int j=0;j<nFernTests;j++)//add tests around
            fern.add(dummyImage,offsets[0],offsets[1],thres_generator(r));

    }
}

void FernSet::toStream(std::ostream &str)const{

    str.write((char*)&_imageSize,sizeof(_imageSize));
    uint32_t s=Ferns.size();
    str.write((char*)&s,sizeof(s));
    for(auto &f:Ferns)f.toStream(str);
}

void FernSet::fromStream(std::istream &str){
    str.read((char*)&_imageSize,sizeof(_imageSize));
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    Ferns.resize(s);
    for(auto &f:Ferns)f.fromStream(str);
}

void FernKeyFrameDataBase::setParams(float nFernsAsImageAreaPercentage, int nFernTests, int maxTestdistance){
    _nFernsAsImageAreaPercentage=nFernsAsImageAreaPercentage;
    _nFernTests=nFernTests;
    _maxTestdistance=maxTestdistance;
}

void FernKeyFrameDataBase::clear(){
    _Fset.clear();
    map_id_code.clear();
    _nFernsAsImageAreaPercentage=0.06;
    _nFernTests=4;
    _maxTestdistance=2;


}

void FernKeyFrameDataBase::add(cv::Mat &image,uint32_t id){
    if(_Fset.size()==0)
        _Fset.create(image.size(),_nFernsAsImageAreaPercentage*float(image.cols*image.rows),_nFernTests,_maxTestdistance);
    //first, compute
    auto codes=_Fset(image);
    map_id_code[id]=codes;
 }

void FernKeyFrameDataBase::del(uint32_t id){
    assert(map_id_code.count(id));
    map_id_code.erase(id);
}
std::vector<int> FernKeyFrameDataBase::transform(const cv::Mat &image){
    if(_Fset.size()==0)
        _Fset.create(image.size(),_nFernsAsImageAreaPercentage*float(image.cols*image.rows),_nFernTests,_maxTestdistance);
    return _Fset(image);
}

float FernKeyFrameDataBase::FernDist(const std::vector<int> &a,const std::vector<int> &b){
//how many similitudes are there
    int sim=0;
    for(size_t i=0;i<a.size();i++)
        if (a[i]==b[i])sim++;
    return 1. - float(sim)/float(a.size());
}


 std::vector<std::pair<uint32_t,float> > FernKeyFrameDataBase::query(const std::vector<int> &code,bool sorted){

if (map_id_code.size()==0)return{};

     //distLimit
     int maxDist=float(map_id_code.begin()->second.size()*_nFernTests)*0.1;
     assert(maxDist>=1);
    std::vector<std::pair<uint32_t,float> > result;
    result.reserve(map_id_code.size());
    for(auto &midc:map_id_code){
        auto hamm=FernKeyFrameDataBase::HammDist(code,midc.second);
        if (hamm>maxDist)hamm=maxDist;
        float score= 1.f- float(hamm)/maxDist;
        result.push_back({midc.first,score });
    }

    //sort
     if (sorted)
         std::sort(result.begin(),result.end(),[](const std::pair<uint32_t,float> &a,const std::pair<uint32_t,float> &b){return a.second>b.second;});
    return result;
}

 void FernKeyFrameDataBase::toStream(std::ostream &str)const{

     auto   v2Stream=[]( const  std::vector<int> &v,std::ostream &str ) {
         uint32_t s=v.size();
         str.write ( ( char* ) &s,sizeof ( s) );
         str.write (  (char*)&v[0],sizeof(v[0])*v.size());
     };

     str.write((char*)&_nFernsAsImageAreaPercentage,sizeof(_nFernsAsImageAreaPercentage));
     str.write((char*)&_nFernTests,sizeof(_nFernTests));
     str.write((char*)&_maxTestdistance,sizeof(_maxTestdistance));
    _Fset.toStream(str);
    //now, the map
    size_t s=map_id_code.size();
    str.write((char*)&s,sizeof(s));
    for(auto &k_v:map_id_code){
        str.write((char*)&k_v.first,sizeof(k_v.first));
        v2Stream(k_v.second,str);
    }
 }

 void FernKeyFrameDataBase::fromStream(std::istream &str) {

     auto fromStream=[] (  std::vector<int> &v,std::istream &str ) {
         uint32_t s;
         str.read( ( char* ) &s,sizeof ( s) );
         v.resize(s);
          str.read(  (char*)&v[0],sizeof(v[0])*v.size());
     };
     str.read((char*)&_nFernsAsImageAreaPercentage,sizeof(_nFernsAsImageAreaPercentage));
     str.read((char*)&_nFernTests,sizeof(_nFernTests));
     str.read((char*)&_maxTestdistance,sizeof(_maxTestdistance));
    _Fset.fromStream(str);
    //now, the map
    size_t s;
    str.read((char*)&s,sizeof(s));
    map_id_code.clear();
    for(size_t i=0;i<s;i++){
        std::pair<uint32_t,std::vector<int> > k_v;
        str.read((char*)&k_v.first,sizeof(k_v.first));
        fromStream(k_v.second,str);
        map_id_code.insert(k_v);
    }
 }
}
