#ifndef UCOSLAM_Ferns_H
#define UCOSLAM_Ferns_H
#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <bitset>

namespace ucoslam{
//a binary test using differential Fern Method
struct FernTest{//computes the
    int off1,off2, Thres;

    FernTest();
    FernTest(cv::Mat &image, cv::Point a, cv::Point b, int thres);
    void setParams(cv::Mat &ims,cv::Point a,cv::Point b,int thres);

    inline bool compute(const cv::Mat &image,cv::Point &pt)const{
        const uchar *center=image.ptr<uchar>(pt.y)+pt.x;
        return (*(center+off1) - *(center+off2) )>Thres;
    }

};

//a Differential Fern
struct Fern{
    cv::Point pt;
    std::vector<FernTest> FernTests;
    void set(cv::Point center);
    void add(cv::Mat &ims,cv::Point a,cv::Point b,int thres);

    inline int operator()(const cv::Mat &image){
        int fern=0;
        for(size_t i=0;i<FernTests.size();i++){
            fern|=FernTests[i].compute(image,pt);
            if ( i<FernTests.size()-1)
                fern=fern<<1;
        }
//        fern=fern>>1;
        return fern;
    }

//    inline void operator()(const cv::Mat &image,uint8_t *fern_ptr,uint32_t fernbytes){
//        int fern=0;
//        for(size_t i=0;i<FernTests.size();i++){
//            fern|=FernTests[i].compute(image,pt);
//            if ( i<FernTests.size()-1)
//                fern=fern<<1;
//        }
//        //now, copy data to pointer
//        memcpy(fern_ptr,&fern,fernbytes);
////        fern=fern>>1;
//        return fern;
//    }

    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
};

struct FernCode{
    ~FernCode(){
        if (_data!=0) free(_data);
    }

//use 64 bit aligned data
    void alloc(int nbitsPerFern,int nferns){
        if (_data!=0) free(_data);
        _bytesPerFern=bytesPerFern(nbitsPerFern);
        int bytes=_bytesPerFern*nferns;
        int finalsize_aligned=bytes/aligment;
        if ( _bytesPerFern%aligment!=0) finalsize_aligned++;
        _sizebytes=finalsize_aligned*aligment;
        _data=(uint64_t*)aligned_alloc(aligment,_sizebytes);
        _alignedwords=_sizebytes/aligment;
        memset(_data,0,_sizebytes);
    }
    //returns pointer to the i-th fern
    inline char *operator[](uint32_t i){
        return ((char*)_data)+ i*_bytesPerFern;
    }

    //return the hamming distance
    inline uint64_t distance(const FernCode &fc){
      uint64_t hamm=0;
      for(uint32_t i=0;i<_alignedwords;i++)
            hamm+=std::bitset<64>(_data[i]^fc._data[i]).count();
      return hamm;
    }

private:
    inline int bytesPerFern(int bitsPerFern){
        int bytes=bitsPerFern/8;
        if(bitsPerFern%8!=0) bytes++;
        return bytes;
    }

    uint64_t *_data=0;
    uint32_t _sizebytes,_alignedwords,_bytesPerFern;
    const int aligment=8;


};

//A set of Ferns
struct FernSet{
    std::vector<Fern> Ferns;//selected locations  to compute the ferns
    cv::Size _imageSize;
    void create(cv::Size ims, int nFerns, int nFernTests, int maxTestdistance);
     inline std::vector<int> operator()(const cv::Mat &image){
        assert(image.size()==_imageSize);
        assert(image.type()==CV_8UC1);
        std::vector<int> codes(Ferns.size());
        for(size_t i=0;i< Ferns.size();i++)
            codes[i]=Ferns[i](image);
        return codes;
    }

    size_t size()const{return Ferns.size();}
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
    void clear();
};

class FernKeyFrameDataBase{
    public:
    float _nFernsAsImageAreaPercentage=0.06,_nFernTests=4,_maxTestdistance=2;
    FernSet _Fset;
    std::map<uint32_t,std::vector<int> > map_id_code;//for each element, its codes
 public:
    void setParams(float nFernsAsImageAreaPercentage, int nFernTests=4, int maxTestdistance=2);
    //adds a keyframe to the database
    //image must have the desired size(small) and be 8UC1
    void add(cv::Mat &image,uint32_t id);
    inline void add(const std::vector<int>&code,uint32_t id){map_id_code[id]=code;}
    //deletes the keyframe indicated
    void del(uint32_t id);
    //query the database and returns a sorted  vector of pairs id-distance.
    std::vector<std::pair<uint32_t,float> >  query(const std::vector<int> &code,bool sorted);

    void clear();
    //number of frames in the database
    size_t size()const{return map_id_code.size();}

    //is the frame of id indicated?
    bool isId(uint32_t id)const {return map_id_code.count(id);}

    std::vector<int> transform(const cv::Mat &image);
    inline float score(const std::vector<int> &a,const std::vector<int> &b)const{
        int maxDist=float(map_id_code.begin()->second.size()*_nFernTests)*0.1;
        auto hamm=FernKeyFrameDataBase::HammDist(a,b);
        if (hamm>maxDist)hamm=maxDist;
        return 1.f- float(hamm)/maxDist;
    }


    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;

private:
    static float FernDist(const std::vector<int> &a,const std::vector<int> &b);
    inline float HammDist(const std::vector<int> &a,const std::vector<int> &b)const{
        uint64_t hamm=0;
        for(size_t i=0;i<a.size();i++)
            hamm+= std::bitset<32>(a[i]^b[i]).count();
        return hamm;
    }
};
}
#endif
