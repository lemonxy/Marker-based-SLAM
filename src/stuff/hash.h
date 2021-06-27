#ifndef Hash_H_
#define Hash_H_
#include <cstdint>
#include <iostream>
#include <opencv2/core.hpp>
namespace ucoslam {
/**
 * @brief The Hash struct creates a hash by adding elements. It is used to check the integrity of data stored in files in debug mode
 */
struct Hash{
    uint64_t seed=0;

    template<typename T> void add(const T &val){
        char *p=(char *)&val;
        for(uint32_t b=0;b<sizeof(T);b++) seed  ^=  p[b]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    template<typename T> void add(const  T& begin,const T& end){
        for(auto it=begin;it!=end;it++)add(*it);
    }

    void add(bool val){
        seed  ^=   int(val)+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(int val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(uint64_t val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    void add(float val){
        int *p=(int *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(double val){
        uint64_t *p=(uint64_t *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    void add(const cv::Mat & m){
        for(int r=0;r<m.rows;r++){
            const char *ip=m.ptr<char>(r);
            int nem= m.elemSize()*m.cols;
            for(int i=0;i<nem;i++)
                seed  ^=   ip[i]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
    }

    void operator+=(bool v){add(v);}
    void operator+=(int v){add(v);}
    void operator+=(char v){add(v);}
    void operator+=(float v){add(v);}
    void operator+=(double v){add(v);}
    void operator+=(uint32_t v){add(v);}
    void operator+=(uint64_t v){add(v);}
    void operator+=(const cv::Mat & v){add(v);}


     operator uint64_t()const{return seed;}

    std::string tostring(){
        std::string sret;
        std::string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
        unsigned char * s=(uchar *)&seed;
        int n=sizeof(seed)/sizeof(uchar );
        for(int i=0;i<n;i++){
            sret.push_back(alpha[s[i]%alpha.size()]);
        }
        return sret;
    }
};

}

#endif

