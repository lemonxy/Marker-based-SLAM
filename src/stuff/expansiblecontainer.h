#ifndef _ExpansibleContainer_H
#define _ExpansibleContainer_H
#include <vector>
#include <cstdint>
#include <iostream>
#include <cassert>
namespace ucoslam{

/** A container that can grow, without moving existing data from its location in memory
 **/

template<typename T >
class ExpansibleContainer{
public:
    ExpansibleContainer(int chunkSize=200):_chunkSize(chunkSize){
        vbuffer.push_back(new std::vector<T>(_chunkSize));
        curElm=curBuffer=0;
    }

    ~ExpansibleContainer(){
        for(auto ptr:vbuffer)delete ptr;
        vbuffer.clear();
    }

    inline const T & at(uint32_t idx)const{
        assert(idx<vbuffer.size()*_chunkSize);
        int cb=idx/_chunkSize;
        int ce=idx-cb*_chunkSize;
        return vbuffer[cb]->at(ce);
    };

    inline T & at(uint32_t idx){
        assert(idx<vbuffer.size()*_chunkSize);
        int cb=idx/_chunkSize;
        int ce=idx-cb*_chunkSize;
        return vbuffer[cb]->at(ce);
    };

    inline size_t size()const{return curBuffer*_chunkSize+curElm;}
    inline size_t capacity()const{return _chunkSize*vbuffer.size();}

    inline void clear(){
        assert(vbuffer.size()>0);
        //remove all but first
        for(size_t i=0;i<vbuffer.size();i++)
            delete vbuffer[i];
        vbuffer.clear();
        vbuffer.push_back(new std::vector<T>(_chunkSize));
        curElm=curBuffer=0;
    }

    inline  const T&operator[](uint32_t idx)const{return at(idx);}
    inline   T&operator[](uint32_t idx){return at(idx);}

    inline void push_back(const T &v){
        if ( curElm==_chunkSize){
            if( curBuffer==int(vbuffer.size())-1) //need a new buffer
                vbuffer.push_back(new std::vector<T>(_chunkSize));
              curElm=0;
              curBuffer++;
        }
        (*vbuffer[curBuffer])[curElm++]=v;
    }

    inline void pop_back(){
        if ( curElm==0){
            if ( curBuffer>0) {
                curBuffer--;
                curElm=_chunkSize;
            }
            else{

            }

        }
        else{
           curElm--;
        }

    }


    inline void reset(){
        curBuffer=curElm=0;
    }

    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

private:
    std::vector< std::vector<T>* > vbuffer;
    int curBuffer,curElm;
    int _chunkSize;
};

template<typename T >
void ExpansibleContainer<T>::toStream(std::ostream &str)const{
    uint64_t sig=13218888;
    str.write((char*)&sig,sizeof(sig));
    uint64_t s=vbuffer.size();
    str.write((char*)&s,sizeof(s));
    for(auto ptr:vbuffer){
        for(int i=0;i<_chunkSize;i++)
            (*ptr)[i].toStream(str);
    }

    str.write((char*)&curBuffer,sizeof(curBuffer));
    str.write((char*)&curElm,sizeof(curElm));
    str.write((char*)&_chunkSize,sizeof(_chunkSize));

}
template<typename T >
void ExpansibleContainer<T>::fromStream(std::istream &str){
    uint64_t sig;
    str.read((char*)&sig,sizeof(sig));
    if (sig!= 13218888) throw std::runtime_error("ExpansibleContainer::fromStream invalid signature ");

    uint64_t s;
    str.read((char*)&s,sizeof(s));
    vbuffer.clear();
    vbuffer.resize(s);
    for(auto &ptr:vbuffer){
        ptr=new std::vector<T>(_chunkSize);
        for(int i=0;i<_chunkSize;i++)
            (*ptr)[i].fromStream(str);
    }

    str.read((char*)&curBuffer,sizeof(curBuffer));
    str.read((char*)&curElm,sizeof(curElm));
    str.read((char*)&_chunkSize,sizeof(_chunkSize));

}
};
#endif
