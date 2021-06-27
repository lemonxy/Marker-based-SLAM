#ifndef UCOSLAM_FrameDataBase_H
#define UCOSLAM_FrameDataBase_H
#include <map>
#include "basic_types/frame.h"
#include "stuff/covisgraph.h"
#include "stuff/ferns.h"
namespace ucoslam{
class Slam;
class KFDataBaseVirtual{
public:
    virtual  bool isEmpty()const=0;
    virtual bool add(  Frame &f)=0;
    virtual bool del(const Frame &f)=0;
    virtual void clear( )=0;
    virtual vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={})=0;
    virtual uint64_t getSignature()const=0;
    virtual bool isId(uint32_t id)const=0;
     virtual void toStream_(std::iostream &str)const =0;
    virtual void fromStream_(std::istream &str)=0;
    virtual void loadFromFile(const std::string &filename)=0 ;
    virtual size_t size()const=0;
    virtual float score(Frame &f,Frame &f2) =0;


};

class FernsFrameDataBase:public KFDataBaseVirtual{
    FernKeyFrameDataBase db;
public:
    bool add(Frame &f){db.add(f.smallImage,f.idx);return true;}
    bool del(const Frame &f){db.del(f.idx);return true;}
    bool del(uint32_t &f){db.del(f);return true;}
    bool isEmpty()const{return db.size()==0;}
    void clear(){db.clear();}
    size_t size()const{return db.size();}
    vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={});
    void toStream_(std::iostream &str)const {db.toStream(str);}
    void fromStream_(std::istream &str){db.fromStream(str);}
    bool isId(uint32_t id)const {return  db.isId(id);}
    virtual void loadFromFile(const std::string &filename) {assert(false);throw std::runtime_error("FernsFrameDataBase::loadFromFile not implemented");} ;
    uint64_t getSignature()const;
    float score(Frame &f,Frame &f2)  {
        if (f.fernCode.empty()) f.fernCode=db.transform(f.smallImage);
        if (f2.fernCode.empty()) f2.fernCode=db.transform(f2.smallImage);
        return db.score(f.fernCode,f2.fernCode);
    }

};


class DummyDataBase:public KFDataBaseVirtual{
    std::set<uint32_t> frames;
 public:
    bool add(Frame &f){frames.insert(f.idx); return true;}
    bool del(const Frame &f){frames.erase(f.idx);return true;}
    bool del(uint32_t &f){frames.erase(f);return true;}
    bool isEmpty()const{return frames.size()==0;}
    void clear(){frames.clear();}
    size_t size()const{return frames.size();}
    vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={}){
        return {};
    }
    void toStream_(std::iostream &str)const;
    void fromStream_(std::istream &str);
    bool isId(uint32_t id)const {return  frames.count(id)!=0;}
    virtual void loadFromFile(const std::string &filename) {assert(false);throw std::runtime_error("DummyDataBase::loadFromFile not implemented");} ;
    uint64_t getSignature()const;
    float score(Frame &f,Frame &f2)  {
        return 0;
    }

};

class KFDataBase{
public:
    KFDataBase();
    void loadFromFile(string filePathOrNothing);
    bool isEmpty()const{return _impl->isEmpty();}
    bool isId(uint32_t id)const {return _impl->isId(id);}

    bool add(Frame &f){return _impl->add(f);}
    bool del(const Frame &f){return _impl->del(f);}
    void clear(){_impl->clear();}

    size_t size()const{return _impl->size();}
    void toStream(std::iostream &str)const ;
    void fromStream(std::istream &str);
    vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={}){
        return _impl->relocalizationCandidates(frame,fset,covisgraph,sorted,minScore,excludedFrames);
    }
    float score(Frame &f,Frame &f2)  {return _impl->score(f,f2); }

private:
    std::shared_ptr<KFDataBaseVirtual> _impl;
    int _type=-1;
};
}
#endif
