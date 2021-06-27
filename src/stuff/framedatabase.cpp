#include "stuff/framedatabase.h"
#include "stuff/io_utils.h"
#include "stuff/timers.h"
namespace ucoslam{

void DummyDataBase::toStream_(iostream &str)const{
toStream__(frames,str);
}

void DummyDataBase::fromStream_(std::istream &str){
    fromStream__(frames,str);

}
uint64_t DummyDataBase::getSignature()const{
    Hash sig;
    for(auto f:frames) sig+=f;
    return sig;

}


vector<uint32_t> FernsFrameDataBase::relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted, float minScore, const std::set<uint32_t> &excludedFrames){
    ScopedTimerEvents Timer("FernsFrameDataBase::relocalizationCandidates");
    //first compute distances to all frames
    vector<int> code=db.transform(frame.smallImage);
    auto frame_score_=db.query(code,true);

    Timer.add("query db");
    std::vector< pair<uint32_t,double> >   frame_score;
    double bestAccScore=0;
    for(auto fs:frame_score_){
        if (!excludedFrames.count(fs.first))
            if ( fs.second>minScore){
              frame_score.push_back(fs);
              if (fs.second>bestAccScore) bestAccScore=fs.second;
            }
        if( frame_score.size()>=20) break;
    }
    Timer.add("select best candidates");


    Timer.add("covis voting");
    // Return all those keyframes with a score higher than 0.75*bestScore
    double minScoreToRetain = 0.75f*bestAccScore;
    frame_score.erase(    std::remove_if(frame_score.begin(),frame_score.end(),[&](const pair<uint32_t,double> &v){return v.second<minScoreToRetain;}),
                               frame_score.end());

    Timer.add("step4");

    //copy remaining to vector and return
    vector<uint32_t> candidates;candidates.reserve(frame_score.size());
    for(const auto &fs:frame_score)candidates.push_back(fs.first);
    return candidates;


}
uint64_t FernsFrameDataBase::getSignature()const{
    stringstream sstr;
    db.toStream(sstr);
    uint64_t seed=0;
    while(!sstr.eof())
        seed^= sstr.get()+  0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}

KFDataBase::KFDataBase(){
     _impl=std::make_shared<DummyDataBase>();
    _type=0;
}


void KFDataBase::loadFromFile(std::string str){
}

void KFDataBase::toStream(iostream &str)const
{
    str.write((char*)&_type,sizeof(_type));

    _impl->toStream_(str);
}
void KFDataBase::fromStream(std::istream &str)
{
    str.read((char*)&_type,sizeof(_type));
    if (_type==0)
        _impl=std::make_shared<DummyDataBase>();
    else
        _impl=std::make_shared<FernsFrameDataBase>();

    _impl->fromStream_(str);
}


}

