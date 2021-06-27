#include "frame.h"
#include "stuff/hash.h"
namespace ucoslam {
void Frame::copyTo( Frame &f)const{
    f.idx=idx;
    f.markers=markers;
    f.und_markers=und_markers;
    f.markers_solutions=markers_solutions;
    f.keypoint_kdtree=keypoint_kdtree;
    desc.copyTo(f.desc);
    f.ids=ids;

    f.nonMaxima=nonMaxima;
    pose_f2g.copyTo(f.pose_f2g);
    f.und_kpts=und_kpts;
    f.kpts=kpts;
    f.depth=depth;

    image.copyTo(f.image);
    smallImage.copyTo(f.smallImage);
     f.fernCode=fernCode;
    f.fseq_idx=fseq_idx;
    f. scaleFactors= scaleFactors;
    f.imageParams=imageParams;//camera with which it was taken
    f.isBad=isBad;
    memcpy(f.extra_1,extra_1,sizeof(extra_1));
 }


Frame & Frame::operator=(const Frame &f){
    f.copyTo(*this);
    return *this;
}


void Frame::nonMaximaSuppresion(){
        //for each assigned element, remove these around
        for(size_t i=0;i<ids.size();i++){
            if (ids[i]!=std::numeric_limits<uint32_t>::max()){
                //find points around
                auto kpts=getKeyPointsInRegion(und_kpts[i].pt, scaleFactors[ und_kpts[i].octave]*2.5,und_kpts[i].octave-1,und_kpts[i].octave);
                //these unassigned, remove them
                for(auto k:kpts)
                    if ( ids[k]==std::numeric_limits<uint32_t>::max())
                        nonMaxima[k]=true;
            }
        }

        //now, suppress non maxima in octaves

        for(size_t i=0;i<ids.size();i++){

            if (ids[i]==std::numeric_limits<uint32_t>::max() && !nonMaxima[i]){
                auto kpts=getKeyPointsInRegion(und_kpts[i].pt, scaleFactors[ und_kpts[i].octave]*2.5,und_kpts[i].octave-1,und_kpts[i].octave);
                //get the one with maximum response

                pair<uint32_t ,float> best(std::numeric_limits<uint32_t>::max(),0);
                for(auto k:kpts){
                    if ( !nonMaxima[k])
                        if (best.second<und_kpts[k].response)
                            best={k,und_kpts[k].response};
                }
                assert(best.first!=std::numeric_limits<uint32_t>::max());
                //set invalid all but the best
                for(auto k:kpts)
                    if(  k!=best.first)
                        nonMaxima[k]=true;

            }
        }
}

std::vector<uint32_t> Frame::getKeyPointsInRegion(cv::Point2f p, float radius ,  int minScaleLevel,int maxScaleLevel)const{

    cv::KeyPoint kp;
    kp.pt=p;
    auto  idx_dist=keypoint_kdtree.radiusSearch(und_kpts,kp,radius,false);

    std::vector<uint32_t> ret;ret.reserve(idx_dist.size());
    //find and mark these that are not in scale limits
    for( auto &id:idx_dist)
        if ( und_kpts[id.first].octave>=minScaleLevel && und_kpts[id.first].octave<=maxScaleLevel)
            ret.push_back(id.first);
    return ret;

}

int Frame::getMarkerIndex(uint32_t id)const  {
    for(size_t i=0;i<und_markers.size();i++)
        if (uint32_t(und_markers[i].id)==id)return i;
    return -1;

}

aruco::Marker Frame::getMarker(uint32_t id) const{
    for(const auto &m:und_markers)
        if (uint32_t(m.id)==id)return m;
    throw std::runtime_error("Frame::getMarker Could not find the required marker");
}
 MarkerPosesIPPE Frame::getMarkerPoseIPPE(uint32_t id)const
{
    for(size_t i=0;i<und_markers.size();i++)
        if (uint32_t(und_markers[i].id)==id) return markers_solutions[i];
    throw std::runtime_error("Frame::getMarkerPoseIPPE Could not find the required marker");

}


cv::Point3f Frame::getCameraCenter()const{
     return pose_f2g.inv()*cv::Point3f(0,0,0);  //obtain camera center in global reference system
 }

  cv::Point3f Frame::getCameraDirection()const{
        Se3Transform mPose=pose_f2g.inv();
        auto v0=mPose*cv::Point3f(0,0,0);  //obtain camera center in global reference system
        auto v1=mPose*cv::Point3f(0,0,1);
        auto vd=v1-v0;
        return vd/cv::norm(vd);
  };

  uint64_t  __getSignature(const vector<cv::KeyPoint> &kp,const cv::Mat &desc){
    uint64_t sig=0;
    for(const cv::KeyPoint &p:kp)
        sig+=p.pt.x*1000+p.pt.y*1000+p.angle*100+p.octave*100+p.response*100;
    //now, descriptor
    int nem= desc.elemSize()*desc.cols/sizeof(char);
    for(int r=0;r<desc.rows;r++)
        for(int i=0;i<nem;i++) sig+=desc.ptr<char>(r)[i];
    return sig;

  }
  vector<uint32_t> Frame::getMapPoints()const{
      vector<uint32_t> res;res.reserve(this->und_kpts.size());
      for(auto id:this->ids)
          if (id!=std::numeric_limits<uint32_t>::max())
              res.push_back(id);
      return res;
  }

  vector<uint32_t> Frame::getIdOfPointsInRegion( cv::Point2f  p,float radius){

      cv::KeyPoint kp;kp.pt=p;
      auto v_id_dist=keypoint_kdtree.radiusSearch( und_kpts,kp,radius,false);
      vector<uint32_t> ret;ret.reserve(v_id_dist.size());
      for(auto id_dist:v_id_dist){
          if ( ids[id_dist.first]!=std::numeric_limits<uint32_t>::max())
              ret.push_back(ids[id_dist.first]);
      }
      return ret;
  }

  uint64_t Frame::getSignature()const{

      Hash hash;

      hash+=idx;
      for(auto m:markers) for(auto p:m) hash.add(p);
      for(auto m:und_markers) for(auto p:m) hash.add(p);
      hash+=desc;
      hash.add(ids.begin(),ids.end());
      hash.add(nonMaxima.begin(),nonMaxima.end());
      hash+= pose_f2g;
      hash.add(und_kpts.begin(),und_kpts.end());
      hash.add(kpts.begin(),kpts.end());
      hash.add(depth.begin(),depth.end());
      hash+= image;
      hash+= smallImage;

       hash+=fseq_idx;
      hash.add(fernCode.begin(),fernCode.end());
      hash.add(scaleFactors.begin(),scaleFactors.end());
      hash.add(imageParams.getSignature());
      hash+=isBad;


      return hash;

  }


void Frame::clear()
{
      idx=std::numeric_limits<uint32_t>::max();//frame identifier
      markers.clear();
      und_markers.clear();
      markers_solutions.clear();
      keypoint_kdtree.clear();
      desc=cv::Mat();
      ids.clear();
      nonMaxima.clear();
      pose_f2g=se3();
      und_kpts.clear();
      kpts.clear();
      depth.clear();
      image=cv::Mat();
      smallImage=cv::Mat();
       fernCode.clear();
      fseq_idx=std::numeric_limits<uint32_t>::max();
      scaleFactors.clear();
      imageParams.clear();
      isBad=false;
}

void Frame::toStream(std::ostream &str) const  {

    int magic=134243;
    str.write((char*)&magic,sizeof(magic));

    str.write((char*)&idx,sizeof(idx));
    str.write((char*)&fseq_idx,sizeof(fseq_idx));
    str.write((char*)extra_1,sizeof(extra_1));
    str.write((char*)&isBad,sizeof(isBad));



    toStream__(desc,str);
    toStream__(und_kpts,str);
    toStream__(kpts,str);
    toStream__(depth,str);



    toStream__(ids,str);
    toStream__(nonMaxima,str);


    toStream__ts(und_markers,str);
    toStream__ts(markers,str);
    toStream__ts(markers_solutions,str);
    pose_f2g.toStream(str);
     toStream__(fernCode,str);


    toStream__(scaleFactors,str);
    imageParams.toStream(str);

    toStream__(image,str);
    toStream__(smallImage,str);


    keypoint_kdtree.toStream(str);

    magic=134244;
    str.write((char*)&magic,sizeof(magic));

}

void Frame::fromStream(std::istream &str) {
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if ( magic!=134243)throw std::runtime_error("Frame::fromStream error in magic");
    str.read((char*)&idx,sizeof(idx));
    str.read((char*)&fseq_idx,sizeof(fseq_idx));
    str.read((char*)extra_1,sizeof(extra_1));
    str.read((char*)&isBad,sizeof(isBad));

    fromStream__(desc,str);
    fromStream__(und_kpts,str);
    fromStream__(kpts,str);
    fromStream__(depth,str);


    fromStream__(ids,str);
    fromStream__(nonMaxima,str);
    fromStream__ts(und_markers,str);
    fromStream__ts(markers,str);
    fromStream__ts(markers_solutions,str);


    pose_f2g.fromStream(str);
     fromStream__(fernCode,str);

    fromStream__(scaleFactors,str);
    imageParams.fromStream(str);

    fromStream__(image,str);
    fromStream__(smallImage,str);

    keypoint_kdtree.fromStream(str);
    str.read((char*)&magic,sizeof(magic));
   if ( magic!=134244)throw std::runtime_error("Frame::fromStream error in magic");
   //  create_kdtree();
}





void FrameSet::toStream(ostream &str) const {
    //set magic
    int magic=88888;
    str.write((char*)&magic,sizeof(magic));
     ReusableContainer<ucoslam::Frame>::toStream(str);
 }

void FrameSet::fromStream(istream &str)
{
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if (magic!=88888) throw std::runtime_error("FrameSet::fromStream error reading magic");

    ReusableContainer<ucoslam::Frame>::fromStream(str);

}
uint64_t FrameSet::getSignature()const{
    uint64_t sig=0;
    for(auto const &f:*this)
        sig+=f.getSignature();
    return sig;
}
void MarkerPosesIPPE::toStream(std::ostream &str) const{
    toStream__(sols[0],str);
    toStream__(sols[1],str);
    str.write((char*)&errs[0],sizeof(errs[0]));
    str.write((char*)&errs[1],sizeof(errs[1]));
    str.write((char*)&err_ratio,sizeof(err_ratio));

}
void MarkerPosesIPPE::fromStream(std::istream &str){
    fromStream__(sols[0],str);
    fromStream__(sols[1],str);
    str.read((char*)&errs[0],sizeof(errs[0]));
    str.read((char*)&errs[1],sizeof(errs[1]));
    str.read((char*)&err_ratio,sizeof(err_ratio));
}

}
