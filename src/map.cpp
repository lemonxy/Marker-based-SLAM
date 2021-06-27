#include "map.h"
#include "stuff/io_utils.h"
#include "stuff/utils.h"
#include "stuff/timers.h"
#include "aruco/aruco.h"
#include "optimization/globaloptimizer.h"
namespace ucoslam{

//bool Map::operator==(const Map &m)const{
//    return map_points==m.map_points;
//}
//  Map *Map::ptr=0;

//Map* Map::singleton(){
//    if (ptr==0)   ptr=new Map();
//    return ptr;
//}

Map::Map()
{
    map_markers.clear();
}
float Map::getEffectiveFocus()const{
    if (keyframes.size()==0)return -1;
    return keyframes.begin()->imageParams.fx();
}

//! \param camPos 3D position of the camera (i.e. translation)
MapPoint & Map::addNewPoint(uint32_t frameSeqId){
    auto it_idx=map_points.insert(MapPoint());
    it_idx.first->id=it_idx.second;
    map_points[it_idx.second].creationSeqIdx=frameSeqId;
    return map_points[it_idx.second];
}

 MapPoint & Map::addNewPoint(const MapPoint &mp){
    auto it_idx=map_points.insert(mp);
    it_idx.first->id=it_idx.second;
    //add point info if already in
    auto &Mp=map_points[it_idx.second];

    vector<uint32_t> framesInMapPoint;
    for(auto fi:Mp.frames){
        assert(keyframes[fi.first].ids[fi.second]==std::numeric_limits<uint32_t>::max()  );
        keyframes[fi.first].ids[fi.second]=Mp.id;
        framesInMapPoint.push_back(fi.first);
    }
    //addd the covis info
    for(size_t i=0;i<framesInMapPoint.size()-1;i++)
        for(size_t j=i+1;j<framesInMapPoint.size();j++)
             TheKpGraph.createIncreaseEdge( framesInMapPoint[i],framesInMapPoint[j]);

    return Mp;
}
 Marker &Map::addMarker(const aruco::Marker &m){
    if (m.id == 231) cout<<"test\n";
     if( map_markers.count(m.id)==0) {
         if (m.id == 231)
             cout<<"id = 231\n";
         map_markers[m.id].id=m.id;
         map_markers[m.id].size=m.ssize;
         map_markers[m.id].dict_info=m.dict_info;
     }
     return map_markers[m.id];
 }

 Frame &Map::addKeyFrame(const Frame&f,bool setId){


     assert(getEffectiveFocus()<0 || fabs( getEffectiveFocus()-f.imageParams.fx())<10);//sets the effective focus if not yet

     if (getEffectiveFocus()>0)
         if ( fabs( getEffectiveFocus()-f.imageParams.fx())>10) throw std::runtime_error("Map::addKeyFrame keyframes added should have a similar focus than these already in the dataset");
             //we must be sure that the id is correct (returns the id, and must be the same already used)
     uint32_t kf_id=keyframes.addFrame(f);
     //work with the new frame, and free the pointer
     Frame &newFrame=keyframes[kf_id];
     if(setId)newFrame.idx=kf_id;
     //add it to the database
     _debug_msg_("");
     TheKFDataBase.add(newFrame);
     _debug_msg_("");
     return newFrame;
 }

bool Map::removeMapPointObservation(uint32_t pointId,uint32_t frameId,uint32_t minNumProjPoints){
    assert (map_points.is(pointId));
    auto &TheMapPoint=map_points[pointId];
    if (TheMapPoint.isStereo) minNumProjPoints--;

        if ( TheMapPoint.getNumOfObservingFrames()<=minNumProjPoints){
            removePoint(pointId);
            return true;
    }
    else{//otherwise, remove only its appearance in the frame
        if (keyframes.is(frameId)){
            keyframes[frameId].ids[ TheMapPoint.frames.at(frameId) ]=std::numeric_limits<uint32_t>::max();
            TheMapPoint.frames.erase(frameId);
            //need to update the covisgraph
            for(auto f_i:TheMapPoint.frames)
                TheKpGraph.decreaseRemoveEdge(f_i.first,frameId);
        }
    }
        return false;
}

void Map::addMarkerObservation(uint32_t markerId,uint32_t KeyFrame){
    assert(keyframes.is(KeyFrame));
    assert(map_markers.count(markerId)!=0);
    Marker &marker=map_markers[markerId];
    assert(marker.frames.count(KeyFrame)==0);//is not yet added

    //add connection with all frames in this point
    for(auto fi:marker.frames)
        TheKpGraph.createIncreaseEdge( fi,KeyFrame,4);
    //add the observation of this frame to the marker
    marker.frames.insert(KeyFrame);
}


 void Map::addMapPointObservation(uint32_t mapPoint,uint32_t KeyFrame,uint32_t KfKPtIdx){

     assert(keyframes.is(KeyFrame));
     assert(map_points.is(mapPoint));
     MapPoint &mp=map_points[mapPoint];
    //set it in keyframe
     assert (keyframes[KeyFrame].ids[KfKPtIdx]==mapPoint || keyframes[KeyFrame].ids[KfKPtIdx]==std::numeric_limits<uint32_t>::max());
     keyframes[KeyFrame].ids[KfKPtIdx]=mapPoint;
     //add connection with all frames in this point
     for(auto fi:mp.getObservingFrames())
         TheKpGraph.createIncreaseEdge( fi.first,KeyFrame);
     //add the observation of this frame to the point
     mp.addKeyFrameObservation(keyframes[KeyFrame],KfKPtIdx);

 }


 void Map::removePoint(uint32_t pid_remove, bool fullRemoval){
     assert(map_points .is(pid_remove));

     auto  &MP=map_points [pid_remove];
     MP.isBad=true;
     vector<uint32_t> frames;frames.reserve(MP.frames.size());
     for(auto f_id:MP.frames){//remove each frame connection
         frames.push_back(f_id.first);
         if ( keyframes[f_id.first].ids[f_id.second]==pid_remove)
             keyframes[f_id.first].ids[f_id.second]=std::numeric_limits<uint32_t>::max();
     }
     if (fullRemoval){
         //now, remove all frame connections
         for(size_t i=0;i<frames.size();i++)
             for(size_t j=i+1;j<frames.size();j++)
                 TheKpGraph.decreaseRemoveEdge(frames[i],frames[j]);
         //finally, remove the point
         map_points.erase(pid_remove);
     }

 }

 set<uint32_t> Map::removeKeyFrames(const std::set<uint32_t> &keyFrames,int minNumProjPoints){
     set<uint32_t> pointsToRemove;
     for(auto fidx:keyFrames){
         const auto &frame=keyframes[fidx];
         //first, remove from databse
         TheKFDataBase.del(frame);
         //remove its reference in the map points
         uint32_t invalid_ids=std::numeric_limits<uint32_t>::max();
         for(auto ids: frame.ids){
             if (ids!=invalid_ids){
                map_points[ids].frames.erase(fidx);
                 //if the number of frames it projects after this operation is below the threshold, the point should be removed too
                 if (map_points[ids].frames.size()<size_t(minNumProjPoints) ) removePoint(ids);
             }
         }

         //remove its reference in the markers
         for(auto marker:frame.und_markers)
             map_markers[marker.id].frames.erase(fidx);
         //finally, remove
         keyframes.erase(fidx);
         //also from covis graph
         TheKpGraph.removeNode(fidx);
         _debug_msg_("Removing Frame "<<fidx);
     }
     return pointsToRemove;
 }

 float Map::getFrameMedianDepth(uint32_t frame_idx){

     assert(keyframes.count(frame_idx)!=0);
     auto frame=keyframes[frame_idx];
     //must transform the points to the camera
     //compute the average depth of the frame and check the ratio baseline/depth
     vector<float> depths;
     depths.reserve(frame.ids.size());
      for(auto p:frame.ids){
         if ( p!=std::numeric_limits<uint32_t>::max()){
             auto &mp=map_points[p];
             auto p=  frame.pose_f2g*mp.getCoordinates();
             depths.push_back( p.z);
         }
     }

     //now the marker points
     for(auto m:frame.und_markers){
         assert(  map_markers.count(m.id) );
         if (  map_markers[m.id].pose_g2m.isValid() ){
             auto p3d=map_markers[m.id].get3DPoints();
             for(auto p:p3d) {
                 p=frame.pose_f2g*p;
                 depths.push_back(p.z);
             }
         }
     }

     auto outliers=outlierFiltering(depths,3);
     //compute the mean without the outliers
     removeFromVector(depths,outliers);
     float  mean =0;
     for(auto d:depths) mean+=d;

     return  mean/float(depths.size());
 }
void Map::clear(){
    map_points.clear();
    map_markers.clear();
    keyframes.clear();
    TheKpGraph.clear();
    TheKFDataBase.clear();
}

uint32_t Map::getNextFrameIndex()const{return keyframes.getNextPosition();}

bool Map::hasPointStereoObservations(uint32_t mpIdx)const{
    for(const auto &frame:map_points.at(mpIdx).frames){
        if ( keyframes[frame.first].depth[frame.second]!=0)return true;
    }
    return false;
}

void Map::fuseMapPoints(uint32_t mp1,uint32_t mp2 ,bool fullRemovePoint2){
    //save observations
    auto &Mp2=map_points[mp2];
    auto p2_observations=Mp2 .frames;
    auto &Mp1=map_points[mp1];
    //remove the point
    removePoint(mp2,fullRemovePoint2);
    //add observations to the point that remains
    for(auto p2_obs:p2_observations){
        if(! Mp1.frames.count(p2_obs.first))//the points should not be already in this frame. Otherwise it would be projected in two different locations
            addKeyFrameObservation(mp1,p2_obs.first,p2_obs.second);
    }
 }

void Map::addKeyFrameObservation(uint32_t mapId, uint32_t frameId, uint32_t frame_kpidx){
    assert(map_points.is(mapId));
    assert(keyframes.is(frameId));
    assert(keyframes[frameId].ids[frame_kpidx]==std::numeric_limits<uint32_t>::max());
    assert( map_points[mapId].frames.count(frameId)==0);//the points should not be already in this frame. Otherwise it would be projected in two different locations
    auto observingFrame=    map_points[mapId].getObservingFrames();
    map_points[mapId].addKeyFrameObservation(keyframes[frameId],frame_kpidx);
    keyframes[frameId].ids[frame_kpidx]=mapId;
    //add an edges to others frames
    for(auto of:observingFrame)
        if (of.first!=frameId)
            TheKpGraph.createIncreaseEdge( frameId, of.first);
}

void Map::toStream(iostream &str)  throw (std::exception){
    lock(__FUNCTION__,__FILE__,__LINE__);
    TheKFDataBase.toStream(str);

    map_points.toStream(str);
    toStream__kv_complex(map_markers,str);
    keyframes.toStream(str);
    TheKpGraph.toStream(str);
    unlock(__FUNCTION__,__FILE__,__LINE__);
}

void Map::fromStream(std::istream &str)throw (std::exception){
    lock(__FUNCTION__,__FILE__,__LINE__);
    clear();

    TheKFDataBase.fromStream(str);
    map_points.fromStream(str);
    fromStream__kv_complexmap(map_markers,str);
    keyframes.fromStream(str);
    TheKpGraph.fromStream(str);
    unlock(__FUNCTION__,__FILE__,__LINE__);
}

void Map::saveToFile(std::string   fp)throw(std::exception){
    std::fstream file(fp, ios::out|ios::binary);
    if (!file) throw std::runtime_error("Could not open file for writing:"+fp);
    uint64_t sig=225237123;//magic number
    file.write((char*)&sig,sizeof(sig));
    toStream(file);
}

void Map::readFromFile(std::string fp)throw(std::exception){
    std::ifstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for reading:"+fp);
    uint64_t sig=225237123;
    file.read((char*)&sig,sizeof(sig));
    if (sig!=225237123)throw std::runtime_error("Map::readFromFile   is not of approrpiate type");
    fromStream(file);
}
uint64_t Map::getSignature()const{
    Hash sig;
    for(const auto &p:map_points)
        sig+=p.getSignature();
     sig+=keyframes.getSignature();
     return sig;

}

bool Map::checkConsistency(bool checkCovisGraph,bool useMutex){
      std::unique_lock<std::mutex> lockconsistency(consitencyMutex);
    bool consistent=true;
//    if ( _curKFRef!=-1){
//        if (! keyframes.is(_curKFRef)){
//            cerr<<" _curKFRef is invalid"<<endl;
//            consistent=false;
//        }
//    }

    if ( keyframes.size()==1)return true;
 if(useMutex)   lock(__FUNCTION__,__FILE__,__LINE__);

//check the number of elements in the graph is the same as the number of frames
//    if ((TheKpGraph.size()+TheMarkerGraph.size()) >=keyframes.size() ){
//        consistent=false;
//        cerr<<" TheCVGraph.size()+TheMarkerGraph.size()!=keyframes.size()"<<endl;
//    }
    //check all are in
    for(auto f:keyframes)
        if (!TheKpGraph.isNode(f.idx) && !TheKpGraph.isNode(f.idx) ){
            consistent=false;
            cerr<<" Frame "<<f.idx<<" not in any of the Graphs"<<endl;
        }

    //check points are valid
    for(const auto & p:map_points){
        if (! p.isValid()){
            cerr<<"Point << "<<p.id<<" invalid"<<endl;
            consistent=false;
        }
    }

    //check that points-frames are consistent
     for(const auto & p:map_points){
        if (p.isBad)continue;
        //for each frame it projects in
        for(const auto &f:p.frames){
            if (!keyframes.is(f.first)){
                cerr<<"x1 Missing frame :"<<f.first<<endl;
                consistent=false;
            }
          if ( (keyframes)[f.first].ids[f.second]!=p.id){
               consistent=false;
               cerr<<"point "<<p.id<<" is suposed to be in frame "<<f.first<<endl;
            }
        }
    }

    //check consistency of point-frames the other way around
    for(auto &f:keyframes){
        for(auto id:f.ids){
            if (id!= std::numeric_limits<uint32_t>::max()){
                if ( !map_points.is(id)) {
                    consistent=false;
                    cerr<<"Point "<<id<<" is lost but referenced in frmae "<<f.idx<<endl;
                }
                //check that the point has this reference
                if ( !map_points[id].frames.count(f.idx)){
                    cerr<<"Point "<<id<<" should reference to frame "<<f.idx<<endl;
                    consistent=false;
                }
            }
        }
    }

    //everypoint must have at least two projections
        for(const auto &p:map_points){
            bool isStereo=false;
            for(auto f:p.frames)
                if (keyframes[f.first].depth[f.second]!=0) isStereo|=true;
            if (p.frames.size()<=1 && !isStereo){
                cerr<<"Point "<<p.id<<" should have at least two projections "<<endl;
                consistent=false;
            }
        }


        for(const auto &p:map_points){
            if (p.frames.size()==0){
                cerr<<"Point "<<p.id<<" should have at least one projection "<<endl;
                consistent=false;
            }
        }

    //////MARKERS
    //check that marker-frames are consistent
    for(const auto & m:map_markers){
        //for each frame it projects in
        for(auto f:m.second.frames){
            const auto &frame=(keyframes)[f];
            //find the marker in the frame vector(slow)
            int idx=-1;
            for(size_t i=0;i<frame.und_markers.size();i++)
                if ( frame.und_markers[i]==m.first) {idx=i;break;}
            if (idx==-1){consistent=false;cerr<<"Marker "<<m.first<<" is suposed to be in frame "<<f<<endl;
            }
        }
    }

    //check consistency of marker-frames the other way around
    for(auto &f:keyframes){
        for(const auto &m:f.und_markers){
                if ( !map_markers.count(m.id)) {
                    consistent=false;
                    cerr<<"Marker "<<m.id<<" is lost but referenced in frmae "<<f.idx<<endl;
                }
                //check that the point has this reference
                if ( !map_markers[m.id].frames.count(f.idx)){
                    cerr<<"Marker "<<m.id<<" should reference to frame "<<f.idx;
                    consistent=false;
                }
        }
    }

    //check that all markers detected are in the map, even if without valid position
    {
     for(const auto &frame:keyframes)
        for(auto m:frame.und_markers)
            if ( map_markers.count(m.id)==0){
                cerr<<"Marker "<<m<<" should be registered in the map"<<endl;
                consistent=false;
            }
    }

    //ckeck the covisibility graph of markers is consistent
   if (0) {
        //to do so, create the covis graph here
        std::map<uint64_t,zeroinitvar<uint32_t> > frame_commonmakers;
        for(const auto &marker_:map_markers){
            vector<uint32_t> vframes;
            for(auto ob:marker_.second.frames)vframes.push_back(ob);
                for(size_t i=0;i<vframes.size();i++)
                    for(size_t j=i+1;j<vframes.size();j++)
                        frame_commonmakers[CovisGraph::join(vframes[i],vframes[j])]++;
        }
        //check that the value is the same as in the covisgraph
        for(auto fij:frame_commonmakers){
            auto i_j=CovisGraph::separe(fij.first);
            if (!TheKpGraph.isEdge(i_j.first,i_j.second)){
                cerr<<"Marker graph edege "<<i_j.first<<"-"<<i_j.second<<" is not in the graph "<<endl;
                consistent=false;
            }
            if ( TheKpGraph.getEdge(i_j.first,i_j.second)!= fij.second){
                cerr<<"Marker graph edege "<<i_j.first<<"-"<<i_j.second<<" has weight "<<TheKpGraph.getEdge(i_j.first,i_j.second)<<" but should be "<< fij.second<<endl;
                consistent=false;
            }
        }

        for(auto fij:TheKpGraph.getEdgeWeightMap()){
            if (frame_commonmakers.count(fij.first)==0){
                auto i_j=CovisGraph::separe(fij.first);
                cerr<<"Marker graph edge "<<i_j.first<<"-"<<i_j.second<<" should not be"<<endl;
                consistent=false;

            }
        }

    }


//    //check that if a marker has a valid position, there is at least two views to the marker(ie, they have not been removed)
//    for(const auto &m:map_markers){
//        if (m.second.frames.size()<2 && m.second.pose_g2m.isValid()){
//            cerr<<"The valid marker "<<m.first<<" should be visible in at least two views"<<endl;
//            consistent=false;
//        }
//    }

    //check no nan point
    for(  auto &p:map_points)
    {
        auto p3d=p.getCoordinates();
        if (std::isnan(p3d.x)||std::isnan(p3d.y)||std::isnan(p3d.z)){
            consistent=false;
            cerr<<"point "<<p.id<<" is nan"<<endl;
        }
    }

    //all frames in KFDatabase and no other
    {
        if (!TheKFDataBase.isEmpty()){//it is valid
            int nframes_seen=0;
            for(auto f:keyframes){
                if (!TheKFDataBase.isId(f.idx)){
                    consistent=false;
                    cerr<<"frame :"<<f.idx<<" not found in the database"<<endl;
                }
                else nframes_seen++;
            }
            //now, check that there is the same number of elements in bt
            if (TheKFDataBase.size()!=size_t(nframes_seen)){
                consistent=false;
                cerr<<"The dabase contains frames not in the TheFrameSet"<<endl;
            }
        }
    }
    //check that matches are correct

//    for(auto m:map_matches){
////        if (map_points.is(m.trainIdx)){
////            cout<<"Match to map point "<<m.trainIdx<< " is incorrect"<<endl;
////            consistent=false;
////        }
//        if (m.queryIdx>= _cFrame.ids.size()){
//            cout<<"match query out of index"<<endl;
//            consistent=false;
//        }
//    }

    //covis graph
    if (checkCovisGraph){
        ScopeTimer Timer("COVIS");
        CovisGraph auxCV;
        for(const auto &p:map_points){
            vector<uint32_t> frames;
            for(auto f:p.frames)frames.push_back(f.first);
            for(size_t i=0;i<frames.size();i++)
                for(size_t j=i+1;j<frames.size();j++){
                    auxCV.createIncreaseEdge(frames[i],frames[j]);
                }
        }
        //now, for the markers
        for(auto m:map_markers){
            vector<uint32_t> frames;
            for(auto f:m.second.frames)frames.push_back(f);
            for(size_t i=0;i<frames.size();i++)
                for(size_t j=i+1;j<frames.size();j++){
                    auxCV.createIncreaseEdge(frames[i],frames[j],4);
                }
        }

        //now compare
        auto n1= auxCV.getNodes( );
        auto n2= TheKpGraph.getNodes( );

        //check are equal
        if (n1!=n2){
            cerr<<"Covisgraph missing nodes (y)"<<endl;
            consistent=false;
        }
        //now, check the neighbors of each

        for(auto n:n1){
            auto neigh1=auxCV.getNeighbors(n);
            auto neigh2=TheKpGraph.getNeighbors(n);
            if (neigh1!=neigh2){
                cerr<<"Neigbors of "<<n<<" are incorrect"<< endl;
                consistent=false;
            }
            //now, check weights
            for(auto ne1:neigh1)
                if (auxCV.getEdge(n,ne1)!=TheKpGraph.getEdge(n,ne1) ){
                    cerr<<"Edge of nodes "<<n<<","<<ne1<< " is incorrect. It is:"<<TheKpGraph.getEdge(n,ne1) <<" and shoudl be :"<<auxCV.getEdge(n,ne1) <<endl;
                    consistent=false;

                }
        }
    }



    if(useMutex)  unlock(__FUNCTION__,__FILE__,__LINE__);

    _debug_msg_("Consistency CHECK PASSED="<<consistent);
    return consistent;
}
vector<uint32_t> Map::relocalizationCandidates(Frame &frame)throw(std::exception){
    return TheKFDataBase.relocalizationCandidates(frame,keyframes,TheKpGraph);
}
vector<cv::DMatch> Map::matchFrameToKeyFrame(const Frame&, uint32_t kf, float minDescDist,xflann::Index &xfindex){
    //take all the descriptors of the mappoints seen and put them in a matrix. Then create a index for fast search
    Frame &kframe=keyframes[kf];
    //save the indices of the map points in mpts
    vector<uint32_t> mpts;mpts.reserve(kframe.und_kpts.size());
    for(auto kpidx:kframe.ids){
        if (kpidx!=std::numeric_limits<uint32_t>::max())
            mpts.push_back(kpidx);
    }
    if (mpts.size()==0)return{};
    //create the matrix MapPointDescriptors where the descritors are copied
    cv::Mat ADescriptor;
    map_points[ mpts[0]].getDescriptor(ADescriptor);//take a descriptor to know its size and type
    cv::Mat MapPointDescriptors(mpts.size(),ADescriptor.cols,ADescriptor.type() );
    //start the copy
    for(size_t i=0;i<mpts.size();i++){
        cv::Mat row=MapPointDescriptors.row(i);
        map_points[ mpts[i]].getDescriptor(row);
    }

    //now, do a search amonts the descriptors in this the search frame f

    cv::Mat indices,distances;
    xfindex.search(MapPointDescriptors,2,indices,distances,xflann::KnnSearchParams(32,true));

    //index.search(curFrame.desc,2,indices,distances,xflann::KnnSearchParams(32,true));
    if ( distances.type()==CV_32S) distances.convertTo(distances,CV_32F);

    float distThres= minDescDist;
    float nn_match_ratio=0.8;

    vector<cv::DMatch> matches;
    for(int i = 0; i < indices.rows; i++) {
        if ( distances.at<float>(i,0)<distThres){
            if(distances.at<float>(i,0)  < float(nn_match_ratio * float(distances.at<float>(i,1)))) {
                cv::DMatch  match;
                match.queryIdx= indices.at<int>(i,0);
                match.trainIdx= mpts[ i];
                match.distance=distances.at<float>(i,0);
                matches.push_back(match);
            }
        }
    }
    filter_ambiguous_query(matches);
    return matches;
}

vector<cv::DMatch> Map::matchFrameToKeyFrame(const Frame&curFrame,uint32_t kf,float minDescDist)
{
        //take all the descriptors of the mappoints seen and put them in a matrix. Then create a index for fast search
        Frame &kframe=keyframes[kf];
        //save the indices of the map points in mpts
        vector<uint32_t> mpts;mpts.reserve(kframe.und_kpts.size());
        for(auto kpidx:kframe.ids){
            if (kpidx!=std::numeric_limits<uint32_t>::max())
                mpts.push_back(kpidx);
        }
        if (mpts.size()==0)return{};
        //create the matrix MapPointDescriptors where the descritors are copied
        cv::Mat ADescriptor;
        map_points[ mpts[0]].getDescriptor(ADescriptor);//take a descriptor to know its size and type
        cv::Mat MapPointDescriptors(mpts.size(),ADescriptor.cols,ADescriptor.type() );
        //start the copy
        for(size_t i=0;i<mpts.size();i++){
            cv::Mat row=MapPointDescriptors.row(i);
            map_points[ mpts[i]].getDescriptor(row);
        }

        //now, do a search amonts the descriptors in this the search frame f
        xflann::Index index;
        index.build(MapPointDescriptors,xflann::HKMeansParams(8,0));
        cv::Mat indices,distances;
        index.search(curFrame.desc,2,indices,distances,xflann::KnnSearchParams(32,true));
        if ( distances.type()==CV_32S) distances.convertTo(distances,CV_32F);

        float distThres= minDescDist;
        float nn_match_ratio=0.8;

        vector<cv::DMatch> matches;
        for(int i = 0; i < indices.rows; i++) {
            if ( distances.at<float>(i,0)<distThres){
                if(distances.at<float>(i,0)  < float(nn_match_ratio * float(distances.at<float>(i,1)))) {
                    cv::DMatch  match;
                    match.queryIdx= i;
                    match.trainIdx= mpts[ indices.at<int>(i,0)];
                    match.distance=distances.at<float>(i,0);
                    matches.push_back(match);
                }
            }
        }
        filter_ambiguous_train(matches);
        return matches;
}

std::vector<cv::DMatch> Map::matchFrameToMapPoints( const std::vector<uint32_t > &  used_frames, Frame & curframe, const cv::Mat & pose_f2g_, float minDescDist, float maxRepjDist,bool markMapPointsAsVisible,bool useAllPoints)
{

    Se3Transform pose_f2g;pose_f2g=pose_f2g_;
    ScopedTimerEvents timer("Slam::matchMapPtsOnFrames");
    //remove this already seen that have been marked with the lastSeqFrameIdx in the map point
    std::vector<uint32_t>  smap_ids=getMapPointsInFrames(used_frames.begin(),used_frames.end());
    timer.add("step1");
    if (!useAllPoints){
        for(size_t i=0;i< smap_ids.size();i++){
            if ( map_points.is ( smap_ids[i]))
                if ( map_points[ smap_ids[i]].getExtra<uint32_t>()[0]==curframe.fseq_idx)//already seen and projected
  //              if ( map_points[ smap_ids_all[i]].lastTimeSeenFSEQID==curframe.fseq_idx)//already seen and projected
                    smap_ids[i]= std::numeric_limits<uint32_t>::max();
        }
        smap_ids.erase(std::remove(smap_ids.begin(), smap_ids.end(),  std::numeric_limits<uint32_t>::max()),smap_ids.end());
    }

    timer.add("step2");
    if (smap_ids.size()==0) return {};

    //Condition 2. Compute angle between current viewing ray and the map point mean viewing direction
    //             Discard if v.dot(n) < cos(60)

    cv::Point3f camCenter= pose_f2g.inv()*cv::Point3f(0,0,0);  //obtain camera center in global reference system
    // Project points given pose
    timer.add("step3");


    std::vector<cv::DMatch> map_matchesLocalMap; // [query=kpts; train=map]
    map_matchesLocalMap.reserve(smap_ids.size());

    float fx=curframe.imageParams.CameraMatrix.at<float>(0,0);
    float fy=curframe.imageParams.CameraMatrix.at<float>(1,1);
    float cx=curframe.imageParams.CameraMatrix.at<float>(0,2);
    float cy=curframe.imageParams.CameraMatrix.at<float>(1,2);

    std::vector<float> scaleFactors=curframe.scaleFactors;
    float scaleFactorLog=log(scaleFactors[1]);
    // Find matches map-points keypoints
    for (uint32_t mpix = 0; mpix < smap_ids.size(); mpix++)
    {

        MapPoint &mapPoint=map_points[ smap_ids[mpix] ];
        float viewCos=mapPoint.getViewCos(camCenter);
        if (viewCos<0.7) continue;
        //move point to camera coordinates
        cv::Point3f p3d=pose_f2g* mapPoint.getCoordinates();
        if (p3d.z<0)
            continue;//only consider if in front of camera
        //only consider if the keypoint is into margin limits
        //compute distance to point
        float distToPoint= cv::norm(p3d);
        if (   !(mapPoint.getMinDistanceInvariance() < distToPoint && distToPoint< mapPoint.getMaxDistanceInvariance()))
            continue;
        p3d.z=1./p3d.z;
        cv::Point2f p2d(  p3d.x*fx*p3d.z+cx,p3d.y*fy*p3d.z+cy);

        if ( !(p2d.x>0  &&p2d.y>0  &&p2d.x<curframe.imageParams.CamSize.width &&p2d.y<curframe.imageParams.CamSize.height))
            continue;
        //mark as visible
        if (markMapPointsAsVisible)
            mapPoint.setVisible(curframe.fseq_idx );

        // ----------------------------------------------------------
        //now, analyze these and find the one with minimal descriptor distance
        float radius_scale=1;//scale for radius search that depedens on the octave
        if(viewCos<0.9) radius_scale*=1.5;
        int best_candidate_kp=-1,bestLevel=0,bestLevel2=-1;
        //now, predict the octave
        int predictedOctave=mapPoint.predictScale(distToPoint,scaleFactorLog,scaleFactors.size());
      //  predictedOctave=mapPoint.predictScale(distToPoint,scaleFactorLog,scaleFactors.size());


        radius_scale=scaleFactors[predictedOctave];
        float best_candidate_distance=std::numeric_limits<float>::max(),bestdistance2=std::numeric_limits<float>::max();

        // Find keypoints closer (spatial distance) to this map point
        vector<uint32_t> keyp_region=curframe.getKeyPointsInRegion(p2d,radius_scale*maxRepjDist,predictedOctave-1,predictedOctave);
        for (auto kp_idx : keyp_region )
        {
                //compute visual distance to map point
                float desc_dist= mapPoint.getDescDistance(curframe.desc.row( kp_idx));
                if (desc_dist<minDescDist){
                    if( desc_dist<best_candidate_distance){
                        best_candidate_distance=desc_dist;
                        best_candidate_kp=kp_idx;
                        bestLevel=curframe.und_kpts[kp_idx ].octave;
                    }
                    else if (desc_dist<bestdistance2){
                        bestdistance2=desc_dist;
                        bestLevel2=curframe.und_kpts[kp_idx].octave;
                    }

                }
        }


        if (best_candidate_kp!=-1){
            bool valid=true;
            if ( bestLevel2==bestLevel && best_candidate_distance> 0.8*bestdistance2)  valid=false;
            if (valid){
                cv::DMatch mt;
                mt.queryIdx = best_candidate_kp; // kp
                mt.trainIdx = smap_ids[mpix];                       // map
                mt.distance = best_candidate_distance;
                map_matchesLocalMap.push_back(mt);
            }
        }
    }
    timer.add("step4");



    _debug_msg("final number of matches= "<<map_matchesLocalMap.size(),10);
    filter_ambiguous_query(map_matchesLocalMap);


    _debug_msg("final number of matches2= "<<map_matchesLocalMap.size(),10);
    return map_matchesLocalMap;
}

double Map::globalReprojChi2( const std::vector<uint32_t> &used_frames , std::vector<float > *chi2vector,
                               std::vector<std::pair<uint32_t,uint32_t> > *map_frame_ids,bool useKeyPoints,bool useMarkers) {

    double chi2Sum=0;
    int np=0;
    if (chi2vector!=0) {
        chi2vector->clear();
        chi2vector->reserve(map_points.size());
    }
    if(map_frame_ids!=0){
        map_frame_ids->clear();
        map_frame_ids->reserve(map_points.size());
    }

    //set all used frames info
    vector<bool> usedFramesVector;
    if ( used_frames.size()==0) usedFramesVector=std::vector<bool>(keyframes.capacity(),true);
    else {
        usedFramesVector=std::vector<bool>(keyframes.capacity(),false);
        for(auto f:used_frames) usedFramesVector[f]=true;
    }

    //set in the frames the projection matrix
    if (useKeyPoints){
    for(  auto & p:map_points){
        cv::Point3f p3d=p.getCoordinates();
        //for each frame it projects in
        for(auto f:p.frames){
            if (usedFramesVector[f.first]){
                const Frame &frame=keyframes[f.first];
                cv::Point2f p2= frame.project(p3d,true,false);
                if ( !isnan(p2.x)){
                        cv::Point2f kp= frame.und_kpts[f.second].pt;
                        auto ee=p2-kp;
                        float inv_scale_factor=1./frame.scaleFactors[frame.und_kpts[f.second].octave];
                        float chi2=inv_scale_factor *( ee.x*ee.x+ee.y*ee.y);
                        chi2Sum+=chi2;
                        if (chi2vector) chi2vector-> push_back(chi2  );
                        if(map_frame_ids)map_frame_ids->push_back({p.id,f.first});
                        np++;
                }
            }
        }
    }
    }

    //do the same for markers
    if(useMarkers){
        for(const auto & m:map_markers){
            if ( m.second.pose_g2m.isValid()){
                for(auto f:m.second.frames){
                    if (usedFramesVector[f]){
                    auto points2d=keyframes[f].getMarker(m.second.id);
                    auto points3d=m.second.get3DPoints();
                    for(int i=0;i<4;i++){
                        auto p2= keyframes[f].project(points3d[i],true);
                        if ( !isnan(p2.x)){
                            auto ee=p2-points2d[i];
                            chi2Sum+=ee.x*ee.x;
                            chi2Sum+=ee.y*ee.y;
                            np++;
                        }
                    }
                }
                }
            }
        }
    }
    return chi2Sum;
}
void Map::removeBadAssociations(const vector<std::pair<uint32_t,uint32_t>> &BadAssociations,int minNumPtObs){
     for(auto badAssoc:BadAssociations){
        if (map_points.is(badAssoc.first))
        removeMapPointObservation(badAssoc.first,badAssoc.second,minNumPtObs);
    }
}
void Map::updatePointInfo(uint32_t pid){
    struct bestObs{
        uint32_t frame=std::numeric_limits<uint32_t>::max();
        int octave=std::numeric_limits<int>::max();
        uint32_t kpIdx=std::numeric_limits<uint32_t>::max();
    } bOb;


    MapPoint &mp=map_points[pid];
    //compute the new normal of the point
    auto p3d=mp.getCoordinates();
    vector<cv::Point3f> normals;
    for(auto f:map_points[pid].frames){
        Frame &frame=keyframes[f.first];
        normals.push_back(frame.getCameraCenter()-p3d);
        if(bOb.octave>frame.und_kpts[f.second].octave){
            bOb.octave=frame.und_kpts[f.second].octave;
            bOb.frame=f.first;
            bOb.kpIdx=f.second;
        }
    }
    //now, update the normal
   mp.updateNormals(normals);
   mp.updateBestObservation(keyframes[bOb.frame],bOb.kpIdx);
}





void Map::saveToPcd(std::string filepath,float pointViewDirectionDist)throw(std::exception){
    std::vector<cv::Vec4f> points2write;
    //markers

    float markerSize=-1;
    for(const auto &m:map_markers)
    {
        markerSize=m.second.size;
        assert(m.second.size>0);
        if (m.second.pose_g2m.isValid()){
            cv::Scalar color;
            color=cv::Scalar(255,0,0);
            auto points4=getPcdPoints(m.second.get3DPoints(),color);
            points2write.insert(points2write.end(),points4.begin(),points4.end());
            auto points_id=getMarkerIdPcd(m.second.pose_g2m,m.second.size,m.second.id,color);
            points2write.insert(points2write.end(),points_id.begin(),points_id.end());
            //max_msize=std::max(max_msize,m.second.markerSize);
        }
    }

     vector<cv::Point3f> marker_points = { cv::Point3f ( -markerSize/2., markerSize/2.,0 ),cv::Point3f ( markerSize/2., markerSize /2.,0 ),
                                          cv::Point3f ( markerSize/2., -markerSize/2.,0 ),cv::Point3f ( -markerSize/2., -markerSize/2.,0 )  };
    for(auto frame:keyframes){
        cv::Mat f2c=frame.pose_f2g.inv();
        vector<cv::Point3f> mpoints(4);
        for(int i=0;i<4;i++)  mpoints[i]=mult(f2c, marker_points[i]);
        //  for(auto p:mpoints)cout<<p<<endl;
        auto color=cv::Scalar(0,255,0);
        if (frame.idx==0)
            color=cv::Scalar(0,0,255);
        auto pcam=getPcdPoints( mpoints,color,25);
        points2write.insert(points2write.end(),pcam.begin(),pcam.end());
        auto points_id=getMarkerIdPcd(frame.pose_f2g.inv(), cv::norm(marker_points[0]-marker_points[1]),frame.idx,color);
        points2write.insert(points2write.end(),points_id.begin(),points_id.end());


    }
    //3d points
    {
        cv::Scalar pcolor(255,0,255);
        float fcolor;uchar *c=(uchar*)&fcolor;
        for(int i=0;i<3;i++)c[i]=pcolor[i];
        c[3]=0;
        for(auto p:map_points){
            cv::Point3f p3d=p.getCoordinates();
            points2write.push_back(cv::Vec4f(p3d.x,p3d.y,p3d.z,fcolor));
        }

    }

    //add the viewing direction
    if (pointViewDirectionDist>0){
        for(  auto  p:map_points){
            auto v=p.getNormal();
            auto pc=p.getCoordinates();
            auto pviewdir=getPcdPointsLine(pc,pc+(v*pointViewDirectionDist),cv::Scalar  (255,125,125),25);
            points2write.insert(points2write.end(),pviewdir.begin(),pviewdir.end());

        }
    }
    std::ofstream filePCD ( filepath, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";


    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));

}

std::vector<cv::Vec4f> Map::getPcdPointsLine(const cv::Point3f &a,const cv::Point3f &b,cv::Scalar color,int npoints ){
    vector<cv::Vec4f> points;
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    //lines joining points
    cv::Point3f v=b-a;
    float ax=1./float(npoints);//npoints
    for(float x=0;x<=1;x+=ax){
        cv::Point3f p3=a+v*x;
        points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
    }
    return points;
}

std::vector<cv::Vec4f> Map::getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints ){
   vector<cv::Vec4f> points;
   double msize=cv::norm(mpoints[0]-mpoints[1]);
   float fcolor;uchar *c=(uchar*)&fcolor;
   for(int i=0;i<3;i++)c[i]=color[i];

   //lines joining points
   for(size_t i=0;i<mpoints.size();i++){
       cv::Point3f v=mpoints[(i+1)%mpoints.size()]-mpoints[i];
       float ax=1./float(npoints);//npoints
       for(float x=0;x<=1;x+=ax){
           cv::Point3f p3=mpoints[i]+v*x;
           points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
       }
   }

   //line indicating direction
   //take first and second, first and last , and get the cross vector indicating the direction
   cv::Point3f v1=mpoints[1]-mpoints[0];
   cv::Point3f v2=mpoints[3]-mpoints[0];
    v1*=1./cv::norm(v1);
   v2*=1./cv::norm(v2);
   cv::Point3f vz=v2.cross(v1);
   vz*=1./cv::norm(vz);//now, unit

   //set the center
   cv::Point3f center=(mpoints[0]+mpoints[1]+mpoints[2]+mpoints[3])*0.25;
   float ax=(msize/3)/100;
   for(float x=0;x<=msize/3;x+=ax){
       cv::Point3f p3=center+vz*x;
       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));

   }


   return points;

}

std::vector<cv::Vec4f> Map::getMarkerIdPcd( cv::Mat rt_g2m_, float _markerSize,uint32_t id,cv::Scalar color ){

    vector<cv::Point3f> marker_points = { cv::Point3f ( -_markerSize/2., _markerSize/2.,0 ),cv::Point3f ( _markerSize/2., _markerSize /2.,0 ),
                                          cv::Point3f ( _markerSize/2., -_markerSize/2.,0 ),cv::Point3f ( -_markerSize/2., -_markerSize/2.,0 )  };

    cv::Mat rt_g2m;
    rt_g2m_.convertTo(rt_g2m,CV_32F);
     //marker id as a set of points
    string text = std::to_string(id);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline=0;
    float markerSize_2=_markerSize/2;
    cv::Size textSize = cv::getTextSize(text, fontFace,
                                        fontScale, thickness, &baseline);
    cv::Mat img(textSize +cv::Size(0,baseline/2), CV_8UC1,cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0,textSize.height+baseline/4), fontFace, fontScale,cv::Scalar::all(255), thickness, 8);
    //raster 2d points as 3d points
    vector<cv::Point3f> points_id;
    for(int y=0;y<img.rows;y++)
        for(int x=0;x<img.cols;x++)
            if (img.at<uchar>(y,x)!=0) points_id.push_back( cv::Point3f((float(x)/float(img.cols))-0.5,(float(img.rows-y)/float(img.rows))-0.5,0));


    //now,scale
    for(auto &p:points_id)p*=markerSize_2;
    //finally, translate
    for(auto &p:points_id)p= mult<float>( rt_g2m,p);
    //now, add to ouput

    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    vector<cv::Vec4f> res;
    for(auto &p:points_id)
        res.push_back(cv::Vec4f(p.x,p.y,p.z, fcolor));



    return res;
}



std::vector<uint32_t> Map::getNearestNeighborsKeyFrames(const Frame &frame, size_t nneighbors, float minDist) {

    //find the ids of the points detected and the frames they belong to
    std::map<uint32_t,zeroinitvar<int> > frame_counter;
    for(auto id:frame.ids){
        if (id!=std::numeric_limits<uint32_t>::max()){
            //take the keypoint and add the frames it appears in
            if ( map_points.is(id)){
                if  ( !map_points[id].isBad)
                    for(auto f_id:map_points[id].frames)
                        frame_counter[f_id.first]++;
            }
        }
    }
    //no matches found,
    if  (frame_counter.size()==0)   return  {};


    vector<pair<uint32_t,float > > ndist;
    ndist.reserve(frame_counter.size());
    cv::Point3f cam1= frame.getCameraCenter();
    float distT2=minDist*minDist;
    for(auto fc:frame_counter){
        if (fc.second>20){//at least 20 matches in common, then find distance
            cv::Point3f cam2=keyframes[fc.first].getCameraCenter();
            float d2=(cam1.x-cam2.x)*(cam1.x-cam2.x) + (cam1.y-cam2.y)*(cam1.y-cam2.y)+(cam1.z-cam2.z)*(cam1.z-cam2.z);
            if (d2<distT2)
                ndist.push_back(  {fc.first,d2});
        }
    }
    if ( ndist.size()>nneighbors){
        std::sort(ndist.begin(),ndist.end(),[](const pair<uint32_t,float > &a,const pair<uint32_t,float >&b){return a.second<b.second;});
        ndist.resize( nneighbors);
    }
    vector<uint32_t> result(ndist.size());
    for(size_t i=0;i<result.size();i++)result[i]=ndist[i].first;
    return result;
}

///通过闭环marker检测当前的位姿
///重定位过程也用
se3 Map::getBestPoseFromValidMarkers(const Frame &frame,const vector<uint32_t> &markersOfFrameToBeUsed,float minErrorRatio){

    struct minfo{
        int id;
        cv::Mat rt_f2m;
        double err;
    };
    se3 pose_f2g_out;//result
    //estimate the markers locations and see if there is at least one good enough
    vector<minfo> good_marker_locations;
    vector<minfo> all_marker_locations;
    for(const auto m:markersOfFrameToBeUsed){//for ech visible marker
        const auto &marker_poseinfo=frame.getMarkerPoseIPPE(m);
        minfo mi;
        mi.id=m;
        mi.err=marker_poseinfo.errs[0];
        mi.rt_f2m=marker_poseinfo.sols[0];
        all_marker_locations.push_back(mi);
        ///BIT ADD
        if( marker_poseinfo.err_ratio > minErrorRatio &&
            marker_poseinfo.distR > 1.2*marker_poseinfo.tau &&
            marker_poseinfo.theta < 32)
            good_marker_locations.push_back(mi);
        mi.rt_f2m=marker_poseinfo.sols[1];
        all_marker_locations.push_back(mi);
//        good_marker_locations.push_back(mi);
    }


    ///方法一：求重投影误差（有效marker多于两个）
    if (markersOfFrameToBeUsed.size()>=2) {
        //collect all the markers 3d locations
        vector<cv::Point2f> markerPoints2d;
        vector<cv::Point3f> markerPoints3d;
        for(auto ml:all_marker_locations){
            auto p2d=frame.getMarker(ml.id);
            markerPoints2d.insert(markerPoints2d.end(),p2d.begin(),p2d.end());
            auto p3d=map_markers[ml.id].get3DPoints();
            markerPoints3d.insert(markerPoints3d.end(),p3d.begin(),p3d.end());
        }

        //take the all poses and select the one that minimizes the global reproj error
        vector<cv::Mat> allPoses;
        for(auto & ml:all_marker_locations){
            auto pose= ml.rt_f2m *map_markers[ml.id].pose_g2m.convert().inv();
            //now,  compute the repj error of all markers using this info
            ///将Marker的3D坐标转到2维坐标，并计算误差
            ml.err=reprj_error(markerPoints3d,markerPoints2d,frame.imageParams.undistorted(),  pose);
        }
        //sort and get the best
        std::sort(all_marker_locations.begin(),all_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
        _debug_msg("err="<<all_marker_locations.front().err,10);
        auto &best=all_marker_locations.front();
        cout<<"relocalize best err: "<<best.err<<endl;
        if (best.err < 8)
        {
            pose_f2g_out=best.rt_f2m *map_markers[best.id].pose_g2m.convert().inv();
            //now, do a  finer optimization
            cv::Mat rv=pose_f2g_out.getRvec(),tv=pose_f2g_out.getTvec();
            cv::solvePnP(markerPoints3d,markerPoints2d,frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,true);
            pose_f2g_out=se3(rv,tv);
        }

        _debug_msg("err opt="<< reprj_error(markerPoints3d,markerPoints2d,frame.imageParams.undistorted(),  pose_f2g_out),10);
    }

    ///BIT ADD
    /*if (markersOfFrameToBeUsed.size()>=2)
    {
        ///1,collect all the markers 3d locations
        vector<cv::Point2f> markerPoints2d;
        vector<cv::Point3f> markerPoints3d;
        for(auto ml:all_marker_locations)
        {
            auto p2d=frame.getMarker(ml.id);
            markerPoints2d.insert(markerPoints2d.end(),p2d.begin(),p2d.end());
            auto p3d=map_markers[ml.id].get3DPoints();
            markerPoints3d.insert(markerPoints3d.end(),p3d.begin(),p3d.end());
        }
        ///2,
        for(auto &ml1 : all_marker_locations)
        {
            auto R1 = map_markers[ml1.id].pose_g2m.getRotation3x3();
            auto R1_m = ml1.rt_f2m.rowRange(0,3).colRange(0,3);
            for (auto ml2 : all_marker_locations)
            {
                auto R2 = map_markers[ml2.id].pose_g2m.getRotation3x3();
                if (ml1.id == ml2.id) continue;
                auto R2_m = ml2.rt_f2m.rowRange(0,3).colRange(0,3);

                cv::Mat deltaR = R1.t()*R2 - R1_m.t()*R2_m;
                ml1.err = cv::norm(deltaR);
            }
        }
        std::sort(all_marker_locations.begin(),all_marker_locations.end(),
                [](const minfo &a,const minfo &b){return a.err<b.err;});
        auto &best=all_marker_locations.front();
        ///3,
        cout<<"relocalize best err = "<<best.err<<endl;
        if (best.err < 0.3)
        {
            pose_f2g_out=best.rt_f2m *map_markers[best.id].pose_g2m.convert().inv();
            //now, do a  finer optimization
            cv::Mat rv=pose_f2g_out.getRvec(),tv=pose_f2g_out.getTvec();
            cv::solvePnP(markerPoints3d,markerPoints2d,frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,true);
            pose_f2g_out=se3(rv,tv);
        }
    }*/

    if ( pose_f2g_out.isValid()==false &&  good_marker_locations.size()>0){
        cout<<"one valid marker\n";
        std::sort(good_marker_locations.begin(),good_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
        auto best=good_marker_locations[0];
        ///BID ADD
//        float min_baseline = std::numeric_limits<float>::max();
//        for (auto sol : all_marker_locations)
//        {
//            se3 pose_f2g;
//            pose_f2g = sol.rt_f2m *map_markers.at(sol.id).pose_g2m.convert().inv();
//            float baseline = cv::norm(pose_f2g.getTvec(),keyframes.end()->pose_f2g.getTvec());
//            if (baseline < min_baseline)
//            {
//                min_baseline = baseline;
//                pose_f2g_out = pose_f2g;
//            }
//
//        }
        //estimate current location
        pose_f2g_out= best.rt_f2m *map_markers.at(best.id).pose_g2m.convert().inv();
//        cout<<"relocalize baseline = "<<min_baseline<<endl;
    }
    return   pose_f2g_out;

}
std::set<uint32_t> Map::getNeighborKeyFrames(uint32_t fidx,bool includeFidx){

    return TheKpGraph.getNeighbors(fidx,includeFidx);
}


void Map::saveToMarkerMap(std::string filepath)const {
       aruco::MarkerMap Mmap;
       std::string dict;
       if (map_markers.size()!=0)
           dict=map_markers.begin()->second.dict_info;
       Mmap.setDictionary(dict);
       Mmap.mInfoType=aruco::MarkerMap::METERS;
       for(const auto &mm:map_markers){
           const ucoslam::Marker &marker=mm.second;
           if (!marker.pose_g2m.isValid()) continue;
           aruco::Marker3DInfo m3di;
           m3di.id=marker.id;
           auto points3d=marker.get3DPoints();
           m3di.points.insert(m3di.points.end(),points3d.begin(),points3d.end());
           Mmap.push_back(m3di);

           cv::Mat T = cv::Mat::eye(4,4,CV_32F);
           cv::Mat Rvec = marker.pose_g2m.getRvec();
           cv::Mat t = marker.pose_g2m.getTvec().t();
           cv::Mat R;
           cv::Rodrigues(Rvec,R);
           R.copyTo(T.rowRange(0,3).colRange(0,3));
           t.copyTo(T.rowRange(0,3).col(3));
           Mmap.vT.emplace_back(T);

       }
       Mmap.saveToFile(filepath);
}


}
