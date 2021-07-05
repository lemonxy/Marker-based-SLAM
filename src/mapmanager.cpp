#include "mapmanager.h"
#include "map.h"
#include "slam.h"
#include "optimization/ippe.h"
#include "stuff/utils.h"
#include "stuff/io_utils.h"
#include "stuff/timers.h"
#include "optimization/globaloptimizer.h"
#include "optimization/loopclosure.h"
#include "optimization/solvepnp.h"
#include "stuff/minmaxbags.h"
#include <xflann/xflann.h>
#include <omp.h>
namespace ucoslam
{
    MapManager::MapManager(){
    _curState=IDLE;
 }
    MapManager::~MapManager(){
    stop();
}

    ///kf: 当前帧， curkeyFrame: 参考帧
    int MapManager::newFrame(Frame &kf, int32_t curkeyFrame  ){
    ScopedTimerEvents tevent("MapManager::newFrame");
    _counter++;
    _CurkeyFrame=curkeyFrame;

    if (  _curState.load()==IDLE ){

        ///Step1，回环检测(检测，回环优化，校正地图)
        _LoopClosureInfo=_TheLoopDetector.detectLoopFromMarkers(kf,curkeyFrame);
        if (_LoopClosureInfo.foundLoop())
        {
            ///BIT ADD*****
            cout<<"found loop!, loop frame: "<<_LoopClosureInfo.matchingFrameIdx<<
                ", loop marker: ";
            for(size_t i; i<_LoopClosureInfo.loopMarkers.size(); ++i)
                cout<<_LoopClosureInfo.loopMarkers[i]<<", ";
            cout<<endl;
            ///******
            _TheLoopDetector.correctMap(_LoopClosureInfo);
            loopClosurePostProcessing(kf,_LoopClosureInfo);//the keyframe is added here
            tevent.add("detectLoop");
            return 2;
        }

        tevent.add("detectLoop");
        ///Step2, 是否需要关键帧
        if ( mustAddKeyFrame(kf,curkeyFrame)   ){

                tevent.add("mustAddKeyFrame");
                Frame *newKF =new Frame(kf);
                keyframesToAdd.push(   newKF );
                //if the thread is not running, we work in synchronous mode
                if (!_TThread.joinable())
                    mainFunction();///局部优化
                tevent.add("mainFunction");
                return 1;
        }
        else{
            tevent.add("mainFunction");
        }
    }

    return 0;
}

    void MapManager::start(){
    if (_TThread.joinable()) return;//is running
    mustExit=false;
    _TThread= std::thread([this]{ this->runThread();});

}
    void MapManager::stop(){
    if (!_TThread.joinable()) return;//is not running
    mustExit=true;
    keyframesToAdd.push(NULL);//this will wakeup in mainFuntion if sleeping
    _TThread.join();
    _curState=IDLE;

}

    Frame& MapManager::addKeyFrame(Frame *newPtrFrame)
    {
        auto getNOFValidMarkers=[this](){
            int nValidMarkers=0;
            for(auto &m:TheMap->map_markers)
                if (m.second.pose_g2m.isValid()) nValidMarkers++;
            return nValidMarkers;
        };

        ///Step1: ADD KEYFRAME
        ScopedTimerEvents Timer("MapManager::addKeyFrame");
        Frame &newFrame=TheMap->addKeyFrame(*newPtrFrame);
        Timer.add("Add frame  ");
        ///Step2: 维护youngKeyFrames的大小，暂时不知道有什么用
        youngKeyFrames.insert({newFrame.idx,0});
        //remove old frames from here
        vector<uint32_t> toRemove;
        for(auto kf:youngKeyFrames){
            kf.second++;
            if (kf.second>3)  toRemove.push_back(kf.first);
        }
        for(auto r:toRemove) youngKeyFrames.erase(r);

        newFrame.nonMaximaSuppresion();
        Timer.add("NonMaximaSuppresion");

        ///Step3: add keyPoints
        //add to the set of unstable frames
        //UPDATE EXISTING MAP POINTS WITH OBSERVATIONS IN THIS FRAME
        for(size_t i=0;i<newFrame.ids.size();i++){
            if (newFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                  TheMap-> addMapPointObservation(newFrame.ids[i],newFrame.idx,i);
            }
        }

        //Analyze if the case in which the first marker of the scene is spotted. Then, the map would need to change scale
        _hasMapBeenScaled=false;
        bool mayNeedChangeMapScale=false;
        if( getNOFValidMarkers()==0 && TheMap->map_points.size()!=0) mayNeedChangeMapScale=true;

        ///Step4: ADD marker,计算新Markers的位姿
        for(size_t m=0;m< newFrame.markers.size();m++)
        {
            ///BIT ADD
//            if (newFrame.markers_solutions[m].theta > Slam::getParams().max_theta)
//                continue;
            //add marker if not yet
            auto &map_marker= TheMap->addMarker(newFrame.markers[m]);
            //add observation
            TheMap->addMarkerObservation(map_marker.id,newFrame.idx);
            //has the marker valid info already?
            ///方法一： 计算新Markers的位姿
            if (!map_marker.pose_g2m.isValid())
            {//no, see if it is possible to add the pose now with current data

//                cout<<"compute new marker pose, id: "<<map_marker.id<<endl;
                //if is not the first marker and is allowed oneframe initialization
                if (!mayNeedChangeMapScale  &&
                    Slam::getParams().aruco_allowOneFrameInitialization )  {
                    //Is the detection reliable (unambiguos)? If so, assign pose
                    if( newFrame.markers_solutions[m].err_ratio> Slam::getParams().aruco_minerrratio_valid &&
//                        newFrame.markers_solutions[m].distR > 1.3*newFrame.markers_solutions[m].tau &&
                        newFrame.markers_solutions[m].theta < Slam::getParams().max_theta)
                    {
                        map_marker.pose_g2m = newFrame.pose_f2g.inv()*newFrame.markers_solutions[m].sols[0];
                        cout<<"Single, new marker id = "<<map_marker.id<<endl;
                    }
                }
            }

            auto FramesDist=[](Se3Transform &a,Se3Transform &b){
                auto ta=a(cv::Range(0,3),cv::Range(3,4));
                auto tb=b(cv::Range(0,3),cv::Range(3,4));
                return cv::norm(ta-tb);
            };
            //if the above method did not succed, can we do the same using multiple views????
            ///方法二
            if (!map_marker.pose_g2m.isValid() &&
                map_marker.frames.size()>=size_t(Slam::getParams().aruco_minNumFramesRequired) )
            {
                //check how many views with enough distance are
//                vector<uint32_t> vframes( map_marker.frames.begin(),map_marker.frames.end());
                ///BIT ADD
                vector<uint32_t> vframes;
                for (auto f : map_marker.frames)
                {
                    for (size_t m = 0; m < TheMap->keyframes[f].markers.size(); m++)
                    {
                        if (TheMap->keyframes[f].markers[m].id == map_marker.id)
                        {
                            if (TheMap->keyframes[f].markers_solutions[m].theta < Slam::getParams().max_theta)
                                vframes.emplace_back(f);
                        }
                    }
                }
                if (vframes.size() < size_t(Slam::getParams().aruco_minNumFramesRequired)) continue;
                ///BIT ADD
               /* vector<uint32_t> vframes;
                for (auto f : map_marker.frames)
                {
                    for(size_t i = 0; i < TheMap->keyframes[f].markers.size(); i++)
                    {
                        if (TheMap->keyframes[f].markers[i].id != map_marker.id) continue;
                        if (TheMap->keyframes[f].markers_solutions[i].theta > Slam::getParams().max_theta) break;
                        vframes.push_back(f);
                    }
                }*/
                std::vector<bool> usedFrames(vframes.size(),false);
                std::vector<uint32_t> farEnoughViews;farEnoughViews.reserve(map_marker.frames.size());
                ///1，从帧组中找到距离合适的帧组，即排除比较近的帧
                for(size_t i=0; i<vframes.size(); i++){
                    //find farthest
                    if (!usedFrames[i]){
                        pair<int,float> best(-1,std::numeric_limits<float>::lowest());
                        for(size_t j=i+1;j<vframes.size();j++){
                            if( !usedFrames[j]){
                                float d=FramesDist( TheMap->keyframes[vframes[i]].pose_f2g,
                                                    TheMap->keyframes[vframes[j]].pose_f2g);
                                double theta = computeTheta(TheMap->keyframes[vframes[i]],
                                                            TheMap->keyframes[vframes[j]]);
//                                double theta = computeDeltaTheta(TheMap->keyframes[vframes[i]].markers,
//                                                                 TheMap->keyframes[vframes[j]].markers,
//                                                                 TheMap->keyframes[vframes[i]].imageParams.undistorted(),
//                                                                 Slam::getParams().aruco_markerSize);
                                if (    (d>Slam::getParams().minBaseLine+0.03 || theta > Slam::getParams().delta_theta) &&
                                        d> best.first)
                                    best={j,d};
                            }
                        }
                        if (best.first!=-1){
                            assert(std::find(farEnoughViews.begin(),farEnoughViews.end(),vframes[i])==farEnoughViews.end());
                            assert(std::find(farEnoughViews.begin(),farEnoughViews.end(),vframes[best.first])==farEnoughViews.end());
                            farEnoughViews.push_back(vframes[i]);
                            farEnoughViews.push_back(vframes[best.first]);
                            usedFrames[i]=true;
                            usedFrames[best.first]=true;
                        }
                    }
                }
                ///2,观测到该Marker的帧必须>3个以
                if (farEnoughViews.size()>=size_t(Slam::getParams().aruco_minNumFramesRequired)){//at least x views to estimate the pose using multiple views

                    vector<aruco::Marker > marker_views;
                    vector<double> vmarker_thetas;
                    vector<se3> frame_poses;
                    ////BIT ADD****
                    vector<vector<se3> > vvMarker_poses;
                    vector<vector<aruco::Marker> > vvMapMarker_views;
                    vector<uint32_t> frameFseqId;
                    ///****
                    int valid_marker = 0;
                    for(auto f:farEnoughViews){
                        marker_views.push_back(TheMap->keyframes[f].getMarker(map_marker.id));
                        frame_poses.push_back(TheMap->keyframes[f].pose_f2g);
                        double theta = TheMap->keyframes[f].getMarkerPoseIPPE(map_marker.id).theta;
                        vmarker_thetas.push_back(theta);

                        ////BIT ADD****
                        frameFseqId.emplace_back(TheMap->keyframes[f].fseq_idx);
                        vector<aruco::Marker> vMarkers = TheMap->keyframes[f].markers;
                        vector<se3> vMapMarker_poses;
                        vector<aruco::Marker> vMapMarker_views;
                        for (auto m : vMarkers)
                        {
                            if (TheMap->map_markers.find(m.id) == TheMap->map_markers.end()) continue;
                            Marker map_marker_i = TheMap->map_markers[m.id];
                            if (!map_marker_i.pose_g2m.isValid()) continue;
                            if (map_marker.id == map_marker_i.id) continue;
                            vMapMarker_poses.push_back(map_marker_i.pose_g2m);
                            vMapMarker_views.push_back(TheMap->keyframes[f].getMarker(m.id));
                            valid_marker++;
                        }
                        vvMarker_poses.push_back(vMapMarker_poses);
                        vvMapMarker_views.push_back(vMapMarker_views);
                        ///****
                    }
                    ///BIT ADD
                    float tau_theta = Slam::getParams().max_theta;
                    if (valid_marker != 0)
                    {
                        auto pose=ARUCO_bestMarkerPose_BIT(marker_views,vvMarker_poses,
                                                           farEnoughViews, frameFseqId,
                                                           vvMapMarker_views,
                                                           frame_poses, newFrame.imageParams.undistorted(), map_marker.size,
                                                           vmarker_thetas,
                                                           tau_theta);
                        ///******
                        if (!pose.empty()){
                            _debug_msg_("added marker "<<map_marker.id<<" using multiple views");
                            cout<<"Multi, new marker id = "<<map_marker.id<<endl;
                            map_marker.pose_g2m=pose;
                        }
                    }
                    /*if (pose.empty())
                    {
                        auto pose1=ARUCO_bestMarkerPose(marker_views,frame_poses,newFrame.imageParams.undistorted(), map_marker.size);
                        pose1.copyTo(pose);
                    }*/

                }
            }
        }
        /////////////////////////////////////////////////////////////////////////////
        ///     MAP RESCALING BECAUSE OF NEW MARKER FOUND
        //if there were no valid markers before, but points, and a new marker is found,
        //then, there is the need to establish
        if (mayNeedChangeMapScale&& getNOFValidMarkers()>0){
                //the pose of a marker has been established. It is required to scale the map by
                //findding correspondences between the marker system and the points
                //to do so, analyze the points into the marker. They are employed to scale
            //find the 3d points into the marker in this image
            pair<double,double> avrg_scale(0,0);
            for(auto &m:newFrame.markers){
                auto &mapMarker=TheMap->map_markers.at(m.id);
                if (!mapMarker.pose_g2m.isValid()) continue;
                //determine center and maximum distance to it
                cv::Point2f center(0,0);
                for(auto p:m)center+p;
                center*=1./4.;
                //now max dist
                double maxDist=std::numeric_limits<double>::min();
                for(auto p:m)   maxDist=std::max( cv::norm(center-p),maxDist);
                vector<uint32_t> p3dis=newFrame.getIdOfPointsInRegion(center,maxDist);
                if ( p3dis.size()<5)continue;//too few points
                //get the scale
                //get the average distance to the camera
                double distSum=0;
                for(auto pid:p3dis){
                    //move the point to the frame and compute distance to camera
                    distSum+=   cv::norm( newFrame.pose_f2g*TheMap->map_points[pid].getCoordinates());
                }
                double avrgPointDist=distSum/double(p3dis.size());
                //compute the distance of the marker to the frame
                cv::Mat f2m=newFrame.pose_f2g*mapMarker.pose_g2m.convert();
                double frameDist=cv::norm(  f2m.rowRange(0,3).colRange(3,4));
                avrg_scale.first+=frameDist/avrgPointDist;
                avrg_scale.second++;
            }

            if ( avrg_scale.second==0){//cant scale because no evidences found. Remove pose of the markers
                for(auto &m:TheMap->map_markers)
                    m.second.pose_g2m=se3();
            }
            else{//reescale the whole map
                // Scale points
                double scaleFactor=avrg_scale.first/avrg_scale.second;
                for(auto &mp:TheMap->map_points)
                   mp.scalePoint(scaleFactor);
               //do the same with the frame locations
                for(auto &frame:TheMap->keyframes){
                    cv::Mat t=frame.pose_f2g.rowRange(0,3).colRange(3,4);
                    t*=scaleFactor;
                }
                globalOptimization(10);
                _hasMapBeenScaled=true;

            }
        }

        return newFrame;
}

    ///局部优化
    void MapManager::mainFunction(){
     //first check if any new frame to be inserted
    Frame    *newPtrFrame;
    keyframesToAdd.pop(newPtrFrame);


    if(newPtrFrame==NULL) return;//a NULL frame mean, leave
    _curState=WORKING;


    ///ADD KEYFRAME

    ScopedTimerEvents Timer("MapManager::mainFunction");
    TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
    Frame &newFrame=addKeyFrame(newPtrFrame);
    _lastAddedKeyFrame=newFrame.idx;
    _debug_msg_("");
    delete newPtrFrame;
    Timer.add("add keyframe");

    ///Step1: 检测回环
    _LoopClosureInfo=_TheLoopDetector.detectLoopFromKeyPoints(newFrame,_CurkeyFrame);
    Timer.add("loop detection ");

    _debug_msg_("");
    Timer.add("update existing points ");
    TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);
  //  assert(TheMap->checkConsistency(true ));
    _debug_msg_("");


    ///Step2: 融合特征点
    PointsToRemove=mapPointsCulling( );
    TheMap->removePoints(PointsToRemove.begin(),PointsToRemove.end(),false);//preremoval
    Timer.add("map point culling and removal");

    TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
    /// Step3：增加新特征点
    if ( newFrame.imageParams.isStereoCamera()){
        for(const MapPoint &nmp:createCloseStereoPoints(newFrame,10)){
            MapPoint &newmp=TheMap->addNewPoint(nmp);
            newmp.creationSeqIdx=_counter;
        }
    }

    for(const MapPoint &nmp:createNewPoints(newFrame,20)){
        MapPoint &newmp=TheMap->addNewPoint(nmp);
        newmp.creationSeqIdx=_counter;
    }
    TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);

    Timer.add("Add   points " );
     if (keyframesToAdd.empty()){
        TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
        auto ptremove=searchInNeighbors(newFrame);
        PointsToRemove.insert(PointsToRemove.end(),ptremove.begin(),ptremove.end());
        Timer.add("Search in neighbors ");
        TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);
      //  assert(TheMap->checkConsistency( ));
    }

    //---------------------------------------------------------
    // UNLOCK
    //---------------------------------------------------------


    Timer.add("map point culling  analysis");
     ///Step4: 局部优化
    if (keyframesToAdd.empty() && TheMap->keyframes.size()>1 ){
        localOptimization(newFrame.idx);
        Timer.add("Local optimization");
    }
    //keyframe culling
        KeyFramesToRemove=keyFrameCulling(newFrame.idx);
        Timer.add("keyframe culling  analysis");
     Timer.add("mapUpdate");


    _curState=WAITINGFORUPDATE;
  //  assert(TheMap->checkConsistency());

}

    Se3Transform MapManager::mapUpdate()
    {
        Se3Transform ReturnValue;
        if (_curState!=WAITINGFORUPDATE) return  ReturnValue;
        _curState=WORKING;



        TheMap->lock(__FUNCTION__,__FILE__,__LINE__);

        if (_LoopClosureInfo.foundLoop()){
            //get the detection and correct the map
            _TheLoopDetector.correctMap(_LoopClosureInfo);
            loopClosurePostProcessing(TheMap->keyframes[_lastAddedKeyFrame],_LoopClosureInfo  );
            ReturnValue=TheMap->keyframes[_lastAddedKeyFrame].pose_f2g;
        }
        else{
            vector<std::pair<uint32_t,uint32_t>> BadAssociations;
            if (Gopt){//if we jsut wake up, from a loadFromStream, this object is not created
                Gopt->getResults(TheMap);
                BadAssociations=Gopt->getBadAssociations();
                Gopt=nullptr;
            }
            TheMap->removeBadAssociations(BadAssociations,Slam::getParams().minNumProjPoints);
        }
        //complete point removal
        TheMap->removePoints(PointsToRemove.begin(),PointsToRemove.end(),true);
        PointsToRemove.clear();
        TheMap->removeKeyFrames(KeyFramesToRemove,Slam::getParams().minNumProjPoints);
        for(auto kf:KeyFramesToRemove) youngKeyFrames.erase(kf);

        if (_hasMapBeenScaled)
            ReturnValue=TheMap->keyframes[_lastAddedKeyFrame].pose_f2g;

        TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);
        //---------------------------------------------------------
        // UNLOCK
        //---------------------------------------------------------
        //  assert(TheMap->checkConsistency());
        PointsToRemove.clear();
        KeyFramesToRemove.clear();
        _curState=IDLE;
        return ReturnValue;

}

    void MapManager::runThread(){


    while(!mustExit){       
        mainFunction();
    }
}

    ///关键帧融合
    set<uint32_t> MapManager::keyFrameCulling(uint32_t keyframe_idx)
    {
        ScopedTimerEvents timer("MapManager::keyFrameCulling");

        set<uint32_t> toremove;
        toremove= keyFrameCulling_Markers(keyframe_idx);
        timer.add("Detected markers to remove");

        return toremove;

}
    ///关键帧融合_markers
    set<uint32_t> MapManager::keyFrameCulling_Markers(uint32_t keyframe_idx)
    {

        auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };
        //auto separe=[](uint64_t a_b){  uint32_t *_a_b_16=(uint32_t*)&a_b;return  make_pair(_a_b_16[1],_a_b_16[0]);};

        ///Step1: 找出相邻关键帧组
        auto neigh=TheMap->TheKpGraph.getNeighbors(keyframe_idx);
        //remove first frame if it is in
        ///Step2: 移除第一帧（共视程度较高）
        neigh.erase(TheMap->keyframes.front().idx);

        std::map<uint64_t,float> frame_distances;
        ///Step3：计算两两帧之间的距离
        vector<uint32_t> vneigh(neigh.begin(),neigh.end());
        for(size_t i=0;i<vneigh.size();i++){
        const auto&fi=TheMap->keyframes[vneigh[i]];
        for(size_t j=i+1;j<vneigh.size();j++){
            const auto&fj=TheMap->keyframes[vneigh[j]];
          //  frame_distances[join(vneigh[i],vneigh[j])]= fi.pose_f2g.t_dist(fj.pose_f2g);
            frame_distances[join(vneigh[i],vneigh[j])]= cv::norm( fi.pose_f2g.getTvec() -fj.pose_f2g.getTvec());
            }
    }

        ///determine the set of markers visible in the neighbors
        std::map<uint32_t,set<uint32_t> > marker_frames;//for each visible marker, the views it is seen it
        for(auto fidx:neigh){
        for(auto m:TheMap->keyframes[fidx].und_markers)
            marker_frames[m.id].insert(fidx);
    }

        auto distanceToFrames=[&](uint32_t fidx,const set<uint32_t> &frames){
            float dist=0;
            for(auto f2idx: frames){
                if (f2idx!=fidx )
                    dist+=frame_distances[join(fidx,f2idx)];
            }
            return dist;
    };

        ///for each marker, select a subset of frames (the most equdistant ones)
        std::map<uint32_t,set<uint32_t> > marker_selected_frames;//for each visible marker, the views it is seen it
        for(auto mf:marker_frames){
        if (  mf.second.size()<=size_t(Slam::getParams().maxVisibleFramesPerMarker)){
            marker_selected_frames[mf.first].insert( mf.second.begin(),mf.second.end());
        }
        else{
            //how many will be selected? (70%)
            //find the first two frames which are most equidistant
            vector<uint32_t> vframes(mf.second.begin(),mf.second.end());
            pair<size_t,size_t> bestIdx;float maxD=std::numeric_limits<float>::lowest();
            for(size_t i=0;i<vframes.size();i++){
                for(size_t j=i+1;j<vframes.size();j++){
                    auto dist= frame_distances[ join(vframes[i],vframes[j]) ];
                    if ( dist>maxD){
                        bestIdx={vframes[i],vframes[j]};
                        maxD=dist;
                    }
                }
            }
            //the best ones are in bestIdx
            marker_selected_frames[mf.first]. insert( bestIdx.first);
            marker_selected_frames[mf.first]. insert( bestIdx.second);
            //now, keep adding. The next is always the farthest from the current elements in the set
            while(marker_selected_frames[mf.first].size()<size_t(Slam::getParams().maxVisibleFramesPerMarker)){
                std::pair<uint32_t,float> best(0, std::numeric_limits<float>::lowest());
                for(size_t i=0;i<vframes.size();i++){
                    if ( marker_selected_frames[mf.first].count(vframes[i])==0){
                        auto d=distanceToFrames(vframes[i],marker_selected_frames[mf.first]);
                        if (d>best.second) best={vframes[i],d};
                    }
                }
                //add it
                assert( std::find(marker_selected_frames[mf.first].begin(),marker_selected_frames[mf.first].end(),best.first)==marker_selected_frames[mf.first].end());
                marker_selected_frames[mf.first]. insert(best.first);
            }
        }

    }

        ///BIT ADD****
        /*std::map<uint64_t,double> frame_theta;
     vector<uint32_t> vneigh_theta(neigh.begin(),neigh.end());
     for(size_t i=0;i<vneigh.size();i++){
         const auto&fi=TheMap->keyframes[vneigh[i]];
         for(size_t j=i+1;j<vneigh.size();j++){
             const auto&fj=TheMap->keyframes[vneigh[j]];
             frame_theta[join(vneigh[i],vneigh[j])]=
                     computeDeltaTheta(fi.markers,fj.markers,fi.imageParams.undistorted(),
                                       Slam::getParams().aruco_markerSize);
         }
     }
     std::map<uint32_t,set<uint32_t> > marker_frames_theta;
     for(auto fidx:neigh){
         for(auto m:TheMap->keyframes[fidx].und_markers)
             marker_frames_theta[m.id].insert(fidx);
     }
     auto thetaToFrames=[&](uint32_t fidx,const set<uint32_t> &frames){
         float dist=0;
         for(auto f2idx: frames){
             if (f2idx!=fidx )
                 dist+=frame_theta[join(fidx,f2idx)];
         }
         return dist;
     };
     std::map<uint32_t,set<uint32_t> > marker_selected_frames_theta;//for each visible marker, the views it is seen it
     for(auto mf:marker_frames_theta){
         if (  mf.second.size()<=size_t(Slam::getParams().maxVisibleFramesPerMarker)){
             marker_selected_frames_theta[mf.first].insert( mf.second.begin(),mf.second.end());
         }
         else
         {
             //Step5.1 :添加最远的两帧
             vector<uint32_t> vframes(mf.second.begin(),mf.second.end());
             pair<size_t,size_t> bestIdx;float maxD=std::numeric_limits<float>::lowest();
             for(size_t i=0;i<vframes.size();i++){
                 for(size_t j=i+1;j<vframes.size();j++){
                     auto dist= frame_distances[ join(vframes[i],vframes[j]) ];
                     if ( dist>maxD){
                         bestIdx={vframes[i],vframes[j]};
                         maxD=dist;
                     }
                 }
             }
             //the best ones are in bestIdx
             marker_selected_frames_theta[mf.first]. insert( bestIdx.first);
             marker_selected_frames_theta[mf.first]. insert( bestIdx.second);
             //now, keep adding. The next is always the farthest from the current elements in the set
             //Step5.2： 以Step5.1选择的两帧中的第一帧为基准，从该Markers的观测中找距离最远的帧，添加到marker_selected_frames中，
             //直到数量>10为止
             while(marker_selected_frames_theta[mf.first].size()<size_t(Slam::getParams().maxVisibleFramesPerMarker))
             {
                 std::pair<uint32_t,float> best(0, std::numeric_limits<float>::lowest());
                 for(size_t i=0;i<vframes.size();i++){
                     if ( marker_selected_frames_theta[mf.first].count(vframes[i])==0){
                         auto theta=thetaToFrames(vframes[i],marker_selected_frames_theta[mf.first]);
                         if (theta>best.second) best={vframes[i],theta};
                     }
                 }
                 marker_selected_frames_theta[mf.first]. insert(best.first);
             }
         }

     }*/
        ///***********
        //now, let us remove the elements not in the set of selected
        std::set<uint32_t> selected;
        for(auto ms:marker_selected_frames)
            selected.insert(ms.second.begin(),ms.second.end());
        //remove the rest
        std::set<uint32_t> toremove;
        for(auto fidx:neigh)    //remove neighbors not in the set
             if( selected.count(fidx)==0) toremove.insert(fidx);

        for(auto m:toremove)
            cout<<"delete: "<<m<<" ";
        cout<<endl;
        return toremove;
    }

    set<uint32_t> MapManager::keyFrameCulling_KeyPoints(uint32_t keyframe_idx)
    {
        set<uint32_t> framesToRemove;

        if (keyframe_idx<0)throw std::runtime_error("invalid _curKFRef");
        if (TheMap->keyframes.size()<size_t(Slam::getParams().minNumProjPoints)) return {};
        auto firstFrame=TheMap->keyframes.front().idx;
        //find the neightbors
        auto neigh=TheMap->TheKpGraph.getNeighbors(keyframe_idx);
        neigh.erase(firstFrame);///exclude first frame
        //exlcude too young frames
        for(auto ykf:youngKeyFrames)
         neigh.erase(ykf.first);



        int thresObs=Slam::getParams().minNumProjPoints;
        for(auto fidx:neigh){
         int nRedundant=0,nPoints=0;
          auto &frame=TheMap->keyframes[fidx];
         if (frame.isBad )continue;

         for(size_t i=0;i<frame.ids.size();i++){
             if (frame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                 auto &mp=TheMap->map_points[frame.ids[i]];
                 if (mp.isBad)continue;//to be removed
                 nPoints++;
                 //check point projections in the other frames and see if this is redundant
                 int nObs=0;//number of times point observed in a finer scale in another frame
                 if( mp.getNumOfObservingFrames()>size_t(thresObs)){
                     for(const auto &f_i:mp.getObservingFrames()){
                         if (f_i.first!=fidx && !TheMap->keyframes[f_i.first].isBad )//not in a frame to be removed
                             if  ( TheMap->keyframes[f_i.first].und_kpts[f_i.second].octave<=frame.und_kpts[i].octave){
                                 nObs++;
                                 if(nObs>=thresObs)   break;
                             }

                     }
                 }
                 if (nObs>=thresObs)
                     nRedundant++;
             }
         }

         float redudantPerc=float(nRedundant)/float(nPoints);
         _debug_msg_("frame :"<<fidx<<" npoints:"<<nPoints<<" redundant="<<redudantPerc*100<<"%");
         if(redudantPerc>Slam::getParams().keyFrameCullingPercentage ){
             _debug_msg_("remove frame:"<<fidx);
             framesToRemove.insert(fidx);
            frame.isBad=true;
         }
        }
        return framesToRemove;
 }

    vector<uint32_t> MapManager::mapPointsCulling(  )
 {
     std::vector<uint32_t> pointsToRemove;

     for(auto &mp:TheMap->map_points) {
         if (!mp.isStable && !mp.isBad){
             uint32_t obsths=std::min(uint32_t(3),TheMap->keyframes.size());
             if (mp.isStereo)
                 obsths=std::min(uint32_t(2),TheMap->keyframes.size());


             if (mp.getVisibility()<0.25 ) mp.isBad=true;
             else if ( mp.kfSinceAddition>=1 && mp.getNumOfObservingFrames()<obsths ) mp.isBad=true;
             else if( mp.kfSinceAddition>=3) mp.isStable=true;
             if ( mp.kfSinceAddition<5) mp.kfSinceAddition++;
         }


         if (mp.isStable)
             if (mp.getVisibility()<0.1 ) mp.isBad=true;
         if(mp.isBad) pointsToRemove.push_back(mp.id);

     }
     _debug_msg_("mapPointsCulling remove:"<<pointsToRemove.size());
     return pointsToRemove;
 }

    ///关键帧插入规则
    bool MapManager::mustAddKeyFrame(const Frame & frame_in ,uint32_t curKFRef)
 {

     bool reskp=false,resm=false,resrgbd=false;
     if (frame_in.imageParams.isStereoCamera() )
         resrgbd=mustAddKeyFrame_stereo(frame_in,curKFRef );


     if (!reskp & Slam::getParams().detectMarkers)
         resm= mustAddKeyFrame_Markers(frame_in ,curKFRef);

     //must add the frame info
     if (!(reskp || resm||resrgbd))
         return false;
     return true;


 }
    bool MapManager::mustAddKeyFrame_stereo(const Frame &frame_in,uint32_t curKFRef){

     if (!frame_in.imageParams.isStereoCamera()) return false;
     // Check how many "close" points are being tracked and how many could be potentially created.
     int nNonTrackedClose = 0;
     int nTrackedClose= 0;

     //count how many close points are seen
     for(size_t i=0;i<frame_in.und_kpts.size();i++){
         if (frame_in.depth[i]>0 && frame_in.imageParams.isClosePoint(frame_in.depth[i])){
             if (frame_in.ids[i]!=std::numeric_limits<uint32_t>::max())
                 nTrackedClose++;
             else nNonTrackedClose++;
         }
     }
      if ((nTrackedClose<100) && (nNonTrackedClose>70))//bNeedToInsertClose
         return true;



     //a minimum number of matches valid
     int mnMatchesInliers=0;
     for(auto id:frame_in.ids)
         if (id!=std::numeric_limits<uint32_t>::max()) mnMatchesInliers++;
     //// AT LEAST 20 matches
     if (mnMatchesInliers<=15){
         _debug_msg_("nValidMatches<15");
         return false;
     }

     uint32_t minObs=std::min(uint32_t(3),TheMap->keyframes.size());
     //number of points  in current keyframe
     int nRefMatches=0;
     for(auto id:TheMap->keyframes[curKFRef].ids)
         if (id!=std::numeric_limits<uint32_t>::max())
             if (TheMap->map_points[id].frames.size()>=minObs) nRefMatches++;



     _debug_msg_("MustAddKeyFrame mnMatchesInliers="<<mnMatchesInliers<<" ref ratio="<<nRefMatches*Slam::getParams().thRefRatio);



      if ( mnMatchesInliers<nRefMatches*Slam::getParams().thRefRatio)
         return true;

      //check distance to nearest neighbor
      float mind=std::numeric_limits<float>::max();
      for(auto n:TheMap->TheKpGraph.getNeighbors(curKFRef,true)){
          float dist=cv::norm( TheMap->keyframes[n].getCameraCenter()-frame_in.getCameraCenter());
          mind=std::min(mind,dist);
      }
      _debug_msg_("MustAddKeyFrame mind="<<mind);
      if ( mind>=Slam::getParams().minBaseLine/2 && mind!=std::numeric_limits<float>::max()){
          _debug_msg_("Added kf by distance !!");
          return true;
      }

 //    if (TheMap->keyframes[curKFRef].getCameraDirection().dot(frame_in.getCameraDirection())<0.9 ){
 //        _debug_msg_("Added kf by angle !!");
 //        return true;
 //    }


     return false;


 }
    ///关键帧插入规则_marker
    bool MapManager::mustAddKeyFrame_Markers(const Frame & frame_in ,uint32_t curKFRef)
    {
        ///condition a: 有新Marker就添加为关键帧
        int i = 0;
        for(auto m:frame_in.und_markers)
        {
            if (TheMap->map_markers.count (m.id)==0)
            {
                /*cout<<"condition a\n";
                return true;*/
                ///BIT ADD
                if(frame_in.markers_solutions[i].theta < Slam::getParams().max_theta)
                {
                    cout<<"condition a\n";
                    return true;
                }
            }
            i++;
        }


        ///conditon b: 地图中有Markers位姿无效，而该帧的Marker位姿歧义大于阈值，该阈值被设为3
        /*for(auto m:frame_in.und_markers){
         if (TheMap->map_markers.count (m.id)!=0) {//is in the map
             if( !TheMap->map_markers [ m.id].pose_g2m.isValid()  )
             {
                ///BIT ADD
                if ( frame_in.getMarkerPoseIPPE(m.id).err_ratio>Slam::getParams().aruco_minerrratio_valid &&
                        Slam::getParams().aruco_allowOneFrameInitialization &&
                        frame_in.getMarkerPoseIPPE(m.id).distR > Slam::getParams().min_distR &&
                        frame_in.getMarkerPoseIPPE(m.id).theta < Slam::getParams().max_theta)
                {
                    cout<<"condition b\n"<<endl;
                    return true;
                }
                     //adding this will set a valid location for m.id
             }//with invalid pose

         }
     }*/

        ///condition c: 与参考关键帧的基线大于阈值
        cout<<"Ref: "<<curKFRef<<endl;
        float baseLine=cv::norm(frame_in.pose_f2g.getTvec(),TheMap->keyframes[curKFRef].pose_f2g.getTvec());
        //&& baseLine < 10*Slam::getParams().minBaseLine
        if ( baseLine> Slam::getParams().minBaseLine )
        {
            cout<<"condition c \n";
            cout<<"baseline = "<<baseLine<<endl;
            return true;
        }


        ///bit condition d:
        cout<<"tau: "<<Slam::getParams().delta_theta<<endl;
        cout<<"frameId = "<<frame_in.fseq_idx<<",Ref frameId = "
            <<TheMap->keyframes[curKFRef].fseq_idx<<endl;

        double deltaTheta = computeTheta(frame_in, TheMap->keyframes[curKFRef]);
//        double deltaTheta = computeDeltaTheta(frame_in.markers,
//                                        TheMap->keyframes[curKFRef].markers,
//                                        frame_in.imageParams.undistorted(),
//                                        Slam::getParams().aruco_markerSize);
        cout<<"deltaTheta: "<<deltaTheta<<endl;
        if (deltaTheta > Slam::getParams().delta_theta)
        {
            cout<<"delta_theta: "<<deltaTheta<<endl;
            return true;
        }
        //check if rotation to any is larger than thres
 //    float rotation=cv::norm(frame_in.pose_f2g.getRvec(),TheMap->keyframes[curKFRef].pose_f2g.getRvec());
 //    if ( rotation> Slam::getParams().aruco_maxRotation)
 //        return true;
         return false;
 }

    bool MapManager::mustAddKeyFrame_KeyPoints(const Frame &frame_in, uint32_t curKFRef )
 {

     //a minimum number of matches valid
     int mnMatchesInliers=0;
     for(auto id:frame_in.ids)
         if (id!=std::numeric_limits<uint32_t>::max()) mnMatchesInliers++;
     //// AT LEAST 20 matches
     if (mnMatchesInliers<20){
         _debug_msg_("nValidMatches<20");
         return false;
     }

     size_t minObs=3;
     if ( TheMap->keyframes.size()<3) minObs=TheMap->keyframes.size();
     //number of points  in current keyframe
     int nRefMatches=0;
     for(auto id:TheMap->keyframes[curKFRef].ids)
         if (id!=std::numeric_limits<uint32_t>::max())
             if (TheMap->map_points[id].frames.size()>=minObs) nRefMatches++;


     //check distance to nearest neighbor
     float mind=std::numeric_limits<float>::max();
     for(auto n:TheMap->TheKpGraph.getNeighbors(curKFRef,true)){
         float dist=cv::norm( TheMap->keyframes[n].getCameraCenter()-frame_in.getCameraCenter());
         mind=std::min(mind,dist);
     }
     _debug_msg_("MustAddKeyFrame mind="<<mind);
     if ( mind>=Slam::getParams().minBaseLine && mind!=std::numeric_limits<float>::max()){
         _debug_msg_("Added kf by distance !!");
         return true;
     }

     _debug_msg_("MustAddKeyFrame mnMatchesInliers="<<mnMatchesInliers<<" ref ratio="<<nRefMatches*Slam::getParams().thRefRatio);
     if ((mnMatchesInliers<nRefMatches*Slam::getParams().thRefRatio) && mnMatchesInliers>15)
         return true;




     //if distance to ref is higher than or the angle is larger than, then create a add
 //    if ( TheMap->map_markers.size()!=0 &&  cv::norm(TheMap->keyframes[curKFRef].getCameraCenter()-frame_in.getCameraCenter())>=Slam::getParams().aruco_minBaseLine){
 //        _debug_msg_("Added kf by distance !!");
 //        return true;
 //    }

     if (TheMap->keyframes[curKFRef].getCameraDirection().dot(frame_in.getCameraDirection())<0.9 ){
         _debug_msg_("Added kf by angle !!");
         return true;
     }


     return false;


 }

    vector<uint32_t> MapManager::getMatchingFrames(Frame &NewFrame,size_t n)    {

     //if there is only one frame (plus the new one), use it. We are coming from arucoInitializeFromSingleView
     if (TheMap->keyframes.size()<=2)
         return {TheMap->keyframes.front().idx};

     ScopedTimerEvents timer("MapManager::getMatchingFrames");


     vector<uint32_t> neighbors =TheMap->TheKpGraph.getNeighborsV(NewFrame.idx);//finally, the neighbors
     timer.add("getNeighborsV");


      std::sort(neighbors.begin(),neighbors.end(),[&](uint32_t a, uint32_t b){return
                 TheMap->TheKpGraph.getWeight(a,NewFrame.idx)>TheMap->TheKpGraph.getWeight(b,NewFrame.idx);});

     timer.add("sort");
     vector<uint32_t> goodframes;
     for(auto n: neighbors){
         auto medianDepth=TheMap->getFrameMedianDepth(n);
         auto baseline= cv::norm(NewFrame.getCameraCenter()-TheMap->keyframes[n].getCameraCenter());
         if(baseline/medianDepth> Slam::getParams().baseline_medianDepth_ratio_min  ) goodframes.push_back(n);
         if (goodframes.size()==n )break;
     }

     timer.add("median depth");

     return goodframes;
 }

    vector<uint32_t> MapManager::searchInNeighbors(Frame &mpCurrentKeyFrame  ){
      auto vpNeighKFs =TheMap->TheKpGraph.getNeighborsV(mpCurrentKeyFrame .idx    ); //getMatchingFrames(mpCurrentKeyFrame,20);
 vector<uint32_t> points2Remove;
     auto pointInfo=[&](uint32_t pid){
         MapPoint &mp=TheMap->map_points[pid];
       cout<<"point "<<mp.id<<":"<<mp.creationSeqIdx<< endl;
       for(auto f:mp.frames)
           cout<<"   f:"<<f.first<<":"<<f.second<<" pt="<<TheMap->keyframes[f.first].und_kpts[f.second].pt<<" oct="<<TheMap->keyframes[f.first].und_kpts[f.second].octave;
       cout<<endl;
     };

     set<uint32_t> vpTargetKFs;

     for(auto n: vpNeighKFs)
     {
         if (TheMap->keyframes[n].isBad) continue;
         vpTargetKFs.insert(n);
 //        // Extend to some second neighbors
 //        vector<uint32_t> vpSecondNeighKFs = getMatchingFrames(TheMap->keyframes[n],5);
 //        for(auto n2:vpSecondNeighKFs){
 //            if (!TheMap->keyframes[n2].isBad && n2!=mpCurrentKeyFrame.idx)
 //                vpTargetKFs.insert(n2);
 //        }
     }

     int nAddedObs=0,nFused=0;



     float th=3;
     // Search matches by projection from current KF in target KFs
     vector<uint32_t> vpMapPointMatches = mpCurrentKeyFrame.getMapPoints();
     for(auto tkf: vpTargetKFs){
         Frame &keyframe=TheMap->keyframes[tkf];
         float mLogScaleFactor=log(keyframe.getScaleFactor());
         cv::Point3f camCenter=keyframe.getCameraCenter();
         for(auto MpId:vpMapPointMatches){
             if ( !TheMap->map_points.is(MpId))continue;
             MapPoint&MP=TheMap->map_points[MpId];
             if (MP.isBad)continue;
             if ( MP.frames.count(keyframe.idx))continue;//already projected
             //project the point
             cv::Point2f p2d= keyframe.project(MP.getCoordinates(),true,true);
             if(isnan(p2d.x))continue;
             float dist= cv::norm(camCenter-MP.getCoordinates());
             if (dist<MP.getMinDistanceInvariance() || dist>MP.getMaxDistanceInvariance())continue;
             //view angle
             if( MP.getViewCos(camCenter)<0.7)continue;
             //ok, now projection is safe
             int nPredictedLevel = MP.predictScale( dist,mLogScaleFactor,keyframe.scaleFactors.size());

             // Search in a radius
             const float radius = th*keyframe.scaleFactors[nPredictedLevel];
             vector<uint32_t> vkpIdx=keyframe.getKeyPointsInRegion(p2d,radius,nPredictedLevel-1,nPredictedLevel);
             //find the best one
             pair<float,int>  best(Slam::getParams().minDescDistance+1e-3,-1);
             for(auto kpidx:vkpIdx){
                 float descDist=MP.getDescDistance( keyframe.desc.row(kpidx));
                 if ( descDist<best.first)
                     best={descDist,kpidx};
             }

             if (best.second!=-1){
                 if ( keyframe.ids[best.second]!=std::numeric_limits<uint32_t>::max()){
                     TheMap->fuseMapPoints(keyframe.ids[best.second],MP.id,false);//the point MP will be partially removed only
                     points2Remove.push_back(MP.id);
                     MP.isBad=true;
                     nFused++;

                 }
                 else{
                     TheMap->addKeyFrameObservation(MP.id,keyframe.idx,best.second);
                     nAddedObs++;

                 }
             }
         }
     }


     std::vector<uint32_t>  smap_ids=TheMap->getMapPointsInFrames(vpTargetKFs.begin(),vpTargetKFs.end());


     //now, analyze fusion the other way around
     //take the points in the target kframes and project them here
     float mLogScaleFactor=log(mpCurrentKeyFrame.getScaleFactor());
     cv::Point3f camCenter=mpCurrentKeyFrame.getCameraCenter();
     for(auto &mpid:smap_ids){
         auto &MP=TheMap->map_points[mpid];
         if (MP.isBad)continue;
         if(MP.isObservingFrame(mpCurrentKeyFrame.idx)) continue;
         cv::Point2f p2d= mpCurrentKeyFrame.project(MP.getCoordinates(),true,true);
         if(isnan(p2d.x))continue;
         float dist= cv::norm(camCenter-MP.getCoordinates());
         if (dist<MP.getMinDistanceInvariance() || dist>MP.getMaxDistanceInvariance())continue;
         //view angle
         if( MP.getViewCos(camCenter)<0.7)continue;
         //ok, now projection is safe
         int nPredictedLevel = MP.predictScale( dist,mLogScaleFactor,mpCurrentKeyFrame.scaleFactors.size());

         // Search in a radius
         const float radius = th*mpCurrentKeyFrame.scaleFactors[nPredictedLevel];
         vector<uint32_t> vkpIdx=mpCurrentKeyFrame.getKeyPointsInRegion(p2d,radius,nPredictedLevel-1,nPredictedLevel);
         //find the best one
         pair<float,int>  best(Slam::getParams().minDescDistance+1e-3,-1);
         for(auto kpidx:vkpIdx){
             float descDist=MP.getDescDistance( mpCurrentKeyFrame.desc.row(kpidx));
             if ( descDist<best.first)
                 best={descDist,kpidx};
         }

         if (best.second!=-1){
             if ( mpCurrentKeyFrame.ids[best.second]!=std::numeric_limits<uint32_t>::max()){
                 TheMap->fuseMapPoints(mpCurrentKeyFrame.ids[best.second],MP.id,false);
                 points2Remove.push_back(MP.id);
                 MP.isBad=true;
                 nFused++;
             }
             else{
                 TheMap->addKeyFrameObservation(MP.id,mpCurrentKeyFrame.idx,best.second);
                 nAddedObs++;
             }
         }
     }
     _debug_msg_(" searchInNeighbors Added="<<nAddedObs<<" FUSED="<<nFused );

     return  points2Remove;



 }

    std::list<MapPoint> MapManager::createCloseStereoPoints(Frame & NewFrame,int nn){
     if (!NewFrame.imageParams.isStereoCamera()) return {};


     //struct to store depth-idx in the frame
     struct kpt_depth_data{
         float depth;
         size_t idx;
         bool operator <(const kpt_depth_data&hp)const{return depth<hp.depth;}
         bool operator >(const kpt_depth_data&hp)const{return depth>hp.depth;}
     };


     NewFrame.nonMaximaSuppresion();
     //count number of close points
     int nclose=0;
     for(size_t i=0;i<NewFrame.ids.size();i++)
         if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max() &&!NewFrame.nonMaxima[i] && NewFrame.depth[i]>0 &&     NewFrame.imageParams.isClosePoint(NewFrame.depth[i]) )
                 nclose++;
     std::vector<kpt_depth_data> usedPoints;
     usedPoints.reserve(std::max(nclose,100));

     //now,if less than 100, use the nearest ones
     if(nclose<100){
         MinBag<kpt_depth_data> bag;
         for(size_t i=0;i<NewFrame.ids.size();i++)
             if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max() && NewFrame.depth[i]>0  &&!NewFrame.nonMaxima[i])
                 bag.push({NewFrame.depth[i],i});
         //now, add the points to the usedPoints vector
         while(!bag.empty() )
             usedPoints.push_back(bag.pop());
     }
     else{
         for(size_t i=0;i<NewFrame.ids.size();i++)
             if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max() &&!NewFrame.nonMaxima[i] && NewFrame.depth[i]>0 &&     NewFrame.imageParams.isClosePoint(NewFrame.depth[i]) )
                 usedPoints.push_back({ NewFrame.depth[i],i});
     }

     std::list<MapPoint> _NewMapPoints_;
     //get the unassigned points that are close
     auto pose_g2f=NewFrame.pose_f2g.inv();
     for(auto &kpd:usedPoints){
         MapPoint mapPoint;
         mapPoint.id=_NewMapPoints_.size();
         mapPoint.creationSeqIdx=NewFrame.fseq_idx;
          cv::Point3f p3d=NewFrame.get3dStereoPoint(kpd.idx);
         //move the point to global coordinates
         p3d=pose_g2f*p3d;
         mapPoint.setCoordinates( p3d );
         mapPoint.addKeyFrameObservation(NewFrame,kpd.idx);
         mapPoint.isStereo=true;
         _NewMapPoints_.push_back(mapPoint);;
     }
     //now, find the new points in the other frames


     return _NewMapPoints_;
 }

    std::list<MapPoint>  MapManager::createNewPoints(Frame &NewFrame ,uint32_t nn){

     struct FrameInfoUnass{
           cv::Mat desc;
           std::vector<uint32_t> positions;
           std::vector<int> octaves;
           std::vector<cv::Point2f> pt;
           int frame_idx;
           uint32_t size()const{return pt.size();}
          };  //given the frame passed, set in desc the unassigned descriptors from frame.desc and in positions the position in the elements added
     auto getUnassignedDescriptors=[](Frame & frame){
         FrameInfoUnass fiu;
         cv::Mat  unassign_Desc(frame.desc.size(),frame.desc.type());//oversize
         fiu.positions.resize(frame.desc.rows);
         fiu.octaves.resize(frame.desc.rows);
         fiu.pt.resize(frame.desc.rows);

         //copy the unassigned keypoint descriptors to the above matrix.
         int idx=0;
         for(size_t i=0;i< frame.ids.size();i++){
             if (frame.ids[i]==std::numeric_limits<uint32_t>::max() && !frame.nonMaxima[i]){ //unassigned
                 frame.desc.row(i).copyTo( unassign_Desc.row(idx));
                 fiu.positions[idx]=i;
                 fiu.octaves[idx]=frame.und_kpts[i].octave;
                 fiu.pt[idx]=frame.und_kpts[i].pt;
                 idx++;
             }
         }

         fiu.desc=unassign_Desc(cv::Range(0,idx),cv::Range::all());//copy only the used part of the oversized matrix
         fiu.positions.resize(idx);//reduce size to fit the used one
         fiu.octaves.resize(idx);
         fiu.pt.resize(idx);
         fiu.frame_idx=frame.idx;
         return fiu;
     };




     auto  match_desc_xflann_Epi=[](xflann::Index &index,const FrameInfoUnass & frame1,const FrameInfoUnass & frame2,const cv::Mat &F12, const vector<float> &scaleFactors, float distThres,double nn_match_ratio ){
         cv::Mat indices,distances;
         index.search(frame2.desc,10,indices,distances ,xflann::KnnSearchParams(32,false));

         vector<cv::DMatch> matches;
         if ( distances.type()==CV_32S)
             distances.convertTo(distances,CV_32F);
         vector<float> mvLevelSigma2(scaleFactors);
         for(auto &v:mvLevelSigma2) v=v*v;
         for(int i = 0; i < indices.rows; i++) {
             float bestDist=distThres,bestDist2=std::numeric_limits<float>::max();
             int64_t bestQuery=-1,bestTrain=-1;
             for(int j=0;j<indices.cols;j++){
                 if ( distances.at<float>(i,j)<bestDist2){
                     if (epipolarLineSqDist( frame1.pt [indices.at<int>(i,j)] ,frame2.pt[i],F12 ) <3.84*mvLevelSigma2[frame2.octaves[ i]]){
                         if (distances.at<float>(i,j)<bestDist){
                             bestDist=distances.at<float>(i,j);
                             bestQuery=i;
                             bestTrain=indices.at<int>(i,j);
                         }
                         else{
                             bestDist2=distances.at<float>(i,j);
                         }
                     }
                 }
             }

             if ( bestQuery!=-1)
                 if( bestDist<bestDist2*nn_match_ratio)
                 {
                     cv::DMatch  match;
                     match.queryIdx=bestQuery;
                     match.trainIdx=bestTrain;
                     match.distance=bestDist;
                     matches.push_back(match);
                 }

         }
         filter_ambiguous_train(matches);

         return matches;

     };



     //the set of created matches
     //for each frame, the matches (NewFrame:kptIdx,Frame2:kptIdx)
     struct fmatch{
         uint32_t newframe_kp_idx;
         uint32_t frame2_id;
         uint32_t frame2_kp_idx;
         cv::Point3f p3d;
         float distance;
     };

     //-------------------------------------------------------------------
     // START
     //-------------------------------------------------------------------
     ScopedTimerEvents timer("MapManager::createNewPoints");

     std::list<MapPoint> _NewMapPoints_;
      vector<float> scaleFactors=NewFrame.scaleFactors;
     vector<float> mvLevelSigma2;
     for(auto sf:scaleFactors) mvLevelSigma2.push_back(sf*sf);
     std::map<uint32_t,set<uint32_t> > frame_kptidx;//store the associtaions frame and its kpt indices to avoid repetitions
     xflann::Index NewFrameIndex;
     Se3Transform TN_G2F=NewFrame.pose_f2g.inv();//matrix moving points from the new frame to the global ref system
     timer.add("initialize");



     //Create a sorted list of the frames far enough. They are sorted with the number of matched points as sorting order
     vector<uint32_t> matchingFrames =getMatchingFrames(NewFrame,nn);
     timer.add("compute list of good matching frames");




     //compute the unassigned points in the new frame
     auto NewFrameUnassigedInfo=getUnassignedDescriptors(NewFrame);
     if(NewFrameUnassigedInfo.size()==0)return {};
     //and build an index for fast search of descriptors
     NewFrameIndex.build(NewFrameUnassigedInfo.desc,xflann::HKMeansParams(8,0));//random centers with no optimization
     timer.add("Index of NewFrame Created");


     //for each possible matching frame, find matches between keypoints
    vector< vector<fmatch> > FrameMatches(matchingFrames.size());//the result is saved here.
   // omp_set_num_threads(2);
 //#pragma omp parallel for
     for(int mf=0;mf<int(matchingFrames.size());mf++){
         Frame &frame2=TheMap->keyframes[matchingFrames[mf]];
         //get the unassigned descriptors of the frame
         auto Frame2Info=getUnassignedDescriptors(frame2);
         //do matching with the newframe
         cv::Mat F12=computeF12(NewFrame.pose_f2g,NewFrame.imageParams.CameraMatrix,frame2.pose_f2g,NewFrame.imageParams.CameraMatrix);


 //        auto matches=match_desc_xflann(NewFrameIndex,NewFrameUnassigedInfo,Frame2Info,Slam::getParams().minDescDistance, 0.6);
         auto matches=match_desc_xflann_Epi(NewFrameIndex,NewFrameUnassigedInfo,Frame2Info, F12, frame2.scaleFactors, Slam::getParams().minDescDistance, 0.6);
         //transform matches to the original elements in the frame
         for(auto &m:matches){
             m.trainIdx=NewFrameUnassigedInfo.positions[m.trainIdx];
             m.queryIdx=Frame2Info.positions[m.queryIdx];
         }

         ///Do triangulation and filter by
         vector<cv::KeyPoint> pa(matches.size()),pb(matches.size());
         for(size_t i=0;i<matches.size();i++){
             pa[i]= NewFrame. und_kpts[ matches[i].trainIdx];
             pb[i]=  frame2. und_kpts[ matches[i].queryIdx];
         }
         vector<bool> vGood;
         vector<cv::Point3f> p3d;
         triangulate_( frame2.pose_f2g* NewFrame.pose_f2g.inv(),pa,pb,NewFrame.imageParams.CameraMatrix,p3d,NewFrame.scaleFactors, vGood);

         ///save the good matches and move the point the frame ref system
         for(size_t i=0;i<matches.size();i++)
             if (vGood[i]) FrameMatches[mf].push_back( {uint32_t(matches[i].trainIdx),frame2.idx, uint32_t(matches[i].queryIdx),TN_G2F*p3d[i],matches[i].distance});

     }
     timer.add("Matches  ("+std::to_string(matchingFrames.size())+")");


  //merge all info grouped by keypoint index
     std::map<uint32_t,vector<fmatch> > KptIndex_MatchInfo;
     for(size_t mf=0;mf<FrameMatches.size();mf++)
         for(const auto &match: FrameMatches[mf])
             KptIndex_MatchInfo[ match.newframe_kp_idx].push_back( match);

     //for points with multiple matches, we must ensure that information is ok
     for(auto &kp:KptIndex_MatchInfo )
     {

         MapPoint mapPoint;
         mapPoint.id=_NewMapPoints_.size();
         mapPoint.creationSeqIdx=NewFrame.fseq_idx;

         //select the point with lowest descriptor distance
         int bestDesc=0;
         for(size_t di=1;di<kp.second.size();di++)
             if ( kp.second[bestDesc].distance<kp.second[di].distance) bestDesc=di;
         const auto &best_match=kp.second[bestDesc];
         mapPoint.setCoordinates(best_match.p3d);
         mapPoint.addKeyFrameObservation(TheMap->keyframes[best_match.frame2_id], best_match.frame2_kp_idx);
         frame_kptidx[best_match.frame2_id].insert(best_match.frame2_kp_idx);//register the connection to avoid using this keypoint in the next section
         mapPoint.addKeyFrameObservation(NewFrame,kp.first);
         _NewMapPoints_.push_back(mapPoint);;
     }

     timer.add("Merging and creation");

     _debug_msg_("Added "<<_NewMapPoints_.size()<<" new points");


 //   //undo the nonmaxima
 //    for(auto &invalid:NewFrame.invalid) invalid=0;

     return _NewMapPoints_;


 //    //try to find the points in other frames

 //     //precompute the search radius in each octave

 //    float search_radius=sqrt(5.999);
 //    float logScaleFactor=log(scaleFactors[1]);

 //    std::map<uint32_t,std::mutex> mp_mutex;
 //    for(const auto &mp:_NewMapPoints_)
 //        mp_mutex[mp.id];

 //    std::map<uint32_t,std::mutex> frame_mutex;
 //    //ensure the frame_kptidx is created for all elements
 //    for(size_t mf=0;mf<matchingFrames.size();mf++) {
 //        frame_kptidx[matchingFrames[mf]];
 //        frame_mutex[matchingFrames[mf]];
 //    }

 //#pragma omp parallel for
 //    for(int mf=0;mf<int(matchingFrames.size());mf++){
 //        uint32_t frameIdx=matchingFrames[mf];
 //        Frame &TheFrame=TheMap->keyframes[ frameIdx];
 //        float fx=TheFrame.imageParams.fx();
 //        float fy=TheFrame.imageParams.fy();
 //        float cx=TheFrame.imageParams.cx();
 //        float cy=TheFrame.imageParams.cy();
 //         //TheFrame.create_kdtree();//TODO (it should no need to be recreated)!
 //        int npointsAdded=0;

 //        for(MapPoint &mapPoint:_NewMapPoints_){

 //            if ( mapPoint.isObservingFrame( frameIdx))  continue;//already connected to this frame

 //            //------------- PROJECT
 //            //check that point is in front of the frame (positive depth)
 //            cv::Point3f map_p3d= TheFrame.pose_f2g*mapPoint.getCoordinates();//transform to camera ref system
 //            if( map_p3d.z<0) continue;
 //            float distCamPoint=cv::norm(map_p3d);
 //            if (   !(mapPoint.getMinDistanceInvariance() < distCamPoint && distCamPoint< mapPoint.getMaxDistanceInvariance()))continue;
 //            //project
 //            map_p3d.z=1./map_p3d.z;
 //            cv::Point2f map_p2d(  map_p3d.x*fx*map_p3d.z+cx,map_p3d.y*fy*map_p3d.z+cy);
 //            //check that it is into limits
 //            if (map_p2d.x<0 || map_p2d.y<0 || map_p2d.x>=TheFrame.imageParams.CamSize.width || map_p2d.y>=TheFrame.imageParams.CamSize.height) continue;



 //            //ok, then, look for the keypoints and take the one that is most similar
 //            //predict the octave it should project
 //            int predictedOctave= mapPoint.predictScale( distCamPoint,logScaleFactor,scaleFactors.size() );
 //            //scale the search radius to acoount for the possibility of finding the point in a coarser scale
 //            float sc= scaleFactors[ std::min( size_t(predictedOctave+1) ,scaleFactors.size()-1 )];

 //            //radius search
 //            std::vector<std::pair<size_t,float>>  vIndices_Dists=TheFrame.getKeyPointsInRegion( map_p2d, search_radius*sc,predictedOctave,predictedOctave+1);

 //            std::pair<size_t,float> best(std::numeric_limits<size_t>::max(),std::numeric_limits<float>::max());
 //            std::pair<size_t,float> best2=best;

 //            for(auto idx_dist : vIndices_Dists){
 //                if ( TheFrame.ids[ idx_dist.first]!=std::numeric_limits<uint32_t>::max() ) continue;//already assigned in previuos time
 //           //     if ( abs(TheFrame.und_kpts[idx_dist.first].octave- predictedOctave)>2) continue;
 //                frame_mutex[TheFrame.idx].lock();
 //                 bool isUsed=frame_kptidx.at(TheFrame.idx).count( idx_dist.first);
 //                 frame_mutex[TheFrame.idx].unlock();
 //                 if (isUsed) continue;//already assigned in this function
 //                assert(TheMap->keyframes[TheFrame.idx].ids[idx_dist.first]==std::numeric_limits<uint32_t>::max());
 //                //get descriptor distance
 //                 float descDist=mapPoint.getDescDistance( TheFrame.desc.row(idx_dist.first));
 //                 if ( descDist <  Slam::getParams().minDescDistance){
 //                    if ( idx_dist.second<best.second)
 //                        best= idx_dist;
 //                    else if( idx_dist.second<best2.second)
 //                        best2= idx_dist;
 //                }
 //            }
 //            //now, take the decission
 //            if ( best.first!=std::numeric_limits<size_t>::max() ){
 //                //the two best possibilities are in the same octave and very similar
 //                if (  best.second>best2.second*0.8) continue;
 //                //mark to avoid repetitions

 //                  mapPoint.addKeyFrameObservation(TheFrame, best.first);
 //                  frame_mutex[TheFrame.idx].lock();
 //                 frame_kptidx.at(TheFrame.idx).insert(best.first);
 //                 frame_mutex[TheFrame.idx].unlock();
 //                npointsAdded++;


 //            }
 //        }
 //        //update to the covis graph
 //        //         covisupdate_nodes_weights[CovisGraph::join(frameIdx,NewFrameIdx)].v+=npointsAdded;

 //        _debug_msg_("added "<<npointsAdded<<" to view "<<frameIdx);
 //    }

 //    //for each point, project
 //    assert(TheMap->checkConsistency());
 //    return _NewMapPoints_;
 }

    void MapManager:: globalOptimization(int niters ){
      GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=5);
     //must set the first and second as fixed?
     params.fixFirstFrame=true;
     params.nIters=niters;


     if (params.fixed_frames.size()==0 && TheMap->map_markers.size()==0){//fixed second one to keep the scale if no markers
         //get the second frame if there is one and set it fixed
         auto it=TheMap->keyframes.begin();
         params.fixed_frames.insert(it->idx);
         ++it;
         if (it!=TheMap->keyframes.end()) {
             if(params.used_frames.count(it->idx) || params.used_frames.size()==0 )
                 params.fixed_frames.insert(it->idx);
         }
     }

     Gopt=GlobalOptimizer::create(Slam::getParams().global_optimizer);
     Gopt->setParams(TheMap,params);
     Gopt->optimize();
     Gopt->getResults(TheMap);
     TheMap->removeBadAssociations(Gopt->getBadAssociations(),Slam::getParams().minNumProjPoints);
     Gopt=nullptr;
 }

    void MapManager:: localOptimization(uint32_t _newKFId){
       //add current frame and all the connected to them for optimization
     //then, also add as fixed all these in which elements in former frames project

     //is there any stereo
     bool hasStereo=false;
     for(auto f:TheMap->keyframes)
         if (f.imageParams.isStereoCamera()){
             hasStereo=true;
             break;
         }

     std::set<uint32_t> neigh=     TheMap->TheKpGraph.getNeighbors(_newKFId,true);

      GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=5);
      params.used_frames.insert(neigh.begin(),neigh.end());
     //must set the first and second as fixed?
     params.fixFirstFrame=true;
     params.nIters=10;


     if (params.fixed_frames.size()==0 && TheMap->map_markers.size()==0 && !hasStereo)
     {//fixed second one to keep the scale if no markers
         //get the second frame if there is one and set it fixed
         auto it=TheMap->keyframes.begin();
         params.fixed_frames.insert(it->idx);
         ++it;
         if (it!=TheMap->keyframes.end()) {
             if(params.used_frames.count(it->idx))
                 params.fixed_frames.insert(it->idx);
         }
     }

     Gopt=GlobalOptimizer::create(Slam::getParams().global_optimizer);
     Gopt->setParams(TheMap,params);
     Gopt->optimize();
 }

    void MapManager::toStream(std::ostream &str) {
     //must wait until no processing is being done

     while(_curState==WORKING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
     //now, in case in state previous to store results, save them and then continue
     mapUpdate();

     uint64_t sig=1823312417;
     str.write((char*)&sig,sizeof(sig));

     str.write((char*)&mustExit,sizeof(mustExit));
     auto val=_curState.load();
     str.write((char*)&val,sizeof(val));
     //toStream__kv(unStablePoints,str);
      toStream__(keyframesToAdd.buffer_,str);
     str.write((char*)&_counter,sizeof(_counter));
     toStream__(PointsToRemove,str);
     toStream__(KeyFramesToRemove,str);

     toStream__kv(youngKeyFrames,str);

     _TheLoopDetector.toStream(str);

     str.write((char*)&_hasMapBeenScaled,sizeof(_hasMapBeenScaled));



 }
    void MapManager::fromStream(std::istream &str){

     stop();

     uint64_t sig;
     str.read((char*)&sig,sizeof(sig));
     if(sig!=1823312417) throw std::runtime_error("Could not read signature of Mapmanager in stream");

     str.read((char*)&mustExit,sizeof(mustExit));
     auto cstatevar=_curState.load();
     str.read((char*)&cstatevar,sizeof(cstatevar));
     //   fromStream__kv(unStablePoints,str);
     fromStream__(keyframesToAdd.buffer_,str);
     str.read((char*)&_counter,sizeof(_counter));
     fromStream__(PointsToRemove,str);
     fromStream__(KeyFramesToRemove,str);

     fromStream__kv(youngKeyFrames,str);

     _curState=cstatevar;
     _TheLoopDetector.fromStream(str);
     str.read((char*)&_hasMapBeenScaled,sizeof(_hasMapBeenScaled));

 }

    //adds the keyframe and correct and join points from both sides
    void MapManager::loopClosurePostProcessing(Frame &frame, const LoopDetector::LoopClosureInfo &lci){
     auto neighborsSideOld=TheMap->TheKpGraph.getNeighbors(lci.matchingFrameIdx,true);

     //remove possible connection with other side of the loop

     for(auto &id:frame.ids)
         if (id!=std::numeric_limits<uint32_t>::max()){
             for(auto n:neighborsSideOld)
                 if (TheMap->map_points[id].isObservingFrame(n))
                     id=std::numeric_limits<uint32_t>::max() ;
         }


     auto &NewFrame=frame;
     if (!TheMap->keyframes.is(frame.idx))//add the keyframe if not yet (in case of marker loopclosure)
            NewFrame= addKeyFrame(&frame);



     //add to the frame, the points seen in the other side of the loop
     for(auto match:lci.map_matches)
         if (NewFrame.ids[match.queryIdx]==std::numeric_limits<uint32_t>::max())
             TheMap->addMapPointObservation(match.trainIdx,NewFrame.idx,match.queryIdx);

     globalOptimization(10);


     //estimate the position again and find matches

     for(auto &mp:TheMap->map_points)
         mp.getExtra<uint32_t>()[0]=std::numeric_limits<uint32_t>::max();
     //set to avoid visiting the points already marked in the NewFrame projected
     for(auto id:NewFrame.ids)
         if (id!=std::numeric_limits<uint32_t>::max())
             TheMap->map_points[id].getExtra<int>()[0]=NewFrame.fseq_idx;

     auto neighborsSideNew=TheMap->TheKpGraph.getNeighbors( lci.curRefFrame,true);
     auto allNeighbors=neighborsSideNew;
     for(auto n:neighborsSideOld)allNeighbors.insert(n);
     allNeighbors.insert(NewFrame.idx);
     vector<uint32_t> allNeighborsV;
     for(auto n:allNeighbors)allNeighborsV.push_back(n);

     auto map_matches2 =TheMap->matchFrameToMapPoints(allNeighborsV, NewFrame,  NewFrame.pose_f2g,Slam::getParams().minDescDistance*1.5, 1.5,false,false);
     //add this new projections
     //add connections to other frames



     for(auto match:map_matches2){
         if (NewFrame.ids[match.queryIdx]==std::numeric_limits<uint32_t>::max())
             TheMap->addMapPointObservation(match.trainIdx,NewFrame.idx,match.queryIdx);
     }
     auto toRemove=searchInNeighbors(NewFrame);
     TheMap->removePoints(toRemove.begin(),toRemove.end());
     //            fusepoints(curRefKf,goodLoopCandidates[0].frame);
   //  assert(TheMap->checkConsistency());
     globalOptimization(10);
     frame.pose_f2g=NewFrame.pose_f2g;
     frame.ids=NewFrame.ids;


 }

    void MapManager::LoopCandidate::fromStream(istream &str){
     fromStream__(nodes,str);
     str.read((char*)&nobs,sizeof(nobs));
     str.read((char*)&timesUnobserved,sizeof(timesUnobserved));

 }
    void MapManager::LoopCandidate::toStream(ostream &str)const
    {
        toStream__(nodes,str);
        str.write((char*)&nobs,sizeof(nobs));
        str.write((char*)&timesUnobserved,sizeof(timesUnobserved));
    }

    double MapManager::computeTheta(Frame f1, Frame f2)
    {
        double avg = 0.0;
        int n = 0;
        for (size_t i = 0; i < f1.markers.size(); ++i)
        {
            uint32_t id1 = f1.markers[i].id;
            for (int j = 0; j < f2.markers.size(); ++j)
            {
                uint32_t id2 = f2.markers[j].id;
                if (id1 != id2) continue;
                double theta1 = f1.markers_solutions[i].theta;
                double theta2 = f2.markers_solutions[j].theta;
                double delta = abs(theta2 - theta1);
                avg = avg + delta;
                n++;
            }
        }
        if (n == 0) return 0.0;
        double deltaTheta = avg / float(n);
        return deltaTheta;
    }
 }

