#include "loopdetector.h"
#include "optimization/loopclosure.h"
#include "utils.h"
#include "slam.h"
#include "stuff/timers.h"
#include "optimization/solvepnp.h"
#include "stuff/io_utils.h"
namespace  ucoslam {

void LoopDetector::setParams(std::shared_ptr<Map> map){
    TheMap=map;
}

//****
//* param: frame: 当前帧
//
LoopDetector::LoopClosureInfo  LoopDetector::detectLoopFromMarkers(Frame &frame, int32_t curRefKf){
    assert(TheMap);
    ScopedTimerEvents Timer("LoopDetector::detectAndCorrectLoopUsingMarkers");


    vector<LoopClosureInfo> loopClosurePossibilities;
    ///Step1, 检测回环，并计算位姿
    if (TheMap->map_markers.size()>0 && frame.markers.size()!=0){
        loopClosurePossibilities= detectLoopClosure_Markers(frame,curRefKf);
        Timer.add("detectLoopClosure_Markers");
    }

    if (loopClosurePossibilities.size()==0) return  LoopDetector::LoopClosureInfo();

    ///Step2，优化本质图位姿
    //并将计算得到的信息写入loopClosurePossibilities
    for(auto &lc:loopClosurePossibilities) solve(frame,lc);

    int best_solution=0;
    if (loopClosurePossibilities.size()>1){
        double err1=testCorrection(loopClosurePossibilities[0]);
        double err2=testCorrection(loopClosurePossibilities[1]);
        if (err2<err1) best_solution=1;

    }
    Timer.add("test solutions");
    return loopClosurePossibilities[best_solution];
}


LoopDetector::LoopClosureInfo LoopDetector::detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf ){
    ScopedTimerEvents Timer("LoopDetector::detectLoopFromKeyPoints");
    vector<LoopClosureInfo> loopClosurePossibilities;
    if ( frame.ids.size()!=0){
        loopClosurePossibilities=detectLoopClosure_KeyPoints(frame,curRefKf);
        Timer.add("detectLoopClosure_KeyPoints");
    }
    if (loopClosurePossibilities.size()==0) return LoopDetector::LoopClosureInfo() ;
    else{
         solve(frame,loopClosurePossibilities[0]);
         return loopClosurePossibilities[0];
    }
}




void LoopDetector::toStream(ostream &str)const{
    toStream__kv_complex(loopCandidates,str);

}
void LoopDetector::fromStream(istream &str){
    fromStream__kv_complexmap<uint32_t,LoopDetector::KpLoopCandidate>(loopCandidates,str);

}


vector<LoopDetector::LoopClosureInfo>  LoopDetector::detectLoopClosure_Markers(Frame & frame,int64_t curkeyframe)
{
    //we detect a loop closure if there a valid marker in the view, with good localization, that is not connected in the graph to curkeyframe
    //so first, see if the case holds
    ///Step1, 获取局部关键帧的有效Markers
    LoopClosureInfo lci;
    std::set<uint32_t> l1neigh=TheMap->getNeighborKeyFrames(curkeyframe,true);
     //find all markers in the neighbors and in this
    std::set<uint32_t> visibleMarkers;
    for(auto n:l1neigh)
        for(auto m:TheMap->keyframes[n].markers)
            visibleMarkers.insert(m.id);

    ///Step2, 寻找闭环Markers
    vector<uint32_t> markersCausingLoopClosure;
    for(size_t mi=0;mi< frame.markers.size();mi++){
        auto &marker=frame.markers[mi];
        if (  visibleMarkers.count(marker.id)!=0)continue;//has already been seen
        auto mit=TheMap->map_markers.find(marker.id);//is in the system???
        if(mit!=TheMap->map_markers.end())//yes
            if (mit->second.pose_g2m.isValid()){//is valid??
                //                        if ( frame.markers_solutions[mi].err_ratio>_params.aruco_minerrratio_valid) //is it reliable?
                markersCausingLoopClosure.push_back(marker.id);

            }
    }


    if ( markersCausingLoopClosure.size()==0) return  {};


    //now, well try to solve the problem we have.
    //current frame has two possible locations,
    //first, the one estimated with drift (which is in frame)
    //and the one calculated using the markers seen already (expected)
     //determine this original location using the frames that caused the loop closure
    vector<se3> expectedposes;

    // Step3,利用闭环Marker，计算当前帧位姿
    // Step3.1，利用方法1计算出一个最佳位姿
    se3 safe_expectedpose=TheMap->getBestPoseFromValidMarkers(frame,markersCausingLoopClosure,4,35);
     //is the pose  good enough?
    if (   safe_expectedpose.isValid())
        expectedposes.push_back(safe_expectedpose);
    //if not, it is safer to fo optimization using the two possible ways
    // Step3.2，方法1失败，则利用方法2计算出两个位姿
    else{
        int bestM=0;
        for(size_t i=1;i< markersCausingLoopClosure.size();i++)
            if ( frame.getMarkerPoseIPPE(markersCausingLoopClosure[i]).err_ratio>frame.getMarkerPoseIPPE(markersCausingLoopClosure[bestM]).err_ratio )
                bestM=i;
        //compute current pose based on the two solutions
        //F2G
        int bestMId=markersCausingLoopClosure[bestM];
        cv::Mat  invM=TheMap->map_markers[bestMId].pose_g2m.convert().inv() ;
        se3 pose1=       se3(frame.getMarkerPoseIPPE(bestMId).sols[0]* invM);
        se3 pose2=       se3(frame.getMarkerPoseIPPE(bestMId).sols[1]* invM);
        expectedposes.push_back(pose1);
        expectedposes.push_back(pose2);

    }
 //    if(!expectedpose.isValid());



     //so, the frame is at frame.pose_f2g but it should be at orgpose also
     //loop closure consists in making these two estimations identical by distributing the errors equally amongst the nodes in the
     //path connecting the orignal location and current one
     //But, which is the 'original location'? There are many possibilities in our problem, since the marker has been seen from
     //many other frames probably.
     //So, let us take as origin node the first one in  the video in which it was seen
     //Step4,寻找闭环帧
     uint32_t minFrameSeqIdx=std::numeric_limits<uint32_t>::max(),minFrameIdx=std::numeric_limits<uint32_t>::max();
     for(auto m:markersCausingLoopClosure){
         assert(TheMap->map_markers.count(m)!=0);//the marker must be and be valid
         assert(TheMap->map_markers.at(m).pose_g2m.isValid());
         for(auto frame_idx:TheMap->map_markers[m].frames){
             if ( minFrameSeqIdx>TheMap->keyframes[frame_idx].fseq_idx){
                 minFrameSeqIdx=TheMap->keyframes[frame_idx].fseq_idx;
                 minFrameIdx=frame_idx;
             }
         }
     }

    //Step5，保存闭环信息
    vector<LoopClosureInfo> vlci;
    for(size_t i=0;i<expectedposes.size();i++)
    {
        LoopClosureInfo lci;
        lci.curRefFrame=curkeyframe;
        lci.expectedPos=expectedposes[i];
        lci.matchingFrameIdx=minFrameIdx;
        lci.loopMarkers = markersCausingLoopClosure;
        vlci.push_back(lci);

    }

    return vlci;

}

///优化本质图
void LoopDetector::solve(Frame &frame,   LoopClosureInfo &loopc_info)
{
    //Step1,获取本质图
    //krukal算法
    CovisGraph EssentialGraph;
    TheMap->TheKpGraph.getEG(EssentialGraph);


   //Build an easy to use set of connections
   vector<pair<uint32_t,uint32_t> > edges;
   //nw: EssentialGraph的权重关系,即_EssentialGraph._mweights，
   for(auto nw:EssentialGraph.getEdgeWeightMap()){
       auto i_j=EssentialGraph.separe(nw.first);
       edges.push_back({i_j.first,i_j.second});
   }
    //finally, add the connection between this and the first one in which one of the markers appeared


   assert(EssentialGraph.getNodes().count(loopc_info.curRefFrame));//检测当前帧的参考帧是否在本质图中
   assert(EssentialGraph.getNodes().count(loopc_info.matchingFrameIdx));//检测闭环帧是否在本质图中

   edges.push_back({loopc_info.curRefFrame,frame.idx});
    auto &optimPoses=loopc_info.optimPoses;
//   vector<std::map<uint32_t, cv::Mat> >optimPoses(1);
   for(auto e:edges){
       if ( optimPoses.count(e.first)==0 && TheMap->keyframes.count (e.first)!=0)
           optimPoses[e.first]=TheMap->keyframes[e.first].pose_f2g;
       if ( optimPoses.count(e.second)==0 && TheMap->keyframes.count (e.second)!=0)
           optimPoses[e.second]=TheMap->keyframes[e.second].pose_f2g;
   }
   edges.push_back({frame.idx,loopc_info.matchingFrameIdx});
   optimPoses[frame.idx]=frame.pose_f2g;

    ///optimPoses输入：本质图原始位姿
    ///Step2，回环优化本质图的位姿
    loopclosure::loopClosurePathOptimization_cv(edges,frame.idx,
            loopc_info.matchingFrameIdx,loopc_info.expectedPos, optimPoses);
    //optimPose输出： 本质图优化过的位姿

    ///Step3, 计算优化前和优化后的位姿误差
    ///now, compute the displacement that each marker and point must undergo
    //compute the amount of change for each marker
    std::map<uint32_t,cv::Mat> frame_posechange;
    for(auto op:optimPoses)
    {
        if (op.first==frame.idx)
            frame_posechange[op.first]=frame.pose_f2g*op.second.inv();
        else
            frame_posechange[op.first]=TheMap->keyframes[op.first].pose_f2g*op.second.inv();
    }

    ///Step4，优化Markers和Points
    // *a, 计算每个Marker和Point的总误差
    // *b, 平均总误差
    //find the displacement that must be applied to each point and marker
    //using the average dissplacement according to the frames in which they are seen
    //this process is repeated for each possible solution
    std::map<uint32_t,pair<se3,zeroinitvar<int> > >  marker_inc;
    std::map<uint32_t,pair<se3,zeroinitvar<int> > >  point_inc;
    for(auto fidx_pose:optimPoses)
    {
        if ( TheMap->keyframes.count(fidx_pose.first))
        {
            const Frame &keyFrame=TheMap->keyframes[fidx_pose.first];
            for(auto m:TheMap->keyframes[fidx_pose.first].markers){
                //compute marker to frame
                ///统计优化后对每个Marker的总影响，即总误差
                ///m2f: f->m, pose_f2g: g->f, pose_g2m: m->g
                auto m2f=  TheMap->map_markers[m.id].pose_g2m.convert().inv()*keyFrame.pose_f2g.inv();
                auto m2m=se3(m2f* fidx_pose.second * TheMap->map_markers[m.id].pose_g2m);
                auto &minc=marker_inc[m.id];
                if (minc.second==0) minc.first=m2m;
                else minc.first+=m2m;
                minc.second++;
            }

            ///统计优化后对每个Point的总影响，即总误差
            se3 thisframe_posechange=frame_posechange[keyFrame.idx];
            for(size_t i=0;i<keyFrame.ids.size();i++){
                if (keyFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                    auto &pinc=point_inc[keyFrame.ids[i]];
                    if (pinc.second==0) pinc.first=thisframe_posechange;
                    else pinc.first+=thisframe_posechange;
                    pinc.second++;
                }
            }
        }
    }
    //average the results
    for(auto &m_inc:marker_inc){
        m_inc.second.first/=float(m_inc.second.second);
        loopc_info.marker_change[m_inc.first]=m_inc.second.first;//mean of increments
    }
    for(auto &p_inc:point_inc){
        p_inc.second.first/=float(p_inc.second.second);
        loopc_info.point_change[p_inc.first]=p_inc.second.first;//mean of increments
    }
 }

void LoopDetector::correctMap(  const LoopClosureInfo &lcsol){
    //move markers
    for(const auto m_inc:lcsol.marker_change)
        TheMap->map_markers[m_inc.first].pose_g2m=TheMap->map_markers[m_inc.first].pose_g2m.convert()* m_inc.second.convert().inv();
    //move points
    for(const auto p_inc:lcsol.point_change){
        auto &mapPoint=TheMap->map_points[p_inc.first];
        mapPoint.setCoordinates(  p_inc.second.convert()*mapPoint.getCoordinates());
    }
    //move keyframes
    for(const auto op:lcsol.optimPoses){
        if ( TheMap->keyframes.count(op.first)){
            TheMap->keyframes[op.first].pose_f2g=op.second;
        }
    }
}


double LoopDetector::testCorrection(const LoopClosureInfo &lcsol){

    std::map<uint32_t,se3> marker_poses;
    std::map<uint32_t,cv::Point3f> point_coordinates;
    std::map<uint32_t,Se3Transform> frame_poses;

    for(auto m:TheMap->map_markers)
        marker_poses[m.first]=m.second.pose_g2m;
    for(auto p:TheMap->map_points)
        point_coordinates.insert({p.id,p.getCoordinates()});
    for(auto f:TheMap->keyframes)
        frame_poses[f.idx]=f.pose_f2g;

    correctMap(lcsol);


    //compute the error
    vector<uint32_t> keyframes;
    for(auto kf:lcsol.optimPoses )
        if (TheMap->keyframes.is(kf.first))
            keyframes.push_back(kf.first);
    auto err=TheMap->globalReprojChi2(keyframes,0,0,true,true);
    _debug_msg("err sol ="<<err ,10);

    //restore
    for(auto &m:TheMap->map_markers)
        m.second.pose_g2m = marker_poses[m.first];
    for(auto &k:TheMap->keyframes)
        k.pose_f2g= frame_poses[k.idx];
    for(auto &mp:TheMap->map_points)
        mp.setCoordinates(point_coordinates[mp.id]);

    return err;
}

std::vector<LoopDetector::LoopClosureInfo> LoopDetector::detectLoopClosure_KeyPoints( Frame &frame, int32_t curRefKf){

return {};

}

//void LoopDetector::correctLoopClosure_Markers(const LoopClosureInfo &lci,  Frame &frame, uint64_t _curKFRef){
//   //now, well try to solve the problem we have.
//   //current frame has two possible locations,
//   //first, the one estimated with drift (which is in frame)
//   //and the one calculated using the markers seen already (expected)
//    //determine this original location using the frames that caused the loop closure
//   vector<se3> expectedposes;

//   se3 safe_expectedpose=TheMap->getBestPoseFromValidMarkers(frame,lci.markers,4);
//    //is the pose  good enough?
//   if (   safe_expectedpose.isValid())
//       expectedposes.push_back(safe_expectedpose);
//   //if not, it is safer to fo optimization using the two possible ways
//   else{
//       int bestM=0;
//       for(size_t i=1;i< lci.markers.size();i++)
//           if ( frame.getMarkerPoseIPPE(lci.markers[i]).err_ratio>frame.getMarkerPoseIPPE(lci.markers[bestM]).err_ratio )
//               bestM=i;
//       //compute current pose based on the two solutions
//       //F2G
//       int bestMId=lci.markers[bestM];
//       cv::Mat  invM=TheMap->map_markers[bestMId].pose_g2m.convert().inv() ;
//       se3 pose1=       se3(frame.getMarkerPoseIPPE(bestMId).sols[0]* invM);
//       se3 pose2=       se3(frame.getMarkerPoseIPPE(bestMId).sols[1]* invM);
//       expectedposes.push_back(pose1);
//       expectedposes.push_back(pose2);

//   }
////    if(!expectedpose.isValid());



//    //so, the frame is at frame.pose_f2g but it should be at orgpose also
//    //loop closure consists in making these two estimations identical by distributing the errors equally amongst the nodes in the
//    //path connecting the orignal location and current one
//    //But, which is the 'original location'? There are many possibilities in our problem, since the marker has been seen from
//    //many other frames probably.
//    //So, let us take as origin node the first one in  the video in which it was seen
//    uint32_t minFrameSeqIdx=std::numeric_limits<uint32_t>::max(),minFrameIdx=std::numeric_limits<uint32_t>::max();
//    for(auto m:lci.markers){
//        assert(TheMap->map_markers.count(m)!=0);//the marker must be and be valid
//        assert(TheMap->map_markers.at(m).pose_g2m.isValid());
//        for(auto frame_idx:TheMap->map_markers[m].frames){
//            if ( minFrameSeqIdx>TheMap->keyframes[frame_idx].fseq_idx){
//                minFrameSeqIdx=TheMap->keyframes[frame_idx].fseq_idx;
//                minFrameIdx=frame_idx;
//            }
//        }
//    }
//    CovisGraph EssentialGraph;
//    TheMap->TheKpGraph.getEG(EssentialGraph);



//   //Build an easy to use set of connections
//   vector<pair<uint32_t,uint32_t> > edges;
//   for(auto nw:EssentialGraph.getEdgeWeightMap()){
//       auto i_j=EssentialGraph.separe(nw.first);
//       edges.push_back({i_j.first,i_j.second});
//   }
//    //finally, add the connection between this and the first one in which one of the markers appeared


//   assert(EssentialGraph.getNodes().count(_curKFRef));
//   assert(EssentialGraph.getNodes().count(minFrameIdx));

//   edges.push_back({_curKFRef,frame.idx});
//   vector<std::map<uint32_t, cv::Mat> >optimPoses(1);
//   for(auto e:edges){
//       if ( optimPoses[0].count(e.first)==0 && TheMap->keyframes.count (e.first)!=0)
//           optimPoses[0][e.first]=TheMap->keyframes[e.first].pose_f2g;
//       if ( optimPoses[0].count(e.second)==0 && TheMap->keyframes.count (e.second)!=0)
//           optimPoses[0][e.second]=TheMap->keyframes[e.second].pose_f2g;
//   }
//   edges.push_back({frame.idx,minFrameIdx});
//   optimPoses[0][frame.idx]=frame.pose_f2g;

//   if (expectedposes.size()==2){
//       optimPoses.push_back(optimPoses[0]);
//       cv::Point3f zero(0,0,0);
//       cout<<"dist Error Solutions="<< cv::norm(expectedposes[0]*zero- expectedposes[1]*zero)<<endl;

//   }



//   for(size_t i=0;i<expectedposes.size();i++)
//       loopclosure::loopClosurePathOptimization_cv(edges,frame.idx,minFrameIdx,expectedposes[i], optimPoses[i]);




//   //now, must also move markers accordingly
//   //get all the markers visible

//   //compute the amount of change for each marker
//   vector<std::map<uint32_t,cv::Mat>> frame_posechange(optimPoses.size());
//   for(size_t v=0;v<optimPoses.size();v++){
//       for(auto op:optimPoses[v])
//       {
//           if (op.first==frame.idx)
//               frame_posechange[v][op.first]=frame.pose_f2g*op.second.inv();
//           else
//               frame_posechange[v][op.first]=TheMap->keyframes[op.first].pose_f2g*op.second.inv();
//       }
//   }

//   //find the displacement that must be applied to each point and marker
//   //using the average dissplacement according to the frames in which they are seen
//   //this process is repeated for each possible solution
//   vector< std::map<uint32_t,pair<se3,zeroinitvar<int> > > > marker_inc(optimPoses.size());
//   vector< std::map<uint32_t,pair<se3,zeroinitvar<int> > > > point_inc(optimPoses.size());
//    for(size_t v=0;v<optimPoses.size();v++){
//       for(auto fidx_pose:optimPoses[v]){
//           if ( TheMap->keyframes.count(fidx_pose.first)){
//               const Frame &keyFrame=TheMap->keyframes[fidx_pose.first];
//               for(auto m:TheMap->keyframes[fidx_pose.first].markers){
//                   //compute marker to frame
//                   auto m2f=  TheMap->map_markers[m.id].pose_g2m.convert().inv()*keyFrame.pose_f2g.inv();
//                   auto m2m=se3(m2f* fidx_pose.second * TheMap->map_markers[m.id].pose_g2m);
//                    auto &minc=marker_inc[v][m.id];
//                    if (minc.second==0)minc.first=m2m;
//                    else minc.first+=m2m;
//                    minc.second++;
//               }

//               se3 thisframe_posechange=frame_posechange[v][keyFrame.idx];
//               for(size_t i=0;i<keyFrame.ids.size();i++){
//                   if (keyFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
//                       auto &pinc=point_inc[v][keyFrame.ids[i]];
//                       if (pinc.second==0) pinc.first=thisframe_posechange;
//                       else pinc.first+=thisframe_posechange;
//                       pinc.second++;
//                   }
//               }
//           }
//       }
//       //average the results
//       for(auto &m_inc:marker_inc[v])
//           m_inc.second.first/= float(m_inc.second.second);//mean of increments
//       for(auto &p_inc:point_inc[v])
//           p_inc.second.first/= float(p_inc.second.second);//mean of increments
//   }


//    TheMap->saveToFile("mappre.map");
//    // test the solutions
//    pair<int,double> best(-1,std::numeric_limits<double>::max());

//    if (optimPoses.size()>1){
//        for(size_t v=0;v<optimPoses.size();v++){
//            std::map<uint32_t,Se3Transform> marker_poses;
//            std::map<uint32_t,cv::Point3f> point_coordinates;

//            auto MapMarkersOrg=TheMap->map_markers;
//            for(auto p:TheMap->map_points)
//                point_coordinates.insert({p.id,p.getCoordinates()});
//            //move markers
//            for(auto m_inc:marker_inc[v])
//                TheMap->map_markers[m_inc.first].pose_g2m=TheMap->map_markers[m_inc.first].pose_g2m.convert()* m_inc.second.first.convert().inv();
//            for(auto p_inc:point_inc[v]){
//                auto &mapPoint=TheMap->map_points[p_inc.first];
//                mapPoint.setCoordinates(  p_inc.second.first.convert()*mapPoint.getCoordinates());
//            }


//            //finally, apply transform to path nodes
//            for(auto op:optimPoses[v]){
//                if ( TheMap->keyframes.count(op.first)){
//                    TheMap->keyframes[op.first].pose_f2g=op.second;
//                }
//            }
//            //move the points


//            for(auto m_inc:marker_inc[v])
//                marker_poses[m_inc.first]=TheMap->map_markers[m_inc.first].pose_g2m.convert()* m_inc.second.first.convert().inv();

//            //compute the error of the solution
//            double chi2=0;
//            for(auto op:optimPoses[v]){
//                //find its
//                if ( TheMap->keyframes.count(op.first)==0) continue;
//                const Frame&frame=TheMap->keyframes[op.first];
//                for(auto marker:frame.und_markers){
//                    if (marker_poses.count(marker.id)==0)continue;
//                    //take the 3d points of the marker in its own reference system
//                    vector<cv::Point3f> markerPoints=TheMap->map_markers[marker.id].get3DPoints(false);//points in marker ref system
//                    //now, move to global with the estimated transofrm, project and compute error
//                    auto f2g=op.second*marker_poses[marker.id];
//                    for(int i=0;i<4;i++){
//                        cv::Point2f err= marker[i]-project( markerPoints[i],frame.imageParams.CameraMatrix,f2g);
//                        chi2+= err.x*err.x+ err.y*err.y;
//                    }
//                }
//            }
//            //   if (chi2<best.second) best={v,chi2};


//            vector<uint32_t> keyframes;
//            for(auto kf:optimPoses[v])keyframes.push_back(kf.first);
//            //remove the last one that is not correct
//            keyframes.pop_back();
//            auto err=TheMap->globalReprojChi2(keyframes,0,0,false,true);
//            _debug_msg("err sol "<<v<<"="<<err ,10);
//            err=TheMap->globalReprojChi2(keyframes,0,0,true,true);
//            _debug_msg("err wkp sol "<<v<<"="<<err ,10);
//            if (err<best.second) best={v,err};
//            TheMap->saveToFile("map"+to_string(v)+".map");

//            TheMap->map_markers=MapMarkersOrg;//restore

//            for(auto &mp:TheMap->map_points)
//                mp.setCoordinates(point_coordinates[mp.id]);

//        }
//    }
//    else{
//        best={0,-1};

//    }



//    //move markers
//    for(auto m_inc:marker_inc[best.first])
//        TheMap->map_markers[m_inc.first].pose_g2m=TheMap->map_markers[m_inc.first].pose_g2m.convert()* m_inc.second.first.convert().inv();
//    //move points
//    for(auto p_inc:point_inc[best.first]){
//        auto &mapPoint=TheMap->map_points[p_inc.first];
//        mapPoint.setCoordinates(  p_inc.second.first.convert()*mapPoint.getCoordinates());
//    }
//    //move keyframes
//    for(auto op:optimPoses[best.first]){
//        if ( TheMap->keyframes.count(op.first)){
//            TheMap->keyframes[op.first].pose_f2g=op.second;
//        }
//    }

////   frame.pose_f2g=optimPoses[best.first][frame.idx];

////   _curKFRef=addKeyFrame(frame,{},_curKFRef);
////   globalOptimization();
////   _curPose_f2g=TheFrameSet[frame.idx].pose_f2g;

//}
//void LoopDetector::correctLoopClosure_Markers(Frame &frame, int32_t curRefKf,uint32_t matchingFrameIdx,cv ::Mat expectedPose){

//}



//void LoopDetector::correctLoopForKeyPoints(Frame &frame, int32_t curRefKf,uint32_t matchingFrameIdx,cv ::Mat expectedPose){
//    auto TheFrameSet=&TheMap->keyframes;
//    //now, well try to solve the problem we have.
//    //current frame has two possible locations,
//    //first, the one estimated with drift (which is in frame.pose)
//    //and the one calculated using the markers seen already (expected)
//     //determine this original location using the frames that caused the loop closure

//    CovisGraph EssentialGraph;
//    TheMap->TheKpGraph.getEG(EssentialGraph);

//    //Build an easy to use set of connections
//    vector<pair<uint32_t,uint32_t> > edges;
//    for(auto nw:EssentialGraph.getEdgeWeightMap()){
//        auto i_j=EssentialGraph.separe(nw.first);
//        edges.push_back({i_j.first,i_j.second});
//    }

//    std::map<uint32_t, cv::Mat> optimPoses;
//    for(auto e:edges){
//        if ( optimPoses.count(e.first)==0 && TheFrameSet->count (e.first)!=0)
//            optimPoses[e.first]=(*TheFrameSet)[e.first].pose_f2g;
//        if ( optimPoses.count(e.second)==0 && TheFrameSet->count (e.second)!=0)
//            optimPoses[e.second]=(*TheFrameSet)[e.second].pose_f2g;
//    }


//    //now, we must add the connection between this frame and the refkeyframe
//    //give this a unique idex
//    frame.idx=std::numeric_limits<uint32_t>::max()-1;
//    edges.push_back({curRefKf,frame.idx});
//    edges.push_back({frame.idx,matchingFrameIdx});
//    optimPoses[frame.idx]=frame.pose_f2g;




//    loopclosure::loopClosurePathOptimization_cv(edges,frame.idx,matchingFrameIdx,expectedPose,optimPoses);
//cout<<"Chi2="<<TheMap->globalReprojChi2()<<endl;
//    for(auto op:optimPoses){
//        if (op.first==std::numeric_limits<uint32_t>::max()-1 )
//            frame.pose_f2g=op.second;
//        else
//            TheMap->keyframes[op.first].pose_f2g=op.second;
//    }
//    cout<<"Chi2="<<TheMap->globalReprojChi2()<<endl;
//    //now, must move all points to propagate the changes


//    //compute for each pose, the amount of change experienced

//    std::map<uint32_t,cv::Mat> node_posechange;
//    for(auto op:optimPoses)
//    {
//        if (op.first==std::numeric_limits<uint32_t>::max()-1 ){
//            node_posechange[op.first]=frame.pose_f2g*op.second.inv();
//        }
//        else
//            node_posechange[op.first]=TheMap->keyframes[op.first].pose_f2g*op.second.inv();
//    }
//    //now, propagate the changes to the points and markers
//    for(auto &mp:TheMap->map_points){
//        //apply one of the transforms only
//        mp.setCoordinates( node_posechange[mp.frames.begin()->first]*mp.getCoordinates());
//    }


//}


//bool MapManager::detectLoopKeyPoints(  Frame &frame, int32_t curRefKf){
//    ScopedTimerEvents Timer("MapManager::detectLoop");
//     if (!Slam::getParams().detectKeyPoints) return false;
//    if (TheMap->TheKFDataBase.isEmpty())return false;

//    auto Neighbors=TheMap->TheKpGraph.getNeighbors(curRefKf,true);
//    cout<<"neighbors=";for(auto n:Neighbors)cout<<n<<" ";cout<<endl;
//    // Compute reference BoW similarity score
//    // This is the lowest score to a connected keyframe in the covisibility graph
//    // We will impose loop candidates to have a higher similarity than this

//   // TheMap->TheKFDataBase.computeBow(frame);
//    float minScore = 1;
//    for(auto neigh:Neighbors)
//        minScore=std::min(minScore, TheMap->TheKFDataBase.score( frame,TheMap->keyframes[neigh]));
//    Timer.add("min score neighbors");

//    auto candidates=TheMap->TheKFDataBase.relocalizationCandidates(frame,TheMap->keyframes,TheMap->TheKpGraph,true,minScore,Neighbors);
//    cout<<"candidates=";for(auto c:candidates)cout<<"("<<c<<","<<TheMap->keyframes[c].fseq_idx<<")" ;cout<<endl;
//  Timer.add("find best candidates");
//    if (candidates.size()==0){
//        loopCandidates.clear();
//        return false;
//    }
//    //if the given candidate has not its own loop candidate, create it
//    for(auto c:candidates){
//        if (loopCandidates.count(c)==0){
//            LoopCandidate LC;
//            LC.nodes= TheMap->TheKpGraph.getNeighbors(c,true);
//            LC.nobs=0;
//            loopCandidates[c]=LC;
//            }
//    }

//    //each candidate votes in the other candidates it is
//    //For each candidate, see if already in the set of activated regions, and if so, increase their voting. Otherwise, create it
//    std::set<uint32_t> observedLoopCandidates;//which loop candidates are observed
//    for(auto c:candidates){
//        //find the candidate loop the keyframe belongs to
//        for( auto &lc: loopCandidates){
//            if (lc.second.nodes.count(c)){//is the node in this loop candidate?
//                lc.second.nobs++;
//                observedLoopCandidates.insert(lc.first);
//            }
//        }
//    }
////    //now update observations
//    for( auto &lc:loopCandidates) lc.second.timesUnobserved++;
//    for(auto obc:observedLoopCandidates)
//        loopCandidates[obc].timesUnobserved=0;

////    //remove unobserved for too long
//    for(auto it=loopCandidates.begin();it!=loopCandidates.end();){
//        if (it->second.timesUnobserved>=1)
//            it=loopCandidates.erase(it);
//        else it++;
//    }
////


//    //check for any candidate has strong evidence
//    struct Candidate{uint32_t frame;cv::Mat pose;uint32_t nInliers;};
//    vector<Candidate> goodLoopCandidates;
//    for(auto lc:candidates)
//        if (loopCandidates[lc].nobs>15)
//            goodLoopCandidates.push_back({lc,cv::Mat(),0});

//    //sort by strength
//    //std::sort( goodLoopCandidates.begin(),goodLoopCandidates.end(),[&](uint32_t a,uint32_t b){ return loopCandidates[a].nobs<loopCandidates[b].nobs; });

//     _debug_exec(5, cout<<"goodCandidates="<<goodLoopCandidates.size()<<endl;);
//    Timer.add("find goodLoopCandidates  ");
//    if (goodLoopCandidates.size()!=0){
//        cout<<"goodCandidates=";for(auto &cand:goodLoopCandidates)cout<<cand.frame<<" ";cout<<endl;
//        xflann::Index FrameXFlannIndex (frame.desc,xflann::HKMeansParams(8,0));
//        for(auto &cand:goodLoopCandidates){

//  //          vector<cv::DMatch> matches=TheMap->matchFrameToKeyFrame(frame,cand.frame,Slam::getParams().minDescDistance*2);
//            vector<cv::DMatch> matches=TheMap->matchFrameToKeyFrame(frame,cand.frame,Slam::getParams().minDescDistance*2,FrameXFlannIndex);
//            cout<<"cand="<<cand.frame<<" matches="<<matches.size()<<endl;
//             Timer.add(" matchFrameToKeyFrame");
//            if(matches.size()<50){ //if not enough matches, mark for removal
//                cand.frame=std::numeric_limits<uint32_t>::max();
//                continue;
//            }
//            vector<cv::Point2f> points2d;
//            vector<cv::Point3f> points3d;
//            for(auto m:matches){
//                points2d.push_back(frame.und_kpts[m.queryIdx].pt);
//                points3d.push_back(TheMap->map_points[ m.trainIdx].getCoordinates());
//            }
//            cv::Mat rv,tv;
//            vector<int> inliers;
//            bool res=cv::solvePnPRansac(points3d,points2d, frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,false,100,1.5,0.99,inliers);
//            cout<<"  inliers="<<inliers.size()<<endl;
//            Timer.add(" pnpransac");
//            if (inliers.size()<30 || !res){
//                cand.frame=std::numeric_limits<uint32_t>::max();
//                continue;
//            }
//            cand.nInliers=inliers.size();
//            cand.pose=getRTMatrix(rv,tv,CV_32F);
//        }
//        //remove invalid
//        goodLoopCandidates.erase(std::remove_if(goodLoopCandidates.begin(),goodLoopCandidates.end(),[](const Candidate& c){return c.frame==std::numeric_limits<uint32_t>::max();}),goodLoopCandidates.end());

//        if (goodLoopCandidates.size()==0) return false;
//        //sort by number of inliers
//        std::sort(goodLoopCandidates.begin(),goodLoopCandidates.end(),[](const Candidate &c1,const Candidate &c2){return c1.nInliers>c2.nInliers;});
//        if( goodLoopCandidates.size()>1){assert(goodLoopCandidates[0].nInliers>goodLoopCandidates[1].nInliers);}
//        for(auto c:goodLoopCandidates)cout<<c.frame<<" ";cout<<endl;


//        //ok, go for it. Let us correct
//        //refine even further the pose
//        for(auto &mp:TheMap->map_points) mp.getExtra<uint32_t>()[0]==std::numeric_limits<uint32_t>::max();
//        auto map_matches =TheMap->matchFrameToMapPoints(TheMap->TheKpGraph.getNeighborsV(goodLoopCandidates[0].frame,true), frame,  goodLoopCandidates[0].pose ,Slam::getParams().minDescDistance*1.5, 2.5,false,true);

//        //now do a fine estimation of the pose
//        vector<cv::Point3f> points3d;
//        vector<cv::KeyPoint> kpoints;
//        for(auto match:map_matches){
//            points3d.push_back( TheMap->map_points[match.trainIdx].getCoordinates());
//            kpoints.push_back(frame.und_kpts[match.queryIdx]);
//        }
//        vector<bool> vBadMatches;
//        se3 pse3=goodLoopCandidates[0].pose;
//        poseEstimation(points3d,kpoints,pse3,vBadMatches,frame.imageParams.CameraMatrix,frame.getScaleFactor());
//        goodLoopCandidates[0].pose=pse3;
//        remove_bad_matches(map_matches,vBadMatches);
//        //how many good matches??
//        if (map_matches.size()<40) return false;


//        /////////////////////////////////////////////////////////
//        /// \brief correctLoop
//        ///
//        ///
//        correctLoop(frame,curRefKf, goodLoopCandidates[0].frame,goodLoopCandidates[0].pose);
//        auto neighborsSideOld=TheMap->TheKpGraph.getNeighbors(goodLoopCandidates[0].frame,true);

//        //remove possible connection with other side of the loop

//        for(auto &id:frame.ids)
//            if (id!=std::numeric_limits<uint32_t>::max()){
//                for(auto n:neighborsSideOld)
//                    if (TheMap->map_points[id].isObservingFrame(n))
//                         id=std::numeric_limits<uint32_t>::max() ;
//            }


//        //add the frame and the points shared between both views
//        auto &NewFrame=TheMap->addKeyFrame(frame,true);
//        for(size_t i=0;i< NewFrame.ids.size();i++)
//            if (NewFrame.ids[i]!=std::numeric_limits<uint32_t>::max())
//                TheMap->addMapPointObservation(NewFrame.ids[i],NewFrame.idx,i);
//        //add to the frame, the points seen in the other side of the loop
//        for(auto match:map_matches)
//            if (NewFrame.ids[match.queryIdx]==std::numeric_limits<uint32_t>::max())
//                TheMap->addMapPointObservation(match.trainIdx,NewFrame.idx,match.queryIdx);




//        globalOptimization(10);


//        //estimate the position again and find matches

//            for(auto &mp:TheMap->map_points)
//                mp.getExtra<uint32_t>()[0]==std::numeric_limits<uint32_t>::max();
//            //set to avoid visiting the points already marked in the NewFrame projected
//            for(auto id:NewFrame.ids)
//                if (id!=std::numeric_limits<uint32_t>::max())
//                    TheMap->map_points[id].getExtra<int>()[0]=NewFrame.fseq_idx;

//            auto neighborsSideNew=TheMap->TheKpGraph.getNeighbors( curRefKf,true);
//            auto allNeighbors=neighborsSideNew;
//            for(auto n:neighborsSideOld)allNeighbors.insert(n);
//            allNeighbors.insert(NewFrame.idx);
//            vector<uint32_t> allNeighborsV;
//            for(auto n:allNeighbors)allNeighborsV.push_back(n);

//            auto map_matches2 =TheMap->matchFrameToMapPoints(allNeighborsV, NewFrame,  NewFrame.pose_f2g,Slam::getParams().minDescDistance*1.5, 1.5,false,false);
//            //add this new projections
//            //add connections to other frames



//            for(auto match:map_matches2){
//                if (NewFrame.ids[match.queryIdx]==std::numeric_limits<uint32_t>::max())
//                    TheMap->addMapPointObservation(match.trainIdx,NewFrame.idx,match.queryIdx);
//            }
//            auto toRemove=searchInNeighbors(NewFrame);
//            TheMap->removePoints(toRemove.begin(),toRemove.end());
////            fusepoints(curRefKf,goodLoopCandidates[0].frame);
//     //       assert(TheMap->checkConsistency());
//            globalOptimization(10);
//            frame.pose_f2g=NewFrame.pose_f2g;
//            frame.ids=NewFrame.ids;

//        //find correspondences between the two sides of the loop to do the fusion

//       //  fusepoints(curRefKf,goodLoopCandidates[0].frame);
//     //    assert(TheMap->checkConsistency());
//         loopCandidates.clear();
//         //we are removing the frame and will be added next
//         //TheMap->removeKeyFrames({NewFrame.idx},Slam::getParams().minNumProjPoints);
//     //    assert(TheMap->checkConsistency());
//        return true;

//    }
//    return false;
//}

//void MapManager::correctLoop(Frame &frame, int32_t curRefKf,uint32_t matchingFrameIdx,cv ::Mat expectedPose){
//    auto TheFrameSet=&TheMap->keyframes;
//    //now, well try to solve the problem we have.
//    //current frame has two possible locations,
//    //first, the one estimated with drift (which is in frame.pose)
//    //and the one calculated using the markers seen already (expected)
//     //determine this original location using the frames that caused the loop closure

//    CovisGraph EssentialGraph;
//    TheMap->TheKpGraph.getEG(EssentialGraph);

//    //Build an easy to use set of connections
//    vector<pair<uint32_t,uint32_t> > edges;
//    for(auto nw:EssentialGraph.getEdgeWeightMap()){
//        auto i_j=EssentialGraph.separe(nw.first);
//        edges.push_back({i_j.first,i_j.second});
//    }

//    std::map<uint32_t, cv::Mat> optimPoses;
//    for(auto e:edges){
//        if ( optimPoses.count(e.first)==0 && TheFrameSet->count (e.first)!=0)
//            optimPoses[e.first]=(*TheFrameSet)[e.first].pose_f2g;
//        if ( optimPoses.count(e.second)==0 && TheFrameSet->count (e.second)!=0)
//            optimPoses[e.second]=(*TheFrameSet)[e.second].pose_f2g;
//    }


//    //now, we must add the connection between this frame and the refkeyframe
//    //give this a unique idex
//    frame.idx=std::numeric_limits<uint32_t>::max()-1;
//    edges.push_back({curRefKf,frame.idx});
//    edges.push_back({frame.idx,matchingFrameIdx});
//    optimPoses[frame.idx]=frame.pose_f2g;




//    loopclosure::loopClosurePathOptimization_cv(edges,frame.idx,matchingFrameIdx,expectedPose,optimPoses);
//cout<<"Chi2="<<TheMap->globalReprojChi2()<<endl;
//    for(auto op:optimPoses){
//        if (op.first==std::numeric_limits<uint32_t>::max()-1 )
//            frame.pose_f2g=op.second;
//        else
//            TheMap->keyframes[op.first].pose_f2g=op.second;
//    }
//    cout<<"Chi2="<<TheMap->globalReprojChi2()<<endl;
//    //now, must move all points to propagate the changes


//    //compute for each pose, the amount of change experienced

//    std::map<uint32_t,cv::Mat> node_posechange;
//    for(auto op:optimPoses)
//    {
//        if (op.first==std::numeric_limits<uint32_t>::max()-1 ){
//            node_posechange[op.first]=frame.pose_f2g*op.second.inv();
//        }
//        else
//            node_posechange[op.first]=TheMap->keyframes[op.first].pose_f2g*op.second.inv();
//    }
//    //now, propagate the changes to the points and markers
//    for(auto &mp:TheMap->map_points){
//        //apply one of the transforms only
//        mp.setCoordinates( node_posechange[mp.frames.begin()->first]*mp.getCoordinates());
//    }


//}
}
