#include <list>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/markermap.h>
#include "slam.h"
#include "stuff/utils.h"
#include "stuff/debug.h"
#include "stuff/timers.h"
#include "optimization/solvepnp.h"
 #include "optimization/globaloptimizer.h"
#include "optimization/weigthed_ba.h"
#include "optimization/ippe.h"
#include "optimization/levmarq.h"
#include "optimization/sparselevmarq.h"
#include "stuff/io_utils.h"
#include "stuff/framedatabase.h"
#include "stuff/mapinitializer.h"
#include "optimization/loopclosure.h"
namespace ucoslam{
 Params Slam::_params;



Slam::Slam(){
     map_initializer=std::make_shared<MapInitializer>();
     num_of_image = 0;
     num_of_success = 0;
}

Slam::~Slam(){

    waitForFinished();
}
void Slam::setParams( std::shared_ptr<Map> map, const  Params &p,const string &vocabulary){

    TheMap=map;


    _params=p;

     aruco::MarkerDetector::Params md_params;


    fextractor.setParams( _params);
    fextractor.removeFromMarkers()=p.removeKeyPointsIntoMarkers;
    fextractor.detectMarkers()=p.detectMarkers;
     //now, the vocabulary






    if (TheMap->isEmpty()){//Need to start from zero

        currentState=STATE_ZERO;
        if (!vocabulary.empty() )
            TheMap->TheKFDataBase.loadFromFile(vocabulary);
        MapInitializer::Params params;
        if ( _params.forceInitializationFromMarkers)
            params.mode=MapInitializer::ARUCO;
        else
            params.mode=MapInitializer::BOTH;

        params.minDistance= _params.minBaseLine;
        params.markerSize=_params.aruco_markerSize;
        params.aruco_minerrratio_valid= _params.aruco_minerrratio_valid;
        params.allowArucoOneFrame=_params.aruco_allowOneFrameInitialization;
        params.max_makr_rep_err= _params.max_makr_rep_err;
        params.minDescDistance=_params.minDescDistance;
        params.min_deltaR = _params.min_distR;
        params.max_theta = _params.max_theta;
        map_initializer->setParams(params);


    }
    else
        currentState=STATE_LOST;

}
void Slam::waitForFinished(){
 TheMapManager.stop();
}

cv::Mat Slam::process(const Frame &frame) {
    assert(TheMap->checkConsistency());

    //copy the current frame if not calling from the other member funtion
    ///
    if ((void*)&frame!=(void*)&_cFrame){//only if not calling from the other process member function
        swap(_prevFrame,_cFrame);
        _cFrame=frame;
    }


    _debug_exec(20, saveToFile("world-prev.ucs"););
    _debug_exec(5,
                switch(currentState){
                    case STATE_ZERO:_debug_msg_("STATEZERO");break;
                    case STATE_TRACKING:_debug_msg_("STATE_TRACKING");break;
                    case STATE_LOST:_debug_msg_("STATE_LOST");break;
                };);


    ScopedTimerEvents tevent("Slam::process2");


    num_of_image++;
    //Initialize other processes if not yet
    ///初始化地图
    if (currentMode==MODE_SLAM  && !TheMapManager.hasMap())
        TheMapManager.setMap(TheMap);
    ///开始地图管理线程：runThread()
    if (!_params.runSequential && currentMode==MODE_SLAM )
        TheMapManager.start();

    //update map if required
    ///更新地图（检测回环，默认不执行，除非强制更新）
    Se3Transform pose=TheMapManager.mapUpdate();
    if (!pose.empty())//if a pose is returned, a loopclosure has happened and it is better to update the current pose
        _cFrame.pose_f2g=pose;
    tevent.add("map updated");

    //remove possible references to removed mappoints in _prevFrame
    for(auto &id:_prevFrame.ids)
        if (id!=std::numeric_limits<uint32_t>::max()){
            if (!TheMap->map_points.is(id)) id=std::numeric_limits<uint32_t>::max();
            else if( TheMap->map_points[id].isBad)id=std::numeric_limits<uint32_t>::max();
        }
    tevent.add("removed invalid map references");


    _debug_msg_("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" );
    _debug_msg_("|||||||||||||||||||||| frame="<<frame.fseq_idx<<" sig="<<sigtostring(frame.getSignature())<<"  Wsig="<<sigtostring(getSignature()));
    _debug_msg_("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" );


    _debug_msg_("SIG="<<sigtostring(_cFrame.getSignature()));


    //not initialized yet
    ///Step1,初始化
    if(currentState==STATE_ZERO && currentMode==MODE_SLAM){
        if ( initialize(_cFrame))
            currentState=STATE_TRACKING;
        tevent.add("initialization attempted");
    }
    ///Step2, 跟踪
    else{
        //tracking mode
        ///Step2,跟踪
        if( currentState==STATE_TRACKING){
            ///Step2.1 给上一帧选择参考关键帧
            _curKFRef=getBestReferenceFrame(_prevFrame,_curPose_f2g);//此时_curPose_f2g为上一帧的位姿
            cout<<"Ref id "<<_curKFRef<<endl;
            ///Step2.2 计算当前帧位姿
            _curPose_f2g=track(_cFrame,_curPose_f2g);//此时由上一帧的位姿计算当前帧的位姿并给到_curPose_f2g
            _debug_msg_("current pose="<<_curPose_f2g);
            tevent.add("track");
            if( !_curPose_f2g.isValid())
                currentState=STATE_LOST;
        }
        ///Step2.3 重定位
        if (currentState==STATE_LOST){
            se3 reloc_pose;
            if ( relocalize(_cFrame,reloc_pose)){
                currentState=STATE_TRACKING;
                _curPose_f2g=reloc_pose;
                _curKFRef=getBestReferenceFrame(_cFrame,_curPose_f2g);
            }
            tevent.add("relocalize");
        }


        //Step3 更新地图，局部优化，并进行回环检测
        //update map
        if( currentState==STATE_TRACKING)
        {
            num_of_success++;
            _cFrame.pose_f2g=_curPose_f2g;
            if (currentMode==MODE_SLAM)
               TheMapManager.newFrame(_cFrame,_curKFRef);
            tevent.add("newFrame");
            _debug_msg_("_curKFRef="<<_curKFRef);
        }
    }

    tevent.add("track/initialization done");
    frame_pose[_cFrame.fseq_idx]= _curPose_f2g;
    _cFrame.pose_f2g=_curPose_f2g;
    _debug_msg_("camera pose="<<_curPose_f2g);
    assert(TheMap->checkConsistency(debug::Debug::getLevel()>=10));

    if (currentState==STATE_LOST || currentState==STATE_ZERO)return cv::Mat();
    else return _curPose_f2g;
}

cv::Mat Slam::process(  cv::Mat &in_image, const ImageParams &in_ip,uint32_t frameseq_idx, const cv::Mat & depth) {

    ScopedTimerEvents tevent("Slam::process");

    assert( (in_ip.bl>0 && !depth.empty()) || depth.empty());//if rgbd, then ip must  have the params set
    cv::Mat currentImage;
    float Effectivefocus=-1;


    //Resize image to meet the effective focus criteria
    ///获取有效焦距
    if (TheMap->getEffectiveFocus()>0)//already set in the map
        Effectivefocus=TheMap->getEffectiveFocus();
    else if (_params.effectiveFocus>0)//not set in the map, first frame not yet added, but set a desired value in params
        Effectivefocus=_params.effectiveFocus;
    else  Effectivefocus=in_ip.fx(); //no first frame and not desired value used. Use the current camera. Do nothing


    ImageParams ip=in_ip;
    ip.aprilTag = in_ip.aprilTag;
    float scale=1.;//resize image scale
    if (Effectivefocus!=ip.fx()){
        scale=    Effectivefocus/ip.fx();
        cv::Size newSize(scale*in_image.cols,scale*in_image.rows);
        cv::resize(in_image,currentImage,newSize);
        ip.resize(newSize);
    }
    else currentImage=in_image;


    swap(_prevFrame,_cFrame);


    tevent.add("Input preparation");
    //update the map while processing this new frame to extract keypoints an markers
    std::thread UpdateThread;
    if (currentMode==MODE_SLAM) UpdateThread=std::thread([&](){TheMapManager.mapUpdate();});
    ///Step1, 将image包装给_cFrame
    cout<<"***Image "<<frameseq_idx<<"***\n";
    //determine the internal id that will be given to this frame
    ///Step2, 提取角点
    if (depth.empty()) fextractor.process(currentImage,ip,_cFrame,frameseq_idx );//c
    else  fextractor.process_rgbd(currentImage,depth,ip,_cFrame,frameseq_idx );
    tevent.add("frame extracted ");
    if (currentMode==MODE_SLAM)UpdateThread.join(); //wait for update
   //assert(Map::singleton()->checkConsistency(false));
    tevent.add("map updated ");

   // TheMap->TheKFDataBase.relocalizationCandidates(  _cFrame,  TheMap->keyframes,TheMap->TheKpGraph,true);

    ///Start tracking
    cv::Mat result=process(_cFrame);
    //draw matches in image
    tevent.add("process");


    drawMatches(in_image,1./scale);
    putText(in_image,"Map Points:"+to_string(TheMap->map_points.size()),cv::Point(20,in_image.rows-20));
    putText(in_image,"Map Markers:"+to_string(TheMap->map_markers.size()),cv::Point(20,in_image.rows-40));
    putText(in_image,"KeyFrames:"+to_string(TheMap->keyframes.size()),cv::Point(20,in_image.rows-60));
    int nmatches=0;
    for(auto id:_cFrame.ids) if(id!=std::numeric_limits<uint32_t>::max()) nmatches++;
    putText(in_image,"Matches:"+  to_string(nmatches),cv::Point(20,in_image.rows-80));


    tevent.add("draw");

    return result;

}

void  Slam::putText(cv::Mat &im,string text,cv::Point p ){
    float fact=float(im.cols)/float(1280);

    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(125,255,255),1*fact);

}

uint64_t Slam::getSignature()const{

//    cout<<"SIGS= "<<TheMap->getSignature()<<" | "<<_params.getSignature()<<" | "<<_curPose_f2g.getSignature()<<" "<<endl;
//    cout<<"SIGS2="<<_curKFRef<<" "<<_cFrame.getSignature()<<endl;
//    for(auto fp:frame_pose)
//        cout<<"SIG3="<<fp.first<<" "<<fp.second.getSignature()<<endl;

    uint64_t sig=TheMap->getSignature();
    sig+=_params.getSignature();
    sig+=_curPose_f2g.getSignature();
    sig+=_curKFRef;
    sig+=_cFrame.getSignature();
    for(auto fp:frame_pose)
        sig+=fp.first+fp.second.getSignature();
     return sig;
}

ostream& operator<<(ostream& os, const Slam & w)
{
    os << "Info about the world:" << endl;
    os << "+ Keyframes: " << w.TheMap->keyframes.size() << endl;
    os << "+ Map points: " << w.TheMap->map_points.size() << endl;
    return os;
}

//given a frame and a map, returns the set pose using the markers
//If not possible, return empty matrix
//the pose matrix returned is from Global 2 Frame
cv::Mat Slam::getPoseFromMarkersInMap(const Frame &frame ){
    std::vector<uint32_t> validmarkers;//detected markers that are in the map

    //for each marker compute the set of poses
    vector<pair<cv::Mat,double> > pose_error;
    vector<cv::Point3f> markers_p3d;
    vector<cv::Point2f> markers_p2d;

    for(auto m:frame.und_markers){
        if (TheMap->map_markers.find(m.id)!=TheMap->map_markers.end()){
            ucoslam::Marker &mmarker=TheMap->map_markers[m.id];
            cv::Mat Mm_pose_g2m=mmarker.pose_g2m;
            //add the 3d points of the marker
            auto p3d=mmarker.get3DPoints();
            markers_p3d.insert(markers_p3d.end(),p3d.begin(),p3d.end());
            //and now its 2d projection
            markers_p2d.insert(markers_p2d.end(),m.begin(),m.end());

            auto poses_f2m=IPPE::solvePnP(_params.aruco_markerSize,m,frame.imageParams.CameraMatrix,frame.imageParams.Distorsion);
            for(auto pose_f2m:poses_f2m)
                pose_error.push_back(   make_pair(pose_f2m * Mm_pose_g2m.inv(),-1));
        }
    }
    if (markers_p3d.size()==0)return cv::Mat();
    //now, check the reprojection error of each solution in all valid markers and take the best one
    for(auto &p_f2g:pose_error){
        vector<cv::Point2f> p2d_reprj;
        se3 pose_se3=p_f2g.first;
        project(markers_p3d,frame.imageParams.CameraMatrix,pose_se3.convert(),p2d_reprj);
//        cv::projectPoints(markers_p3d,pose_se3.getRvec(),pose_se3.getTvec(),TheImageParams.CameraMatrix,TheImageParams.Distorsion,p2d_reprj);
        p_f2g.second=0;
        for(size_t i=0;i<p2d_reprj.size();i++)
            p_f2g.second+= (p2d_reprj[i].x- markers_p2d[i].x)* (p2d_reprj[i].x- markers_p2d[i].x)+ (p2d_reprj[i].y- markers_p2d[i].y)* (p2d_reprj[i].y- markers_p2d[i].y);
    }

    //sort by error

    std::sort(pose_error.begin(),pose_error.end(),[](const pair<cv::Mat,double> &a,const pair<cv::Mat,double> &b){return a.second<b.second; });
    //    for(auto p:pose_error)
    //    cout<<"p:"<<p.first<<" "<<p.second<<endl;
    return pose_error[0].first;//return the one that minimizes the error
}



bool Slam::initialize( Frame &f2 ) {
bool res;
    if (f2.imageParams.isStereoCamera()){
         res=initialize_stereo(f2);
    }
    else{
        res=initialize_monocular(f2);
    }

    if(!res)return res;
    _curPose_f2g= TheMap->keyframes.back().pose_f2g;
    _curKFRef=TheMap->keyframes.back().idx;
    _debug_msg_("Initialized");
    isInitialized=true;
    assert(TheMap->checkConsistency(true));

  return true;
}

bool Slam::initialize_monocular(Frame &f2 ){

    _debug_msg_("initialize   "<<f2.und_markers.size());


    if (!map_initializer->process(f2,TheMap)) return false;

    //If there keypoints  and at least 2 frames
    // set the ids of the visible elements in f2, which will used in tracking
        if ( TheMap->keyframes.size()>1 && TheMap->map_points.size()>0){
        f2.ids = TheMap->keyframes.back().ids;
    }
    assert(TheMap->checkConsistency());


    globalOptimization();
    assert(TheMap->checkConsistency(true));

    //if only with matches, scale to have an appropriate mean
    if (TheMap->map_markers.size()==0){
        if ( TheMap->map_points.size()<50) {//enough points after the global optimization??
            TheMap->clear();
            return false;
        }


        float invMedianDepth=1./TheMap->getFrameMedianDepth(TheMap->keyframes.front().idx);
        cv::Mat Tc2w=TheMap->keyframes.back().pose_f2g.inv();


        //change the translation
        // Scale initial baseline
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        TheMap->keyframes.back().pose_f2g=Tc2w.inv();

         // Scale points
         for(auto &mp:TheMap->map_points){
            mp.scalePoint(invMedianDepth);
        }

    }

     return true;
}



bool Slam::initialize_stereo( Frame &frame){
    ScopedTimerEvents tevent("Slam::initialize_stereo");


    frame.nonMaximaSuppresion();
    int nValidPoints=0;
    for(size_t i=0;i<frame.und_kpts.size();i++){
        //check the stereo depth
        if (frame.depth[i]>0 && frame.imageParams.isClosePoint(frame.depth[i]) && !frame.nonMaxima[i])
            nValidPoints++;
    }

    if ( nValidPoints<100) return false;


    Frame & kfframe1=TheMap->addKeyFrame(frame); //[frame2.idx]=frame2;

    _debug_msg_("SIG="<<sigtostring(frame.getSignature()));
    //now, get the 3d point information from the depthmap
    for(size_t i=0;i<frame.und_kpts.size();i++){
        //check the stereo depth
        cv::Point3f p ;
         if ( frame.depth[i]>0 && frame.imageParams.isClosePoint(frame.depth[i]) && !frame.nonMaxima[i]){
            //compute xyz
            p=frame.get3dStereoPoint(i);
            auto &mp= TheMap->addNewPoint(kfframe1.fseq_idx);
            mp.isStable=false;
            mp.kfSinceAddition=1;
            mp.pose=p;
            mp.isStereo=true;
            TheMap->addMapPointObservation(mp.id,kfframe1.idx,i);
            frame.ids[i]=mp.id;
        }
    }
    return true;
}

string Slam::sigtostring(uint64_t sig){
    string sret;
    string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
    uchar * s=(uchar *)&sig;
    int n=sizeof(sig)/sizeof(uchar );
    for(int i=0;i<n;i++){
        sret.push_back(alpha[s[i]%alpha.size()]);
    }
    return sret;
}



pair<uint32_t,se3> Slam::getLastValidPose()const{
    for(auto  it=frame_pose.rbegin();it!=frame_pose.rend();it++){
        if (it->second.isValid())
            return *it;
    }
    return  {std::numeric_limits<uint32_t>::max(),se3()};
}
float Slam::computeSpeed()const{
    //try to find the last 10 consecutive valid elements
    vector<se3> consqposes;consqposes.reserve(10);
    for(auto  it=frame_pose.rbegin();it!=frame_pose.rend();it++){
        if (it->second.isValid()){
            consqposes.push_back(it->second);
            if ( consqposes.size()==5) break;
        }
        else consqposes.clear();
    }


    float speedSum=0;
    if (consqposes.size()>=2){
      for(size_t i=1;i<consqposes.size();i++)
          speedSum+=consqposes[i-1].t_dist( consqposes[i]);
      speedSum/=float(consqposes.size()-1);
    }
    else speedSum=0.1;
 return speedSum;
}
template<typename T>
struct zeroinitvar{
    operator T (){return val;}
    operator const T ()const{return val;}
    void operator++(int){val++;}
    T val=0;
};
///获取最佳参考帧
uint32_t Slam::getBestReferenceFrame(const Frame &curKeyFrame,  const se3 &curPose_f2g){
    ScopeTimer Timer("Slam::getBestReferenceFrame");
    //    if( map_matches.size()==0 && TheMap->map_markers.size()==0)return _curKFRef;

    //try with the markers now
    if ( TheMap->map_markers.size()==0){//NO MARKERS!!
        assert(_curKFRef!=-1);
        return _curKFRef;
    }
    //determine all valid markers seen here
    ///Step1, 计算该帧中在地图中的有效marker
    vector<uint32_t> validMarkers;
    for(auto m:curKeyFrame.und_markers){
        auto marker=  TheMap->map_markers.find(m.id);
        if ( marker!=TheMap->map_markers.end()){
            if (marker->second.pose_g2m.isValid()) validMarkers.push_back(m.id);
        }
    }

    if ( validMarkers.size()==0){//NO validmarkers
        assert(_curKFRef!=-1);
        return _curKFRef;
    }
    ///Step2, 从这些Marker的关键帧中选择距离最小的帧
    pair<uint32_t,float> nearest_frame_dist(std::numeric_limits<uint32_t>::max(),std::numeric_limits<float>::max());
//    pair<uint32_t,double> nearest_frame_theta(std::numeric_limits<uint32_t>::max(),std::numeric_limits<double>::max());
    for(auto marker:validMarkers)
        for(const auto &frame:TheMap->map_markers[marker].frames){
            assert(TheMap->keyframes.is(frame));

            auto d=   curPose_f2g .t_dist( TheMap->keyframes[frame].pose_f2g );
            if ( nearest_frame_dist.second>d) nearest_frame_dist={frame,d};
            ///BIT ADD
//            double theta = computeDeltaTheta(curKeyFrame.markers,
//                                             TheMap->keyframes[frame].markers,
//                                             curKeyFrame.imageParams.undistorted(),
//                                             _params.aruco_markerSize);
//            if (nearest_frame_theta.second > theta && d < 5*_params.minBaseLine) nearest_frame_theta = {frame, theta};
        }
//    if (nearest_frame_theta.first == std::numeric_limits<uint32_t>::max())
//        return _curKFRef;
    return nearest_frame_dist.first;


}

bool pnpRansac(std::vector<cv::Point3f > &points3d,std::vector<cv::Point2f > &points2d, const cv::Mat &cameraParams,const cv::Mat &dist,cv::Mat &rout,cv::Mat& tout,size_t minInliers,vector<int> &inliers){
    std::vector<int> indices(points3d.size());
    auto method=cv:: SOLVEPNP_EPNP    ;
    int nsamples=8;
    int ntimes=100;
    if (points3d.size()<size_t(nsamples*2)) return false;

    for(size_t i=0;i<indices.size();i++)
        indices[i]=i;

    vector<cv::Point3f> p3d(nsamples);
    vector<cv::Point2f> p2d(nsamples);
    inliers.reserve(points3d.size());
    int cindex=0;
    cv::Mat rv,tv;
     for(int i=0;i<ntimes;i++){
        if ( i==0|| int(indices.size()-cindex)<nsamples){
            std::random_shuffle(indices.begin(),indices.end());
            cindex=0;
        }
        for(int p=0;p<nsamples;p++){
            p3d[p]= points3d[indices[cindex]];
            p2d[p]= points2d[indices[cindex]];
         //   cout<<p3d[p]<<" "<<p2d[p]<<" "<<indices[cindex]<<endl;
            cindex++;
        }
         bool res=cv::solvePnP(p3d,p2d, cameraParams,dist,rv,tv,false,method);
         if(!res) continue;
     //   cout<<"SOLVED="<<res<<endl;
    //    cout<< endl<<rv<<endl<<tv<<endl;
        //find inliers
        vector<cv::Point2f> proj;
        cv::projectPoints(points3d,rv,tv,cameraParams,dist,proj);
         //count number of inliers
        inliers.clear();
        for(size_t p=0;p<proj.size();p++){
            if ( cv::norm(proj[p]-points2d[p])<2.5)
                inliers.push_back(p);
        }
      //  cout<<"DONE"<<endl;
         if (inliers.size()>minInliers){
            rv.convertTo(rout,CV_32F);
            tv.convertTo(tout,CV_32F);
            return true;
        }

    }
    inliers.clear();

    return false;

}

bool Slam::relocalize_withkeypoints( Frame &curFrame,se3 &pose_f2g_out ){
    if (curFrame.ids.size()==0)return false;
    if (TheMap->TheKFDataBase.isEmpty())return false;
    vector<uint32_t> kfcandidates=TheMap->relocalizationCandidates(curFrame);
    xflann::Index xfindex(curFrame.desc, xflann::HKMeansParams(32,0));
    if (kfcandidates.size()==0)return false;
    struct solution{
        se3 pose;
        int nmatches=0;
        std::vector<uint32_t> ids;
        std::vector<cv::DMatch> matches;
    };
    vector<solution> Solutions(kfcandidates.size());

#pragma omp parallel for
    for(size_t cf=0;cf<kfcandidates.size();cf++){

        int kf=kfcandidates[cf];
        Solutions[cf].matches=TheMap->matchFrameToKeyFrame(curFrame,kf,getParams().minDescDistance*2,xfindex);

        if (Solutions[cf].matches.size()<25)continue;

        solvePnPRansac(curFrame,TheMap,Solutions[cf].matches,Solutions[cf].pose);
        if (Solutions[cf].matches.size()<15) continue;

        Solutions[cf].matches =TheMap->matchFrameToMapPoints(TheMap->TheKpGraph.getNeighborsV( kf,true), curFrame, Solutions[cf].pose,_params.minDescDistance, 2.5,false);
        solvePnp(curFrame,TheMap,Solutions[cf].matches,Solutions[cf].pose);
        if (Solutions[cf].matches.size()<10) continue;
        Solutions[cf]. ids=curFrame.ids;
        for(auto match: Solutions[cf].matches)
            Solutions[cf].ids[ match.queryIdx]=match.trainIdx;
    }

    //take the solution with more matches
    std::sort( Solutions.begin(),Solutions.end(),[](const solution &a,const solution &b){return a.matches.size()>b.matches.size();});

    if (Solutions[0].matches.size()>30){
        pose_f2g_out=Solutions[0].pose;
        curFrame.ids=Solutions[0].ids;
        return true;
    }
    else return false;

}

bool Slam::relocalize_withmarkers( Frame &f,se3 &pose_f2g_out )
{
     _debug_msg_("Must consider the case of loop closure about to be!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    if (f.und_markers.size()==0)return false;
    //find the visible makers that have valid 3d location
    vector< uint32_t> valid_markers_found;
    for(  auto &m:f.und_markers){//for ech visible marker
        auto map_marker_it=TheMap->map_markers.find(m.id);
        if(map_marker_it!=TheMap->map_markers.end())//if it is in the map
            if ( map_marker_it->second.pose_g2m.isValid() )//and its position is valid
                valid_markers_found.push_back( m.id);
    }
    if (valid_markers_found.size()==0)
    {
        cout<<"relocalize no valid marker!\n";
        return false;
    }


    pose_f2g_out= TheMap->getBestPoseFromValidMarkers(f,valid_markers_found,
                                                    _params.aruco_minerrratio_valid,
                                                    _params.max_theta);
    ///BIT ADD
    if (_curKFRef != -1)
    {
        float baseline = cv::norm(pose_f2g_out.getTvec()-
                                  TheMap->keyframes[_curKFRef].pose_f2g.getTvec());
        cout << "relocalize baseline = " << baseline << endl;
        if (baseline > 20*Slam::getParams().minBaseLine)
        {
            cout << "track baseline = " << baseline << endl;
            cout << "track failed!\n";
            return false;
        }
    }
    return pose_f2g_out.isValid();
}



 bool Slam::relocalize(Frame &f, se3 &pose_f2g_out){
    pose_f2g_out=se3();
    if ( relocalize_withmarkers(f,pose_f2g_out)) return true;
    if (relocalize_withkeypoints(f,pose_f2g_out)) return true;
    return false ;
}


//given lastframe, current and the map of points, search for the map points  seen in lastframe in current frame
//Returns query-(id position in the currentFrame). Train (id point in the TheMap->map_points)
std::vector<cv::DMatch> Slam::matchMapPtsOnPrevFrame(Frame & curframe, Frame &prev_frame, float minDescDist, float maxReprjDist)
{
    return {};


}

//跟踪
// lastKnownPose and _curKFRef are valid and
se3 Slam::track(Frame &curframe,se3 lastKnownPose)
{

    if (curframe.markers.size() == 0)
    {
        cout<<"tracking failed!\n";
        return se3();
    }
    //工具：检测非歧义函数
    //check that not ambiguos
    auto checkNoAmbiguos= [&](const vector<cv::DMatch> &map_matches){
        std::map<uint32_t,uint32_t> query,train;
        for(auto m:map_matches){
            if ( query.count(m.queryIdx)) {cerr<<"Ambigous query"<<endl;return false;}
            if(train.count(m.trainIdx)) {cerr<<"Ambigous train"<<endl; return false;}
            query.insert({m.queryIdx,m.trainIdx});
            train.insert({m.trainIdx,m.queryIdx});
        }
        return true;
    };


    ScopedTimerEvents tev("Slam::track");
    //first estimate current pose
    std::vector<cv::DMatch> map_matches;

    ///Step1: 初始位姿为上一帧位姿
    se3 estimatedPose=lastKnownPose;    //estimate the current pose using motion model


    //search for the points matched and obtain an initial  estimation if there are map points
    ///Point求解位姿
     if (TheMap->map_points.size()>0){
        curframe.pose_f2g=estimatedPose;
        map_matches=matchMapPtsOnPrevFrame(curframe,_prevFrame,_params.minDescDistance,_params.projDistThr);
        tev.add("matchMapPtsOnPrevFrame");
        //do an initial pose estimation. In the process, bad matches will be removed from the vector
        solvePnp( curframe,TheMap,map_matches,estimatedPose,_curKFRef);
        tev.add("poseEstimation");
        _debug_msg("matchMapPtsOnFrames = "<<map_matches.size(),10);
        float projDistance;

        if ( map_matches.size()>30) {//a good enough initial estimation is obtained
            projDistance=2.5;//for next refinement
            for(auto m:  map_matches ){//mark these as already being used and do not consider in next step
                TheMap->map_points[m.trainIdx].getExtra<uint32_t>()[0]= curframe.fseq_idx;
                TheMap->map_points[m.trainIdx].setVisible(curframe.fseq_idx);
            }
        }
        else{
            map_matches.clear();
            projDistance=_params.projDistThr;//not good previous step, do a broad search in the map
        }

        //get the neighbors and current, and match more points in the local map
        auto map_matches2 = TheMap->matchFrameToMapPoints ( TheMap->TheKpGraph.getNeighborsV( _curKFRef,true) , curframe,  estimatedPose ,_params.minDescDistance*2, projDistance,true);
        //add this info to the map and current frame
        _debug_msg("matchMapPtsOnFrames 2= "<<map_matches2.size(),10);
        map_matches.insert(map_matches.end(),map_matches2.begin(),map_matches2.end());
        _debug_msg("matchMapPtsOnFrames final= "<<map_matches.size(),10);
        tev.add("matchMapPtsOnFrames");
        filter_ambiguous_query(map_matches);
    }

    assert(checkNoAmbiguos(map_matches));

    ///Step2, Marker求解位姿
    solvePnp( curframe,TheMap,map_matches,estimatedPose,_curKFRef);

    ///BIT Add
    if (lastKnownPose == estimatedPose)
    {
        cout<<"tracking failed!\n";
        return se3();
    }
    tev.add("poseEstimation");

    ///Step3：检测位姿结果是否可靠
    //determine if the pose estimated is reliable
    //is aruco accurate
    bool isArucoTrackOk=false;
    if ( curframe.markers.size()>0){
        float baseLine=cv::norm(estimatedPose.getTvec(),TheMap->keyframes[_curKFRef].pose_f2g.getTvec());

        //count how many reliable marker are there
        for(size_t  i=0;i<  curframe.markers.size();i++){
            //is the marker in the map with valid pose
            auto mit=TheMap->map_markers.find( curframe.markers[i].id);
            if(mit==TheMap->map_markers.end())continue;//is in map?
            if (!mit->second.pose_g2m.isValid())continue;//has a valid pose?
            //is it observed with enough reliability? < 3
            if (curframe.markers_solutions[i].err_ratio < _params.aruco_minerrratio_valid) continue;
            ///BIT ADD
            if (curframe.markers_solutions[i].err_ratio >= _params.aruco_minerrratio_valid &&
                curframe.markers_solutions[i].theta < _params.max_theta )//curframe.markers_solutions[i].distR < 1.2 * curframe.markers_solutions[i].tau
            {
                isArucoTrackOk=true;
                cout<<"tracking success!\n";
                break;
            }
//            isArucoTrackOk=true;
            break;

        }
        ///BIT ADD
        /*if (baseLine > 10*Slam::getParams().minBaseLine)
        {
            cout<<"track baseline = "<<baseLine<<endl;
            cout<<"track failed!\n";
            isArucoTrackOk = false;
        }*/

    }

    _debug_msg_("A total of "<<map_matches.size()<<" good matches found and isArucoTrackOk="<<isArucoTrackOk);
    if (map_matches.size()<30 && !isArucoTrackOk){
        _debug_msg_("need relocatization");
        cout<<"Track failed!\n";
        return se3();
    }
    /// update mappoints knowing now the outliers. It olny affects to map points

    for(size_t i=0;i<map_matches.size();i++){
            TheMap->map_points[map_matches[i].trainIdx].setSeen();
            curframe.ids[ map_matches[i].queryIdx]=map_matches[i].trainIdx;
    }

    return estimatedPose;
}




void Slam::drawMatches( cv::Mat &image,float inv_ScaleFactor)const{


    int size=float(image.cols)/640.f;
    cv::Point2f psize(size,size);

    if (currentState==STATE_TRACKING){
        for(size_t i=0;i<_cFrame.ids.size();i++)
            if (_cFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                if (!TheMap->map_points.is( _cFrame.ids[i])) continue;
                cv::Scalar color(0,0,0);
                if (!TheMap->map_points[_cFrame.ids[i]].isStable) color[2]=255;
                cv::rectangle(image,inv_ScaleFactor*(_cFrame.kpts[i].pt-psize),inv_ScaleFactor*(_cFrame.kpts[i].pt+psize),color,size);
            }
    }
    else if(currentState==STATE_ZERO){
        for( auto p: _cFrame.kpts)
            cv::rectangle(image,inv_ScaleFactor*(p.pt-psize),inv_ScaleFactor*(p.pt+psize),cv::Scalar(255,0,0),size);
    }

    //now, draw markers found
    for(auto aruco_marker:_cFrame.markers){
        //apply distortion back to the points
        cv::Scalar color=cv::Scalar(0,244,0);
        if( TheMap->map_markers.count(aruco_marker.id)!=0){
            if( TheMap->map_markers.at(aruco_marker.id).pose_g2m.isValid())
                color=cv::Scalar(255,0,0);
            else
                color=cv::Scalar(0,0,255);
        }
        //scale
        for(auto &p:aruco_marker) p*=inv_ScaleFactor;
        //draw
        aruco_marker.draw(image,color);
    }
}



void Slam::globalOptimization(){
    _debug_exec( 10, saveToFile("preopt.ucs"););

    auto opt=GlobalOptimizer::create(_params.global_optimizer);
    GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=5);
    params.fixFirstFrame=true;
    params.nIters=10;
    _debug_msg_("start initial optimization="<<TheMap->globalReprojChi2());

    opt->optimize(TheMap,params );
    _debug_msg_("final optimization="<<TheMap->globalReprojChi2() <<" npoints="<< TheMap->map_points.size());


    TheMap->removeBadAssociations(opt->getBadAssociations(),2);
    _debug_msg_("final points ="<< TheMap->map_points.size());



    _debug_exec( 10, saveToFile("postopt.ucs"););

}






 void Slam::resetTracker(){
     frame_pose.clear();
     _curKFRef=-1;
     _curPose_f2g=se3();
     currentState=STATE_LOST;
     _cFrame.clear();
     _prevFrame.clear();
 }



 void Slam::clear(){
     TheMapManager.stop();

     isInitialized=false;
     currentState=STATE_ZERO;
     TheMap->clear();
 }


 void Slam::saveToFile(string filepath)throw(std::exception){

     waitForFinished();

     //open as input/output
     fstream file(filepath,ios_base::binary|ios_base::out );
     if(!file)throw std::runtime_error("could not open file for writing:"+filepath);

     //write signature
     io_write<uint64_t>(182312,file);

     TheMap->toStream(file);

      //set another breaking point here
     _params.toStream(file);
     toStream__kv_complex(frame_pose,file);
      file.write((char*)&_curPose_f2g,sizeof(_curPose_f2g));
     file.write((char*)&_curKFRef,sizeof(_curKFRef));
     file.write((char*)&isInitialized,sizeof(isInitialized));
     file.write((char*)&currentState,sizeof(currentState));
     file.write((char*)&currentMode,sizeof(currentMode));
     _cFrame.toStream(file);
     _prevFrame.toStream(file);
     fextractor.toStream(file);
     TheMapManager.toStream(file);
     file.flush();
 }

 void Slam::readFromFile(string filepath)throw(std::exception){
     ifstream file(filepath,ios::binary);
     if(!file)throw std::runtime_error("could not open file for reading:"+filepath);

     if ( io_read<uint64_t>(file)!=182312)  throw std::runtime_error("invalid file type:"+filepath);

     TheMap=std::make_shared<Map>();
     TheMap->fromStream(file);

     _params.fromStream(file);
     fromStream__kv_complexmap(frame_pose,file);
      file.read((char*)&_curPose_f2g,sizeof(_curPose_f2g));
     file.read((char*)&_curKFRef,sizeof(_curKFRef));
     file.read((char*)&isInitialized,sizeof(isInitialized));
     file.read((char*)&currentState,sizeof(currentState));
     file.read((char*)&currentMode,sizeof(currentMode));

     _cFrame.fromStream(file);
     _prevFrame.fromStream(file);

     fextractor.fromStream(file);
     TheMapManager.fromStream(file);

 //    setParams(TheImageParams,_params);


 }


 } //namespace
