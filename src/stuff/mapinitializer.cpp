/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://wwmap->gnu.org/licenses/>.
*/

#include "mapinitializer.h"

#include "optimization/ippe.h"
#include "stuff/utils.h"
#include "stuff/timers.h"
#include "stuff/debug.h"
#include "stuff/hash.h"
#include "optimization/globaloptimizer_simple.h"
#include <cmath>
#include <opencv2/calib3d.hpp>

//#include "Thirdparty/DBoW2/DUtils/Random.h"

//#include "Optimizer.h"
//#include "ORBmatcher.h"

#include<thread>
using namespace std;
namespace ucoslam
{
    ///初始化
bool MapInitializer::process(const Frame &frame, std::shared_ptr<Map>  map){

    for (auto m : frame.und_markers)
    {
        vector<cv::Point2f> p2d_un;
        p2d_un.insert(p2d_un.end(),m.begin(),m.end());
//        for (size_t i=0; i<p2d_un.size(); ++i)
//            cout<<p2d_un[i].x<<", "<<p2d_un[i].y<<endl;
    }
    if(frame.kpts.size()<10 && _params.mode==KEYPOINTS) return false;
    ///单帧初始化
    cout<<"allow Aruco One Frame "<<_params.allowArucoOneFrame<<endl;
    if (_params.allowArucoOneFrame)//
        if (aruco_one_frame_initialize(frame,map)) {
            _debug_msg_("initialization from single image using aruco");
            return true;
        }


    //first time
    if (!_refFrame.isValid()){
        setReferenceFrame(frame);
        return false;
    }
    else///两帧初始化
    {
        bool res= initialize(frame, map);
        //check if there any marker

        if(!res){
            //check if any common marker between the frames
            int nCommonMarkers=0;
            for(auto m:frame.und_markers)
                if ( _refFrame.getMarkerIndex(m.id)!=-1 )nCommonMarkers++;
            if (matches.size()<50 && nCommonMarkers==0){//not enough common references, reset reference frame
                setReferenceFrame(frame);
                _debug_msg_("Restart initialization");
            }
        }
        return res;
    }
}


void MapInitializer::setReferenceFrame(const Frame &frame){
    frame.copyTo(_refFrame);
    if (_params.mode!=ARUCO && frame.ids.size()!=0){//no keypoints
        _refFrameIndex=std::make_shared<xflann::Index>();
        _refFrameIndex->build(frame.desc,xflann::HKMeansParams(32,0));
        kpt_matcher.setReferenceFrame(frame);
    }
}

//checks if the frame passed can be used to initialize with the second one
bool MapInitializer::initialize(const Frame &frame2, std::shared_ptr<Map> map)throw(std::exception){
    matches.clear();
    if (!_refFrame.isValid())throw std::runtime_error("Initializer::initialize invalid reference frame");

    //find matches if using keypoints
    if (_params.mode!=ARUCO && _refFrameIndex && frame2.ids.size()>0){
        if(1) {
            cv::Mat indices,distances;

            _refFrameIndex->search(frame2.desc,2,indices,distances ,xflann::KnnSearchParams(20,true));

            bool useMinDescDist=_params.minDescDistance!=-1;
            //force change to 32F
            if (distances.type()==CV_32S)
                distances.convertTo(distances,CV_32F);


            for(int i = 0; i < distances.rows; i++) {
                if (  useMinDescDist && distances.at<float>(i,0)> _params.minDescDistance) continue;
                if(distances.at<float>(i,0) < _params.nn_match_ratio * float(distances.at<float>(i,1))) {
                    if (std::abs( _refFrame.und_kpts[ indices.at<int32_t>(i,0) ].octave- frame2.und_kpts[i].octave)<2 ){
                        cv::DMatch match;match.distance=distances.at<float>(i,0);
                        match.queryIdx=indices.at<int32_t>(i,0);
                        match.trainIdx=i;
                        matches.push_back(match);
                    }
                }
            }

            filter_ambiguous_query(matches);
        }

        else{
            ScopeTimer timer("matches");
            kpt_matcher.trackPoints(frame2,Slam::getParams().minDescDistance*1.25,2.5);
            matches=kpt_matcher.getMatches();
            //swap
            for(auto &m:matches)
                std::swap(m.trainIdx,m.queryIdx);
        }
        cout<<"|@#|@#|@#|@#@| A total of "<<matches.size()<<" matches "<<endl;

        //check the average pixel  distance between matches
        float avrPixDist=0;
        for(auto m:matches)
            avrPixDist+=cv::norm(frame2.und_kpts[ m.trainIdx].pt -_refFrame.und_kpts [m.queryIdx].pt );

        avrPixDist/=float(matches.size());
        avrPixDist/=float(frame2.imageParams.CamSize.width);
        _debug_msg_("matches ="<<matches.size()<<" avrPixDist="<<avrPixDist);
        //    if ( avrPixDist>=0.05)
    }
    //两帧初始化
    bool res= initialize(_refFrame,frame2, matches,map,false);
    if (res)
        cout<<"两帧初始化成功!"<<endl;
    return res;
}




////两帧初始化
bool MapInitializer::initialize(const Frame &frame1, const Frame &frame2,    vector<cv::DMatch> &matches, std::shared_ptr<Map> map, bool force)throw(std::exception){
    cv::Mat rt;
    ScopedTimerEvents timer("Initialize::initialize");


    auto rt_mode=computeRt(frame1,  frame2, matches,_params.minDistance);
    rt=rt_mode.first;
    timer.add("computed rt");

    if (rt_mode.first.empty())return false;
    //set proper ids

    //add frames
    Frame & kfframe1=map->addKeyFrame(frame1); //[frame2.idx]=frame2;
    kfframe1.pose_f2g=cv::Mat::eye(4,4,CV_32F);
    Frame & kfframe2= map->addKeyFrame(frame2); //[frame2.idx]=frame2;
    kfframe2.pose_f2g=rt;



    //now, for each marker set the the frame info in which it has been seen
    for(auto &marker:kfframe1.und_markers)
    {
        map->addMarker(marker);
        map->addMarkerObservation(marker.id,kfframe1.idx);
    }
    for(auto &marker:kfframe2.und_markers)
    {
        map->addMarker(marker);
        map->addMarkerObservation(marker.id,kfframe2.idx);
    }


    if(rt_mode.second==KEYPOINTS){//started with matches only
        _debug_msg_("Initialized from keypoints");
        //need to calculate the markers locations
    }
    else{
        _debug_msg_("Initialized from markers");
    }

    //set markers locations given the established location in rt
    //add markers to the map
    for(auto m:_marker_se3)
        map->map_markers[ m.first ].pose_g2m= m.second;



    if (matches.size()>10){
        vector<cv::Point3f> p3d;
        vector<cv::KeyPoint> pf1,pf2;
        for(auto m:matches){
            pf1.push_back( kfframe1.und_kpts[ m.queryIdx] );
            pf2.push_back( kfframe2.und_kpts[ m.trainIdx] );

        }
         vector<bool> vGood;
         int nvalid=triangulate_(rt,pf1,pf2,frame1.imageParams.CameraMatrix,p3d,frame1.scaleFactors, vGood);


        _debug_msg_("initialization matches="<<nvalid);
        if (nvalid<30){
            map->clear();
            _debug_msg_("too few  matches");
            return false;
        }
        timer.add("triangulate");
        //remove these behind the camera

        //set the ids
        for(size_t i=0;i<p3d.size();i++){
            if (vGood[i]){
                auto &mp= map->addNewPoint(kfframe2.fseq_idx);
                mp.isStable=false;
                mp.kfSinceAddition=1;
                mp.pose= p3d[i];
                 map->addMapPointObservation(mp.id,kfframe1.idx,matches[i].queryIdx);
                 map->addMapPointObservation(mp.id,kfframe2.idx,matches[i].trainIdx);
              }
        }
     }
    assert(map->checkConsistency());

    return true;
}


///两帧初始化
std::pair<cv::Mat,MapInitializer::MODE> MapInitializer::computeRt(const Frame &frame1, const Frame &frame2,
                        vector<cv::DMatch> &matches, float minDistance)  throw(std::exception){

    if (frame1.und_kpts.size()==0 && _params.mode==KEYPOINTS)     return {cv::Mat(),NONE};
    if (frame2.und_kpts.size()==0 && _params.mode==KEYPOINTS)     return {cv::Mat(),NONE};
    if (frame1.und_markers.size()==0 && _params.mode==ARUCO)     return {cv::Mat(),NONE};
    if (frame2.und_markers.size()==0 && _params.mode==ARUCO)     return {cv::Mat(),NONE};
    if (frame1.und_kpts.size()==0 && frame1.und_markers.size()==0  )     return {cv::Mat(),NONE};
    if (frame2.und_kpts.size()==0 && frame2.und_markers.size()==0  )     return {cv::Mat(),NONE};


    if (!frame1.imageParams.isValid())throw std::runtime_error("Need to call setParams to set the camera params first");
    if (_params.markerSize<=0)throw std::runtime_error("Invalid marker size");
    //try first using aruco
    if (_params.mode==BOTH || _params.mode==ARUCO){
        cout<<"ref frame id "<<frame1.fseq_idx<<", frame id "<<frame2.fseq_idx<<endl;
        auto rt=ARUCO_initialize(frame1.und_markers,frame2.und_markers,frame1.imageParams.undistorted(),
                            _params.markerSize,0.02,
                            _params.max_makr_rep_err,_params.minDistance,_marker_se3);
        if (!rt.empty())
                return {rt,ARUCO};
    }
    if (_params.mode==BOTH || _params.mode==KEYPOINTS){

        cv::Mat R33,t;
        vector<cv::Point3f> p3d;
        vector<bool> valid;
        if( getRtFromMatches(frame1.imageParams.CameraMatrix,frame1.und_kpts,frame2.und_kpts,matches,R33,t,p3d,valid)){
            //convert into a single matrix of 32f
            cv::Mat R44=cv::Mat::eye(4,4,CV_32F);
            R33.copyTo( R44.colRange(0,3).rowRange(0,3));
            t*=minDistance;
            t.copyTo(R44.rowRange(0,3).colRange(3,4));
            //    cout<<R44<<" norm="<<cv::norm(t)<<endl;//cin.ignore();
            return {R44,KEYPOINTS};
        }
    }
    return {cv::Mat(),NONE};

}

MapInitializer::MapInitializer( float sigma, int iterations)
{

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}





bool MapInitializer::getRtFromMatches(const cv::Mat &CamMatrix,const std::vector<cv::KeyPoint> & ReferenceFrame,
                             const std::vector<cv::KeyPoint> &cur_frame,   vector<cv::DMatch> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    if (vMatches12.size()<10)return false;//need a minimum number of matches
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2

   // file.write()

    mK = CamMatrix.clone();
    mvKeys1 = ReferenceFrame;
    mvKeys2 = cur_frame;



    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    for(auto m:vMatches12)
        mvMatches12.push_back(make_pair(m.queryIdx,m.trainIdx));


    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
      vAllIndices.push_back(i);


    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

//    DUtils::Random::SeedRandOnce(0);
    srand(0);
    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = rand()% vAvailableIndices.size(); //DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&MapInitializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&MapInitializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    float RH = SH/(SH+SF);


    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    bool result=false;
    if(RH>0.40)
        result= ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    else //if(pF_HF>0.6)
        result=ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

     return result;
}


void MapInitializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


void MapInitializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


cv::Mat MapInitializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat MapInitializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}

float MapInitializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float MapInitializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool MapInitializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool MapInitializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

void MapInitializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void MapInitializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}


int MapInitializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void MapInitializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{

    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

///单帧初始化
bool MapInitializer::aruco_one_frame_initialize(const Frame &frame, std::shared_ptr<Map> map){
//see if with the available markers, it is possible to do initialization
    //detect camera-poses and see if they are robust enough

    ///统计误差比小于0.4的Marker
    int nGoodMarkers=0;
    for(size_t m=0;m<frame.und_markers.size();m++){
        if ( frame.markers_solutions[m].err_ratio > _params.aruco_minerrratio_valid &&
//             frame.markers_solutions[m].distR > 1.3*frame.markers_solutions[m].tau &&
             frame.markers_solutions[m].theta < _params.max_theta)
            nGoodMarkers++;
    }
    cout<<"minerrRatio "<<_params.aruco_minerrratio_valid<<endl;
    if (nGoodMarkers==0)
    {
        cout<<"单帧初始化失败!"<<endl;
        return false;
    }

    auto &MapKeyFrame=map->addKeyFrame(frame);
    MapKeyFrame.pose_f2g=se3(0,0,0,0,0,0);

    for(size_t m=0;m<frame.und_markers.size();m++){
        auto &MapMarker=map->addMarker( frame.und_markers[m]);
        map->addMarkerObservation(MapMarker.id,MapKeyFrame.idx);
        if ( frame.markers_solutions[m].err_ratio > _params.aruco_minerrratio_valid &&
//                frame.markers_solutions[m].distR > 1.2*frame.markers_solutions[m].tau &&
                frame.markers_solutions[m].theta < _params.max_theta)
        {
            MapMarker.pose_g2m=frame.markers_solutions[m].sols[0];
            cout<<"create new marker, id = "<<MapMarker.id<<", distR="<<frame.markers_solutions[m].distR
                <<", theta="<<frame.markers_solutions[m].theta<<endl;
        }


        }
    cout<<"单帧初始化成功!"<<endl;
    return true;

}
} //namespace ORB_SLAM
