#include "solvepnp.h"
#include "stuff/utils.h"
namespace ucoslam{

void g2o_poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d, se3 &pose_io,
                        std::vector<bool> &vBadMatches, const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor, std::vector<std::pair<ucoslam::Marker,aruco::Marker> > *marker_poses=0 );



void ucoslam_poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d, se3 &pose_io, std::vector<bool> &vBadMatches,
                    const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor,  std::vector<std::pair<ucoslam::Marker,aruco::Marker> > *marker_poses=0);




void poseEstimation(const std::vector<cv::Point3f> &p3d, const std::vector<cv::KeyPoint> &p2d, se3 &pose_io, std::vector<bool> &vBadMatches,
                    const cv::Mat &CameraParams, const std::vector<float> &invScaleFactor,const vector<float> &pointConfidence)
{
       g2o_poseEstimation(p3d,p2d,pose_io,vBadMatches,CameraParams,invScaleFactor);
     //  ucoslam_poseEstimation(p3d,p2d,pose_io,vBadMatches,CameraParams,invScaleFactor);


}

bool solvePnPRansac( const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &pose_io)
{
    //get the 3d and 2d matches
    vector<cv::Point2d> p2d; p2d.reserve(matches_io.size());
    vector<cv::Point3d> p3d; p3d.reserve(matches_io.size());
    for(cv::DMatch m: matches_io){
        if( !map->map_points.is(m.trainIdx)) continue;
          auto &mapPoint=map->map_points[m.trainIdx];
        if (mapPoint.isBad) continue;
        p2d .push_back(frame.und_kpts[m.queryIdx].pt);
        p3d.push_back( mapPoint.getCoordinates());
    }
    cv::Mat rv,tv;
    vector<int> inliers;
    bool res=cv::solvePnPRansac(p3d,p2d, frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,false,100,2.5,0.99,inliers);

    std::vector<cv::DMatch> new_matches_io;new_matches_io.reserve(inliers.size());
    for(auto i:inliers)
        new_matches_io.push_back( matches_io[i]);
    matches_io=new_matches_io;
    pose_io.setRT(rv,tv);
    return res;
}

    //求解位姿
    //estimatedPose: 初始值（跟踪时，为上一帧的位姿）
    void solvePnp( const Frame &frame, std::shared_ptr<Map> TheMap,
            std::vector<cv::DMatch> &map_matches, se3 &estimatedPose ,int64_t currentKeyFrame)
    {
        vector<cv::Point3f> p3d(map_matches.size());
        vector<cv::KeyPoint> p2d(map_matches.size());
        vector<float> pointConfidence(map_matches.size());
        
        ///Step1: 使用ORB特征点
        for(size_t i=0;i<map_matches.size();i++){
            p2d[i]=frame.und_kpts[ map_matches[i].queryIdx] ;
            auto &MP=TheMap->map_points[ map_matches[i].trainIdx];
            p3d[i]= MP.getCoordinates() ;
            pointConfidence[i]=MP.getConfidence();
        }
        std::vector<bool> vBadMatches;
        //create the scale factors

        std::vector<float> invScaleFactor=frame.scaleFactors;
        for(auto &v:invScaleFactor) v=1./v;

        //now, markers
        ///Step2: 使用Markers
        std::vector<std::pair<ucoslam::Marker,aruco::Marker> > marker_poses;
        if ( frame.und_markers.size()!=0 && currentKeyFrame!=-1)
        {
            ///Step2.1: 获取相邻关键帧组
            auto neigh=TheMap->getNeighborKeyFrames(currentKeyFrame,true);
            ///Step2.2: 获取相邻关键帧组的有效地图Markers
            std::set<uint32_t> markerInNeighbors;
            for(auto n:neigh)
            {
                for(const auto &m:TheMap->keyframes[n].und_markers){
                    if (TheMap->map_markers[m.id].pose_g2m.isValid())
                        markerInNeighbors.insert(m.id);
                }
            }
            /// BIT add
            if (markerInNeighbors.size() == 0) return;

            cout<<"local marker: ";
            for (auto m : markerInNeighbors)
                cout<<m<<", ";
            cout<<endl;

            ///Step2.3： 获取有效Markers的位姿
            for(auto m:frame.und_markers){
                if ( markerInNeighbors.count(m.id)==0)continue;
                marker_poses.push_back({TheMap->map_markers[m.id],m});
            }
        }
    //   ucoslam_poseEstimation( p3d,  p2d, estimatedPose, vBadMatches, frame.imageParams.CameraMatrix,invScaleFactor);
    ///Step3：计算当前帧的位姿
        g2o_poseEstimation(p3d,p2d,estimatedPose,vBadMatches,frame.imageParams.CameraMatrix,invScaleFactor,&marker_poses);
        for(size_t i=0;i<vBadMatches.size();i++)
            if ( vBadMatches[i])
                map_matches[i].queryIdx=map_matches[i].trainIdx=-1;
        remove_unused_matches(map_matches);
    }
}
