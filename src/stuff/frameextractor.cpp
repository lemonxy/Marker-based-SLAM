#include "frameextractor.h"
#include <thread>
#include <opencv2/imgproc.hpp>
#include "stuff/timers.h"
#include "stuff/utils.h"
#include "optimization/ippe.h"
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
namespace ucoslam {


void FrameExtractor::toStream(std::ostream &str)const
{
    uint64_t sig=1923123;
    str.write((char*)&sig,sizeof(sig));    
    str.write((char*)&_counter,sizeof(_counter));
    str.write((char*)&_removeFromMarkers,sizeof(_removeFromMarkers));
    str.write((char*)&_detectMarkers,sizeof(_detectMarkers));
    str.write((char*)&_detectKeyPoints,sizeof(_detectKeyPoints));
     str.write((char*)&_markerSize,sizeof(_markerSize));
     _mdetector.toStream(str);
}

void FrameExtractor::fromStream(std::istream &str){
    uint64_t sig=1923123;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=1923123) throw std::runtime_error("FrameExtractor::fromStream invalid signature");
     str.read((char*)&_counter,sizeof(_counter));
    str.read((char*)&_removeFromMarkers,sizeof(_removeFromMarkers));
    str.read((char*)&_detectMarkers,sizeof(_detectMarkers));
    str.read((char*)&_detectKeyPoints,sizeof(_detectKeyPoints));
     str.read((char*)&_markerSize,sizeof(_markerSize));
     _mdetector.fromStream(str);

}

FrameExtractor::FrameExtractor(){}


void FrameExtractor::setParams(const Params &params){
     _mdetector.setParameters(params.aruco_DetectorParams);
    _markerSize=params.aruco_markerSize;
}

void FrameExtractor::process_rgbd(const cv::Mat &image,
        const cv::Mat &depthImage,const ImageParams &ip,
        Frame &frame, uint32_t frameseq_idx , uint32_t fidx )throw (std::exception){
    assert(ip.bl>0);
    assert(depthImage.type()==CV_16UC1);
         process(image,ip,frame,frameseq_idx,fidx);
        //  float fb=ip.bl*ip.fx();
         //now, add the extra info to the points
         for(size_t i=0;i<frame.kpts.size();i++){
             //convert depth
             if (depthImage.at<uint16_t>(frame.kpts[i].pt)!=0){
                 frame.depth[i]=depthImage.at<uint16_t>(frame.kpts[i].pt)*ip.rgb_depthscale;
                // frame.mvuRight[i]=frame.kpts[i].pt.x- fb/frame.depth[i];//find projection in IR camera
             }
         }
}
struct marker_analyzer{

    marker_analyzer(vector<cv::Point2f> &m){
        bax = m[1].x - m[0].x;
        bay = m[1].y - m[0].y;
        dax = m[2].x - m[0].x;
        day =  m[2].y - m[0].y;
        a=m[0];b=m[1];d=m[2];

    }

    bool isInto(const cv::Point2f &p)const{
        if ((p.x - a.x) * bax + (p.y - a.y) * bay < 0.0) return false;
        if ((p.x - b.x) * bax + (p.y - b.y) * bay > 0.0) return false;
        if ((p.x - a.x) * dax + (p.y - a.y) * day < 0.0) return false;
        if ((p.x - d.x) * dax + (p.y - d.y) * day > 0.0) return false;

        return true;
    }
    float bax, bay , dax  ,day;
    cv::Point2f a,b,d;

};
void FrameExtractor::process(const cv::Mat &image,const ImageParams &ip,
        Frame &frame,uint32_t frameseq_idx,uint32_t fidx)throw (std::exception){
    assert(image.size()==ip.CamSize);

    frame.clear();
    ScopedTimerEvents tem("FrameExtractor::process");
     if (image.channels()==3)
        cv::cvtColor(image,_imgrey,CV_BGR2GRAY);
    else _imgrey=image;

    std::thread aruco_thread( [&]{
        if (_detectMarkers){
            if (ip.aprilTag)
            {
                ///apriltag 3
                apriltag_detector *td = apriltag_detector_create();
                td->quad_decimate = 1.0;
                td->quad_sigma = 0.8;
                apriltag_family *tf = NULL;
                const char* famname = ip.apriltagDictionary.c_str();
                if (!strcmp(famname, "tag36h11")){
                    tf = tag36h11_create();
                } else if (!strcmp(famname, "tag25h9")) {
                    tf = tag25h9_create();
                } else if (!strcmp(famname, "tag16h5")) {
                    tf = tag16h5_create();
                } else if (!strcmp(famname, "tagCircle21h7")) {
                    tf = tagCircle21h7_create();
                } else if (!strcmp(famname, "tagCircle49h12")) {
                    tf = tagCircle49h12_create();
                } else if (!strcmp(famname, "tagStandard41h12")) {
                    tf = tagStandard41h12_create();
                } else if (!strcmp(famname, "tagStandard52h13")) {
                    tf = tagStandard52h13_create();
                } else if (!strcmp(famname, "tagCustom48h12")) {
                    tf = tagCustom48h12_create();
                } else {
                    printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
                    exit(-1);
                }
                apriltag_detector_add_family(td,tf);
                image_u8_t im = {.width = _imgrey.cols,
                        .height = _imgrey.rows,
                        .stride = _imgrey.cols,
                        .buf = _imgrey.data
                };
                zarray_t *detections = apriltag_detector_detect(td, &im);
                int nTags = zarray_size(detections);
                for (int i = 0; i < nTags; ++i) {
                    apriltag_detection_t *det;
                    zarray_get(detections, i, &det);
                    vector<cv::Point2f> contour;
                    bool bErase = false;
                    for (int j = 0; j < 4; ++j)
                    {
                        cv::Point2f point{float(det->p[j][0]),float(det->p[j][1])};
                        if (point.x < 30 || point.x > 1890 || point.y < 30 || point.y > 1050)
                        {
                            bErase = true;
                            break;
                        }
                        contour.emplace_back(point);
                    }
                    if (bErase)
                        continue;
                    cv::Point2f p1 = contour[0];
                    cv::Point2f p2 = contour[1];
                    cv::Point2f p3 = contour[2];
                    double dist1 = cv::norm(p1 - p2);
                    double dist2 = cv::norm(p3 - p2);
                    double dist = min(dist1, dist2);
                    if (dist < 20)
                        continue;
                    aruco::Marker marker(contour,det->id);
                    frame.markers.emplace_back(marker);
                }
                zarray_destroy(detections);
                apriltag_detector_destroy(td);
                tag36h11_destroy(tf);
            }
            else
                _mdetector.detect(_imgrey,frame.markers);
            //remove elements from the black list
//#warning "remove this"
//            std::vector<int> black_list
//                    ={0,10,300,330,350};
//            frame.markers.erase( std::remove_if(frame.markers.begin(),frame.markers.end(),[black_list](const aruco::Marker &m){ return std::find(black_list.begin(),black_list.end(), m.id)!=black_list.end();}),frame.markers.end());
//            //now, apply IPPE to detect the locations
//            cout<<"**"<<"Marker size "<<frame.markers.size()<<"**\n";
            cout<<"marker id: ";
            for(const auto&m:frame.markers)
            {
                cout<<m.id<<", ";
//                for (size_t i = 0; i < m.size(); ++i)
//                {
//                    cout<<"("<<m[i].x<<", "<<m[i].y<<")"<<endl;
//                }
                auto sols=IPPE::solvePnP_(_markerSize,m,ip.CameraMatrix,ip.Distorsion);
//                cout<<"K = "<<ip.CameraMatrix<<endl;
//                cout<<"distCoef = "<<ip.Distorsion<<endl;
//                cout<<"err1: "<<sols[0].second<<", err2: "<<sols[1].second;
//                cout<<", ratio = "<<sols[0].second/sols[1].second<<endl;
                cv::Mat R1 = sols[0].first.rowRange(0,3).colRange(0,3);
                cv::Mat R2 = sols[1].first.rowRange(0,3).colRange(0,3);
                cv::Mat t1 = sols[0].first.rowRange(0,3).col(3);
                cv::Mat t2 = sols[1].first.rowRange(0,3).col(3);
                double  dist1 = cv::norm(t1);
                double  dist2 = cv::norm(t2);
                double x1 = acos(t1.at<float>(2)/dist1);
                double x2 = acos(t2.at<float>(2)/dist2);
                double theta1 = x1*180/3.1415926;
                double theta2 = x2*180/3.1415926;
                cv::Mat deltaR = R1*R2.t();
                cv::Mat phi;
                cv::Rodrigues(deltaR,phi);
//                cout<<"sol1: \n"<<sols[0].first<<endl;
//                cout<<"sol2: \n"<<sols[1].first<<endl;
                MarkerPosesIPPE mp;
                mp.errs[0]=sols[0].second;
                mp.errs[1]=sols[1].second;
                mp.sols[0]=sols[0].first;
                mp.sols[1]=sols[1].first;
                mp.err_ratio=sols[1].second/sols[0].second;
                mp.distR = cv::norm(phi);
                mp.theta = (theta1+theta2)/2;
                ///test parameter
                mp.tau = 0.0007*mp.theta*mp.theta-0.0483*mp.theta+1.3011;
                cout<<"deltaR = "<<mp.distR<<", theta="<<mp.theta<<", ratio="<<mp.err_ratio<<endl;
                frame.markers_solutions.push_back(mp);
//                cout<<mp.sols[0]<<endl<<mp.sols[1]<<endl;
            }
            cout<<endl;
            for(auto&m:frame.markers)
                m.ssize=_markerSize;

        }
    }
    );
     aruco_thread.join();

    if (debug::Debug::getLevel()>=10)
        image.copyTo(frame.image);

    //create a reduced image version for fern database
    //use the scale factor making the image width of 128 pix
    float sc=128./float(image.cols);

    cv::Mat aux;
    cv::resize(image,aux,cv::Size( float(image.cols)*sc,float(image.rows)*sc));
    if (aux.type()!=CV_8UC1)
        cv::cvtColor(aux,frame.smallImage,CV_BGR2GRAY);
    else frame.smallImage=aux;
    //remove keypoints into markers??

    tem.add("Keypoint/Frames detection");

    if (_removeFromMarkers  ){
        vector<marker_analyzer> manalyzers;
        for(auto m:frame.markers)
            manalyzers.push_back(marker_analyzer(m));
        vector<cv::KeyPoint> kp2;
        kp2.reserve(frame.kpts.size());
        cv::Mat desc2(frame.desc.size(),frame.desc.type());
        for(size_t i=0;i<frame.kpts.size();i++){
            bool isIntoAny=false;
            for(uint a=0;a<manalyzers.size() && !isIntoAny;a++)
                if (manalyzers[a].isInto(frame.kpts[i].pt)) isIntoAny=true;
            if (!isIntoAny){
                frame.desc.rowRange(i,i+1).copyTo(desc2.rowRange(kp2.size(),kp2.size()+1));
                kp2.push_back(frame.kpts[i]);
            }
        }
        desc2.resize(kp2.size() ,desc2.cols);
        frame.desc=desc2;
        frame.kpts=kp2;
    }


    tem.add("remove from markers");

    //remove distortion

    if (frame.kpts.size()>0){
        vector<cv::Point2f> pin;pin.reserve(frame.kpts.size());
        for(auto p:frame.kpts) pin.push_back(p.pt);
        undistortPoints(pin,ip );

        frame.und_kpts=frame.kpts;
        for ( size_t i=0; i<frame.kpts.size(); i++ )
            frame.und_kpts[i].pt=pin[i];
    }

    tem.add("undistort");
    //remove distortion of the marker points if any
    frame.und_markers=frame.markers;
    for(auto &m:frame.und_markers)
        undistortPoints(m,ip);


    frame.nonMaxima.resize(frame.und_kpts.size());
    for(auto &v:frame.nonMaxima) v=false;

    frame.ids.resize(frame.und_kpts.size());
    //set the keypoint ids vector to invalid
    uint32_t mval=std::numeric_limits<uint32_t>::max();
    for(auto &ids:frame.ids) ids=mval;
    //create the grid for fast access


    //set the frame id
  // assert (fidx!=std::numeric_limits<uint32_t>::max()) ;
    frame.idx=fidx;
    frame.fseq_idx=frameseq_idx;
    frame.imageParams=ip;


    frame.depth.resize(frame.und_kpts.size());
    for(size_t i=0;i<frame.depth.size();i++) frame.depth[i]=0;
    frame.create_kdtree();//last thing
 }
}
