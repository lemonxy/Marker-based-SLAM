

///---------------------------------------------------------------------------------------------
#ifndef _SLAM_SGL_OpenCV_VIewer_H
#define _SLAM_SGL_OpenCV_VIewer_H
#include "sgl.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include "../mapviewer.h"
#include "../slam.h"
namespace ucoslam{
//Class using an opencv window to render and manipulate a sgl scene



class slam_sglcvviewer:public MapViewer {
    sgl::Scene _Scene;
    std::string _wname;
    float _f;
    int _w,_h;
    cv::Mat _imshow;
    int mode=1;//mode 0:big 3d, 1 big image
    int PtSize=2;
    float _subw_nsize=0.3;
    bool showingHelp=false;
    cv::Mat _cameraImage;
    bool showNumbers=false;
    bool showKeyFrames=true;
    bool showKeyPoints=true;
    int waitKeyTime=0;
    bool showPointNormals=false;
    struct marker_pi{
        se3 pose;
        vector<sgl::Point3> points;
    };
    std::map<uint32_t,marker_pi> marker_points;

    vector<pair<cv::Point3f,bool> > mapPoints;
    vector< cv::Point3f  > mapPointsNormals;
    std::map<uint32_t,Marker> map_markers;

    std::map<uint32_t,Se3Transform> _FSetPoses;
    int64_t curKF=-1;
    CovisGraph CVGraph;
    bool canLeave=false;
    int showCovisGraph=0;
    cv::Mat _resizedInImage;
    cv::Mat _camPose;
    bool followCamera=false;
    int snapShotIndex=0;
    string userMessage;
    std::mutex drawImageMutex;
public:


    slam_sglcvviewer( ){
        setParams(1,1280,720,"sgl");

    }
    void set(const string &param,const string & val){
        if(param=="showNumbers")
            showNumbers=std::stoi(val);
        else if (param=="showCovisGraph")
            showCovisGraph=std::stoi(val);
        else if(param=="canLeave")
            canLeave=std::stoi(val);
        else if(param=="mode")
            mode=std::stoi(val);
        else if(param=="modelMatrix"){
               stringstream sstr;sstr<<val;
               sgl::Matrix44 m;
               for(int i=0;i<4;i++)
                   for(int j=0;j<4;j++)
                       sstr>>m(i,j);
               _Scene.setModelMatrix(m);
        }
        else if(param=="viewMatrix"){
            stringstream sstr;sstr<<val;
            sgl::Matrix44 m;
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    sstr>>m(i,j);
            _Scene.setViewMatrix(m);

        }
    }



    void setParams(float f,int width,int height,std::string wname){
        _imshow.create(height,width,CV_8UC3);
        _f=f;
        _w=width;_h=height;
        _wname=wname;
        cv::namedWindow(_wname,cv::WINDOW_AUTOSIZE);
       cv::resizeWindow(_wname,width,height);
        cv::setMouseCallback(_wname, &slam_sglcvviewer::mouseCallBackFunc , this);

        sgl::Matrix44 cam;
        cam.translate({0,4,0});
        cam.rotateX(3.1415/2.);
        _Scene.setViewMatrix(cam);

    }

      cv::Mat getImage(){return _imshow;}//returns the image generated after calling show

      void getImage(cv::Mat &image) {
          std::unique_lock<std::mutex> lock(drawImageMutex);
          _imshow.copyTo(image);
      }

      int show(   Slam *slam,const cv::Mat &cameraImage,string userMessage="" ){
          cv::Mat cpose;
       if (slam->getCurrentPose_f2g().isValid()) cpose=slam->getCurrentPose_f2g().convert().inv();
       else cpose=cv::Mat();
       curKF=slam->_curKFRef;
       return show(slam->TheMap,cameraImage,cpose,userMessage);
   }

    int show(   std::shared_ptr<Map> map,const cv::Mat &cameraImage=cv::Mat(),const cv::Mat &cameraPose_f2g=cv::Mat(),string additional_msg=""){
        assert(cameraPose_f2g.total()==16 || cameraPose_f2g.total()==0);
        if (!cameraPose_f2g.empty())
            _camPose=cameraPose_f2g.inv();
        else _camPose=cameraPose_f2g;
        userMessage=additional_msg;
        _cameraImage=cameraImage;
        _FSetPoses.clear();
        mapPoints.clear();
        mapPointsNormals.clear();

         map->lock(__FUNCTION__,__FILE__,__LINE__);
        for(auto &mp:map->map_points){
            mapPoints.push_back({mp.getCoordinates(),mp.isStable});
            mapPointsNormals.push_back( mp.getNormal());
        }
         map_markers=map->map_markers;
        CVGraph=map->TheKpGraph;
        map->unlock(__FUNCTION__,__FILE__,__LINE__);

        for(auto fs:map->keyframes) _FSetPoses.insert({fs.idx,fs.pose_f2g.inv()});

        //set the marker points
        for(auto m:map_markers){
            bool redopoints=false;
            if(marker_points.count(m.first)==0) redopoints=true;
            else if(!(marker_points[m.first].pose== m.second.pose_g2m)) redopoints=true;
            if (redopoints ){
                marker_points[m.first].points=getMarkerIdPcd(m.second,0.5);
                marker_points[m.first].pose=m.second.pose_g2m;
            }
        }


        //first creation of the image

        createImage();


        int key,leaveNow=false;
        do{
            cv::imshow(_wname,_imshow);
            if (canLeave)waitKeyTime=2;
            else waitKeyTime=0;
            key=cv::waitKey(waitKeyTime);
//            if (k!=255) cout<<"wkh="<<k<<endl;
            bool update=false,create=false;
            //change mode
            if (key=='m'){
                mode=mode==0?1:0;
                create=true;
            }
            else if(key=='h'){
                showingHelp=!showingHelp;
                update=true;
            }
            else if(key=='n'){
                showNumbers=!showNumbers;
                update=true;
            }
            else if(key=='k'){
                showKeyFrames=!showKeyFrames;
                update=true;
            }
            else if(key=='p'){
                showKeyPoints=!showKeyPoints;
                update=true;
            }
            else if (key=='w'){
                string name="ucoslam-"+std::to_string(snapShotIndex++)+".png";
                cv::imwrite(name,_imshow);
                std::cerr<<"Image saved to "<<name<<endl;

            }
            else if (key=='x'){
                showPointNormals=!showPointNormals;
                update=true;

            }
            else if (key=='s' ) canLeave=!canLeave;
            else if( key=='g') {
                showCovisGraph++;
                if (showCovisGraph>=3)
                    showCovisGraph=0;
                update=true;

            }
            else if (key=='c'){
                followCamera=!followCamera;
                if(!followCamera){
                    sgl::Matrix44 cam;
                    cam.translate({0,4,0});
                    cam.rotateX(3.1415/2.);
                    _Scene.setViewMatrix(cam);
                }
                else _Scene.setViewMatrix(sgl::Matrix44());
                update=true;

            }
            //shift and ctrl keys
            else if (key==227 || key==225) {leaveNow=false;}
            else  if ( bExitOnUnUnsedKey  && key!=255 ) leaveNow=true;

            if (create) createImage();
            else if (update) updateImage();

        } while( (!canLeave && !leaveNow) && key!=27);
        return key;
    }

    void setSubWindowSize(float normsize=0.3){
        _subw_nsize=normsize;
    }

protected:
    void  createImage(  ) ;
    void updateImage(  );
    void drawScene();
    void drawText();

private:
    void  blending(const cv::Mat &a,float f,cv::Mat &io);


    vector<sgl::Point3> getMarkerIdPcd(ucoslam::Marker &minfo , float perct);

    struct mouseInfo{
        sgl::Point2 pos;
        bool isTranslating=false,isZooming=false,isRotating=false;
    }mi;


    static   void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata){
           slam_sglcvviewer *Sv=(slam_sglcvviewer*)userdata;
           bool redraw=false;
           if  ( event == cv::EVENT_LBUTTONDOWN ){
               Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
               if ( flags&cv::EVENT_FLAG_CTRLKEY)
                   Sv->mi.isZooming=true;
               else if ( flags&cv::EVENT_FLAG_SHIFTKEY) Sv->mi.isTranslating=true;
               else Sv->mi.isRotating=true;
           }
           else if  ( event == cv::EVENT_MBUTTONDOWN ) Sv->mi.isTranslating=true;
           else if ( event == cv::EVENT_LBUTTONUP ) {              Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
           }
           else if ( event == cv::EVENT_MBUTTONUP ) Sv->mi.isTranslating=false;
           else if ( event == cv::EVENT_MOUSEMOVE )
           {
               sgl::Point2  dif(Sv->    mi.pos.x-x,Sv->   mi.pos.y-y);
               sgl::Matrix44 tm;//=Sv->_Scene.getTransformMatrix();

               if (Sv->mi.isRotating){
                   tm.rotateX(-float(dif.y)/100);
                   tm.rotateZ(-float(dif.x)/100);
               }
               else if (Sv->mi.isZooming){
                   auto vp=Sv->_Scene.getViewMatrix();
                   vp.translate({0,0, float(-dif.y*0.01)});
                   Sv->_Scene.setViewMatrix(vp);
                   redraw=true;
               }
               else if (Sv->mi.isTranslating){
                   auto vp=Sv->_Scene.getViewMatrix();
                   vp.translate(sgl::Point3(float(-dif.x)/100, float(-dif.y)/100,0.f));
                   Sv->_Scene.setViewMatrix(vp);
                   redraw=true;
               }
               if (Sv->mi.isRotating||Sv->mi.isZooming ||Sv->mi.isTranslating)  {
                   sgl::Matrix44 res= tm*Sv->_Scene.getModelMatrix() ;
                   Sv->_Scene.setModelMatrix(res);
                   redraw=true;
               }
           }
           Sv->mi.pos=sgl::Point2(x,y);
           if (redraw)     {
               Sv->updateImage();
               cv::imshow(Sv->_wname,Sv->_imshow);
           }

       }

   };



void putText(cv::Mat &im,string text,cv::Point p ){
    float fact=float(im.cols)/float(1280);

    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(125,255,255),1*fact);

}
void slam_sglcvviewer::drawText(){

    int col=20;
if (!userMessage.empty()) {
    putText(_imshow,userMessage,cvPoint(30,col));
     col+=20;
}
    //print help commands
    if(!showingHelp){
        putText(_imshow, "'h' showhelp", cvPoint(30,col));
    }
    else{
        vector<string> messages={ "'h' hide help",
                                  "'s' start/stop video",
                                  "'m' change view mode",
                                  "'RightButtonMouse: Rotate' ",
                                  "'RightButtonMouse+CRTL: Zoom' ",
                                  "'RightButtonMouse+SHFT: Translate' ",
                                  "'n' show/hide marker numbers",
                                  "'x' show/hide point normals ",
                                  "'c' camera mode on/off",
                                  "'g' covis graph change mode",
                                  "'w' take a snapshot",
                                  "'k' show/hide keyframes",
                                  "'p' show/hide keypoints"
                                };

        for(auto msg:messages){

            putText(_imshow,msg,cvPoint(30,col));
//            cv::putText(_imshow, msg, cvPoint(30,col), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,200,250), 2, CV_AA);
//            cv::putText(_imshow, msg, cvPoint(30,col), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(100,100,250), 1, CV_AA);
            col+=20;
        }

    }
}

void slam_sglcvviewer::drawScene( ){

    auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };
    auto drawMarker=[](sgl::Scene &Scn, const ucoslam::Marker &m , int width=1){
        auto points=m.get3DPoints();
        Scn.drawLine((sgl::Point3*)&points[0],(sgl::Point3*)&points[1],{0,0,255},width);
        Scn.drawLine((sgl::Point3*)&points[1],(sgl::Point3*)&points[2],{0,255,0},width);
        Scn.drawLine((sgl::Point3*)&points[2],(sgl::Point3*)&points[3],{255,0,0},width);
        Scn.drawLine((sgl::Point3*)&points[3],(sgl::Point3*)&points[0],{155,0,155},width);
    };


    auto  drawPyramid=[](sgl::Scene &Scn,float w,float h,float z,const sgl::Color &color,int width=1){
        Scn.drawLine( {0,0,0}, {w,h,z},color,width);
        Scn.drawLine( {0,0,0}, {w,-h,z},color,width);
        Scn.drawLine( {0,0,0}, {-w,-h,z},color,width);
        Scn.drawLine( {0,0,0}, {-w,h,z},color,width);
        Scn.drawLine( {w,h,z}, {w,-h,z},color,width);
        Scn.drawLine( {-w,h,z}, {-w,-h,z},color,width);
        Scn.drawLine( {-w,h,z}, {w,h,z},color,width);
        Scn.drawLine( {-w,-h,z}, {w,-h,z},color,width);
    };

    auto  drawFrame=[&](sgl::Scene &Scn,Se3Transform &frame,const sgl::Color &color,int width=1){
        Scn.pushModelMatrix(sgl::Matrix44(frame.ptr<float>(0)));
        drawPyramid(Scn,0.1,0.05,0.04,color,width);
        Scn.popModelMatrix();

    };

    _Scene.clear(sgl::Color(255,255,255));
    if(followCamera && !_camPose.empty()){
        cv::Mat aa=_camPose.inv();
        _Scene.setViewMatrix(sgl::Matrix44(aa.ptr<float>(0)));
        _Scene.setModelMatrix();
    }

    sgl::Color stablePointsColor(0,0,0);
    sgl::Color unStablePointsColor(0,0,255);
    sgl::Color normalColor(255,0,0);

    if (showKeyPoints){
        for(size_t i=0;i< mapPoints.size();i++){
            const auto &point=mapPoints[i];
            _Scene.drawPoint( (sgl::Point3*)&point.first, point.second?stablePointsColor:unStablePointsColor,PtSize);
            const auto &normal=mapPointsNormals[i];
            //find the point at disatnce d from origin in direction of normal
            if (showPointNormals){
                float d=0.01;
                cv::Point3f pend=point.first + d*normal;
                _Scene.drawLine( (sgl::Point3*)&point.first, (sgl::Point3*)&pend,normalColor,1);
            }

        }
    }



    for(const auto &m:map_markers){
        drawMarker(_Scene,m.second);
        if(showNumbers)
            _Scene.drawPoints(marker_points[m.first].points,{125,0,255});
    }

    if (!followCamera){
        //draw frames
        if (showKeyFrames){
            for(auto kf:_FSetPoses){
                sgl::Color color(255,0,0);
                if (curKF==kf.first)  color=sgl::Color (0,255,0);
                drawFrame(_Scene,kf.second,color);
                //                    _Scene.pushModelMatrix(sgl::Matrix44(kf.second.ptr<float>(0)));
                //                    drawPyramid(_Scene,0.1,0.05,0.04,color);
                //                    _Scene.popModelMatrix();
            }


            //draw covis graph
            if(showCovisGraph==1){//show all
                std::set<uint64_t> connectionsDrawn;
                for(auto kf:_FSetPoses){
                    cv::Point3f p1=_FSetPoses[kf.first].getCenter();
                    auto neigh=CVGraph.getNeighbors(kf.first);
                    for(auto n:neigh ){
                        if (!connectionsDrawn.count(join(kf.first,n))){
                            auto color=sgl::Color(155,155,155);
                            if (CVGraph.getWeight(kf.first,n)>25)
                                color=sgl::Color(0,255,0);
                            //draw a line between them
                            cv::Point3f p2=_FSetPoses[n].getCenter();
                            _Scene. drawLine( (sgl::Point3*)&p1,(sgl::Point3*) &p2,color  ,1);
                            connectionsDrawn.insert(join(kf.first,n));
                        }
                    }
                }

            }
            else if (showCovisGraph==2 && curKF!=-1){//show only connected to current keyframe
                if (CVGraph.isNode(curKF)){
                    cv::Point3f p1=_FSetPoses[curKF].getCenter();

                    //shows only the links of the current keyframe
                    auto neigh=CVGraph.getNeighbors(curKF);
                    for(auto n:neigh ){
                        auto color=sgl::Color(155,155,155);
                        if (CVGraph.getWeight(curKF,n)>25)
                            color=sgl::Color(125,255,0);
                        //draw a line between them
                        cv::Point3f p2=_FSetPoses[n].getCenter();
                        _Scene. drawLine( (sgl::Point3*)&p1,(sgl::Point3*) &p2,color  ,1);
                        drawFrame(_Scene,_FSetPoses[n],color);

                    }
                }
            }
        }
        //draw camera if it is possible
        if (!_camPose.empty()){
            _Scene.pushModelMatrix(sgl::Matrix44(_camPose.ptr<float>(0)));
            drawPyramid(_Scene,0.1,0.05,0.04,{0,125,255},2);
            _Scene.popModelMatrix();
        }
    }


}

void slam_sglcvviewer::blending(const cv::Mat &a,float f,cv::Mat &io){


    for(int y=0;y<a.rows;y++){
        const cv::Vec3b *ptra=a.ptr<cv::Vec3b>(y);
        cv::Vec3b *ptrio=io.ptr<cv::Vec3b>(y);
        for(int x=0;x<a.cols;x++){
            ptrio[x][0]= ptra[x][0]*f+(1.-f)*ptrio[x][0];
            ptrio[x][1]= ptra[x][1]*f+(1.-f)*ptrio[x][1];
            ptrio[x][2]= ptra[x][2]*f+(1.-f)*ptrio[x][2];
        }
    }
}

void slam_sglcvviewer::createImage(  ) {
    std::unique_lock<std::mutex> lock(drawImageMutex);

     cv::Size subrectsize(_w*_subw_nsize,_h*_subw_nsize);
    if (mode==0) _Scene.setCameraParams(_f,_w,_h,_imshow.ptr<uchar>(0));
    else  _Scene.setCameraParams(_f,subrectsize.width,subrectsize.height);

     drawScene();

    //copy 3d image and color image
    if (mode==0){
        if (!_cameraImage.empty()){
            auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
            cv::resize( _cameraImage,_resizedInImage,subrectsize);
            blending(_resizedInImage,0.8,subrect);
        }
    }
    else{
        if (!_cameraImage.empty()){
            cv::resize( _cameraImage,_imshow, cv::Size(_w,_h));
        }
        else _imshow.setTo(cv::Scalar::all(0));
        auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
        cv::Mat    im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer());
        blending(im3d,0.5,subrect);
    }

    drawText();

}


void slam_sglcvviewer::updateImage(  ) {
     drawScene();
     cv::Size subrectsize(_w*_subw_nsize,_h*_subw_nsize);
    auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
    //copy 3d image and color image
    if (mode==0){
        if (!_cameraImage.empty()) {
            blending(_resizedInImage,0.8,subrect);

        }
    }
    else{
        cv::Mat    im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer());
        blending(im3d,0.5,subrect);
    }
     drawText();
 }

vector<sgl::Point3> slam_sglcvviewer::getMarkerIdPcd(ucoslam::Marker &minfo,float perct=1 )
{
    auto  mult=[](const cv::Mat& m, cv::Point3f p)
    {
        assert(m.isContinuous());
        assert(m.type()==CV_32F);
        cv::Point3f res;
        const float* ptr = m.ptr<float>(0);
        res.x = ptr[0] * p.x + ptr[1] * p.y + ptr[2] * p.z + ptr[3];
        res.y = ptr[4] * p.x + ptr[5] * p.y + ptr[6] * p.z + ptr[7];
        res.z = ptr[8] * p.x + ptr[9] * p.y + ptr[10] * p.z + ptr[11];
        return res;
    };

    int id = minfo.id;
    // marker id as a set of points
    string text = std::to_string(id);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline = 0;
    float markerSize_2 = minfo.size / 2.f;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Mat img(textSize + cv::Size(0, baseline / 2), CV_8UC1, cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0, textSize.height + baseline / 4), fontFace, fontScale, cv::Scalar::all(255),
                thickness, 8);
    // raster 2d points as 3d points
    vector<cv::Point3f> points_id;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            if (img.at<uchar>(y, x) != 0)
                points_id.push_back(cv::Point3f((float(x) / float(img.cols)) - 0.5f, (float(img.rows - y) / float(img.rows)) - 0.5f, 0.f));

    // now,scale
    for (auto& p : points_id)
        p *= markerSize_2;
    // finally, translate
    for (auto& p : points_id)
        p = mult(minfo.pose_g2m, p);


    //select only a fraction of them number of them
    vector<sgl::Point3> s_points_id;
    if(perct!=1){
        int notused=float(points_id.size())*(1-perct);
        vector<char> used(points_id.size(),true);
        for(int i=0;i<notused;i++) used[i]=false;
        std::random_shuffle(used.begin(),used.end());
        //copy only the selected
        s_points_id.reserve(points_id.size());
        for(size_t i=0;i<points_id.size();i++)
            if ( used[i]) s_points_id.push_back(sgl::Point3(points_id[i].x,points_id[i].y,points_id[i].z));
    }
    else
        memcpy(&s_points_id[0],&points_id[0],points_id.size()*sizeof(cv::Point3f));

    return s_points_id;
}


}
#endif
