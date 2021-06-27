//
// Created by wangzb on 17/4/2021.
//

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <pangolin/pangolin.h>

#include <unistd.h>


#include "optimization/ippe.h"
#include "aruco/marker.h"
#include "aruco/markerdetector.h"
#include "aruco/markerlabeler.h"
using namespace std;
void drawTag(pangolin::OpenGlMatrix &Twc)
{
    const float w = 0.165;
    const float h = 0.165;
    const float z = w*1.0;

    glPushMatrix();
    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
    //一个是整型,一个是浮点数类型
    glMultMatrixd(Twc.m);
    //设置绘制图形时线的宽度
    glPointSize(3);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_POINTS);
    glVertex3f(0.0,0.0,z);
    glEnd();

    glLineWidth(0.8);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f, 1.0f, 0.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINE_LOOP);
    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z);
    glEnd();

    glPopMatrix();
}
void drawFrame(cv::Mat pose, vector<GLfloat> color)
{
    GLfloat red = color[0];
    GLfloat green = color[1];
    GLfloat blue = color[2];
    float size = 0.165;
    vector<cv::Point3f> marker_points =
            {
                    cv::Point3f (-size/2., size/2., 0),
                    cv::Point3f (size/2., size /2.,0),
                    cv::Point3f (size/2., -size /2.,0),
                    cv::Point3f (-size/2., -size /2.,0)
            };

    vector<cv::Point3f> marker_WorldPos;
    marker_WorldPos.resize(4);
    for (int i = 0; i < 4; ++i)
    {
        for (auto &p:marker_points)
        {
            cv::Point3f point = p;
            marker_WorldPos[i].x = pose.at<float>(0,0)*point.x + pose.at<float>(0,1)*point.y +
                                   pose.at<float>(0,2)*point.z + pose.at<float>(0,3);
            marker_WorldPos[i].y = pose.at<float>(1,0)*point.x + pose.at<float>(1,1)*point.y +
                                   pose.at<float>(1,2)*point.z + pose.at<float>(1,3);
            marker_WorldPos[i].z = pose.at<float>(2,0)*point.x + pose.at<float>(2,1)*point.y +
                                   pose.at<float>(2,2)*point.z + pose.at<float>(2,3);
            i++;

        }
    }

    glLineWidth(0.8);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glColor3f(red,green,blue);
    glVertex3f(marker_WorldPos[0].x, marker_WorldPos[0].y, marker_WorldPos[0].z);
    glVertex3f(marker_WorldPos[1].x, marker_WorldPos[1].y, marker_WorldPos[1].z);

    glVertex3f(marker_WorldPos[1].x, marker_WorldPos[1].y, marker_WorldPos[1].z);
    glVertex3f(marker_WorldPos[2].x, marker_WorldPos[2].y, marker_WorldPos[2].z);
//            glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(marker_WorldPos[2].x, marker_WorldPos[2].y, marker_WorldPos[2].z);
    glVertex3f(marker_WorldPos[3].x, marker_WorldPos[3].y, marker_WorldPos[3].z);

    glVertex3f(marker_WorldPos[3].x, marker_WorldPos[3].y, marker_WorldPos[3].z);
    glVertex3f(marker_WorldPos[0].x, marker_WorldPos[0].y, marker_WorldPos[0].z);
    glEnd();
}


void Initialize(vector<cv::Mat> vpose1, vector<cv::Mat> vpose2)
{
    pangolin::CreateWindowAndBind("viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,2000,2000,512,389,0.1,100),
            pangolin::ModelViewLookAt(0,-10,-10, 0,0,0, pangolin::AxisNegY)
    );
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while (!pangolin::ShouldQuit())
    {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        drawTag(Twc);
        s_cam.Follow(Twc);
        vector<GLfloat> color1 = {1.0f, 0.0f, 0.0f};
        vector<GLfloat> color2 = {0.0f, 1.0f, 0.0f};
        for (size_t i = 0; i < vpose1.size(); ++i)
        {
            drawFrame(vpose1[i],color1);
            drawFrame(vpose2[i],color2);
        }

        pangolin::FinishFrame();
//        usleep(100000);
    }
    usleep(300000);
    return;
}

void getParams(const float tagSize, std::vector<cv::Point3f> &objectPoints,
               cv::Mat &K, cv::Mat &distCoef)
{
    cv::Point3f p3d1 = {-tagSize / 2.0f, tagSize / 2.0f, 0.0};
    cv::Point3f p3d2 = {tagSize / 2.0f, tagSize / 2.0f, 0.0};
    cv::Point3f p3d3 = {tagSize / 2.0f, -tagSize / 2.0f, 0.0};
    cv::Point3f p3d4 = {-tagSize / 2.0f, -tagSize / 2.0f, 0.0};
    objectPoints.emplace_back(p3d1);
    objectPoints.emplace_back(p3d2);
    objectPoints.emplace_back(p3d3);
    objectPoints.emplace_back(p3d4);

    cv::Mat _K = cv::Mat::eye(3,3,CV_32F);
    _K.at<float>(0,0) = 1.4331076784077552e+03;
    _K.at<float>(1,1) = 1.4343473528699762e+03;
    _K.at<float>(0,2) = 1.0080640941408149e+03;
    _K.at<float>(1,2) = 7.7739057461427251e+02;
    _K.copyTo(K);

    distCoef.create(5,1,CV_32F);
    distCoef.at<float>(0) = -3.6148028754215483e-01;
    distCoef.at<float>(1) = 1.8878996876111359e-01;
    distCoef.at<float>(2) = 4.2524901401396973e-05;
    distCoef.at<float>(3) = -1.4754956468004843e-05;
    distCoef.at<float>(4) = -5.8449938715207143e-02;
}

int main(int argv, char *argc[])
{
    if (argv < 2)
        return 1;
    std::vector<cv::Mat> vframes;
    for (int i = 1; i < argv; ++i) {
        cv::Mat frame = cv::imread(argc[i], cv::IMREAD_GRAYSCALE);
        vframes.emplace_back(frame);
    }
    /* for (int i = 0; i < 270; ++i) {
         string filename = to_string(930+i);
         filename +=".jpg";
         cv::Mat frame = cv::imread(filename, cv::IMREAD_GRAYSCALE);
         vframes.emplace_back(frame);
     }*/
    aruco::MarkerDetector MDetector;
    aruco::MarkerDetector::Params params;
    params.setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    MDetector.setDictionary(aruco::Dictionary::ARUCO);

    struct marker_info {
        uint32_t id;
        std::map<uint32_t, std::vector<cv::Point2f> > observations;
        std::vector<cv::Mat> pose1;
        std::vector<cv::Mat> pose2;
        std::vector<double> error1;
        std::vector<double> error2;
    };
    ///1, extract Markers
    std::vector<uint32_t> marker_ids;
    std::vector<marker_info> vmarkerInfos;
    std::vector<std::vector<aruco::Marker> > vvMarkers;

    cv::namedWindow("tag_viewer", cv::WINDOW_AUTOSIZE);
    for (size_t j = 0; j < vframes.size(); ++j) {
        std::vector<aruco::Marker> markers;
        MDetector.detect(vframes[j],markers);
        vvMarkers.emplace_back(markers);
        for (size_t k = 0; k < markers.size(); ++k)
        {
            markers[k].draw(vframes[j], cv::Scalar(0,255,255));
            bool found = false;
            for (size_t h = 0; h < vmarkerInfos.size(); ++h) {
                if (vmarkerInfos[h].id == markers[k].id) {
                    vector<cv::Point2f> vp2ds = markers[k];

                    vmarkerInfos[h].observations.insert(make_pair(j, vp2ds));
                    found = true;
                    break;
                }
            }
            if (found)
                continue;
            struct marker_info pi;
//            pi = (struct marker_info*)malloc(sizeof(struct marker_info));
            pi.id = markers[k].id;
            vector<cv::Point2f> vp2ds = markers[k];
            pi.observations.insert(make_pair(j, vp2ds));
            vmarkerInfos.emplace_back(pi);
            marker_ids.emplace_back(markers[k].id);
        }
    }

    cv::imshow("tag_viewer", vframes[0]);
    cv::waitKey(0);

    cv::Mat K, distCoef;
    std::vector<cv::Point3f> objectPoints;
    float tagSize = 0.165;
    getParams(tagSize, objectPoints, K, distCoef);

    ///2, compute MarkersPose
    std::vector<std::vector<cv::Mat> > vvMarkerPose1;
    std::vector<std::vector<cv::Mat> > vvMarkerPose2;
    for (size_t i = 0; i < vvMarkers.size(); ++i) {
        ///i : number of frames
//        cout<<to_string(930+i)<<".jpg"<<endl;
        std::vector<cv::Mat> vMarkerPose1;
        std::vector<cv::Mat> vMarkerPose2;
        std::vector<aruco::Marker> vmarkers = vvMarkers[i];
        for (size_t j = 0; j < vmarkers.size(); ++j) {
            ///j: number of marker
            int idx = vmarkers[j].id;
            std::vector<cv::Point2f> p2ds = vmarkers[j];
            for (size_t k = 0; k < p2ds.size(); ++k)
            {
                cout<<p2ds[k].x<<", "<<p2ds[k].y<<endl;
            }
//            std::vector<cv::Mat> vRvecs;
//            std::vector<cv::Mat> vtvecs;
//            cv::Mat ReprojError;
//            cv::solvePnPGeneric(objectPoints, p2ds, K, distCoef,
//                                vRvecs, vtvecs, false,
//                                cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(),
//                                cv::noArray(), ReprojError);
            vector<pair<cv::Mat, double> > poses_errs;
            poses_errs = IPPE::solvePnP_(objectPoints,p2ds,K,distCoef);



            cv::Mat R1 = poses_errs[0].first.rowRange(0,3).colRange(0,3);
            cv::Mat R2 = poses_errs[1].first.rowRange(0,3).colRange(0,3);
            cv::Mat deltaR = R1*R2.t();
            cv::Mat vec_R;
            cv::Rodrigues(deltaR,vec_R);
            double delta = cv::norm(vec_R);
            cout<<"marker id: "<<vmarkers[j].id<<endl;
            cout<<"error1 = "<<poses_errs[0].second<<
                ", error2 = "<<poses_errs[1].second<<
                ", ratio = "<<poses_errs[0].second/poses_errs[1].second<<endl;
            cout<<"delta = "<<delta<<endl;
//            cv::Mat T1 = cv::Mat::eye(4, 4, CV_32F);
//            cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
//            R1.copyTo(T1.rowRange(0, 3).colRange(0, 3));
//            vtvecs[0].copyTo(T1.rowRange(0, 3).col(3));
//            R2.copyTo(T2.rowRange(0, 3).colRange(0, 3));
//            vtvecs[1].copyTo(T2.rowRange(0, 3).col(3));
//            vMarkerPose1.emplace_back(T1);
//            vMarkerPose2.emplace_back(T2);
            vMarkerPose1.emplace_back(poses_errs[0].first);
            vMarkerPose2.emplace_back(poses_errs[1].first);
//            vector<uint32_t>::iterator iter;
//            iter = find(marker_ids.begin(), marker_ids.end(), vmarkers[j].id);
//            if (iter == marker_ids.end())
//                continue;
//            int index = iter - marker_ids.begin();
//            vmarkerInfos[index].pose1.emplace_back(T1);
//            vmarkerInfos[index].pose2.emplace_back(T2);
//            vmarkerInfos[index].error1.emplace_back(ReprojError.at<float>(0));
//            vmarkerInfos[index].error2.emplace_back(ReprojError.at<float>(1));

        }
        vvMarkerPose1.emplace_back(vMarkerPose1);
        vvMarkerPose2.emplace_back(vMarkerPose2);
    }

    Initialize(vvMarkerPose1[0], vvMarkerPose2[0]);
    return 0;
}

