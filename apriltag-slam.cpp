//
// Created by wangzb on 31/3/2021.
//
#include "slam.h"
#include "mapviewer.h"
#include "stuff/timers.h"
#include <aruco/posetracker.h>
#include <Eigen/Geometry>

void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp);
class CmdLineParser{int argc; char **argv; public:
    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
};

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
int main(int argc,char **argv) {
    try {
        CmdLineParser cml(argc, argv);
        if (argc < 3) {
            cerr
                    << "Usage: videofile  camera_params.yml  [-conf configParams.yml] [-outlog outLog.log] [-dic DICT] [-markersize val]"
                    << endl;
            cerr << "\tvideofile: input video" << endl;
            cerr << "\tcamera_params.yml: file with calibration parameters. Use OpenCv tool or ArUco tool." << endl;
            cerr
                    << "\t-conf <file>: specifies a configuration file for the aruco detector. Overrides [-dic and -markersize]"
                    << endl;
            cerr
                    << "\t-outlog <file>: Runs the video twice. The second time it records the poses and at the end saves the result in the file indicated. Used for evaluation purposes."
                    << endl;
            cerr << "\t-markersize <float>: specifies the marker size in meters. Default value is 1." << endl;
            cerr << "\t-dic <string>: indicates the dictionary to be employed. Default value is ARUCO_MIP_36h12"
                 << endl;
            cerr << "\t Possible Dictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes()) cerr << dict << " ";
            cerr << endl;
            return -1;
        }

        vector<string> vstrImageLeft;
        vector<string> vstrImageRight;
        vector<double> vTimestamps;
        LoadImages(string(argv[1]), vstrImageLeft,
                   vstrImageRight, vTimestamps);

        const int nImages = vstrImageLeft.size();
        if (nImages < 1) {
            cout << "no image input\n";
            return -1;
        }

        ucoslam::Slam Slam;
        ucoslam::ImageParams image_params;
        ucoslam::Params params;
        cv::Mat in_image;
        image_params.readFromXMLFile(argv[2]);
        if (cml["-conf"]) params.readFromYMLFile(cml("-conf"));
        else {
            params.aruco_DetectorParams.dictionary = cml("-dic", "ARUCO_MIP_36h12");
            params.aruco_markerSize = stof(cml("-markersize", "1"));
        }
        //you can manually set desired dictionary as
        //params.aruco_DetectorParams.dictionary="ARUCO"; Change "ARUCO" by the name of the desired dictionary (see aruco::Dictionary class)

        auto TheMap = std::make_shared<ucoslam::Map>();

        //Create the viewer to see the images and the 3D
        auto TViewer = ucoslam::MapViewer::create("Cv");
        TViewer->set("showNumbers", "1");
        TViewer->set("canLeave", "1");
        TViewer->set("mode", "0");
        TViewer->set("modelMatrix",
                     "0.998437 -0.0490304 0.0268194 0  0.00535287 0.561584 0.827403 0  -0.0556289 -0.825967 0.560969 0  0 0 0 1");
        TViewer->set("viewMatrix", " 1 0 0 0.01  0 4.63287e-05 -1 0.910185  0 1 4.63287e-05 9.18  0 0 0 1 ");

        Slam.setParams(TheMap, params);
        cout << "===@params=== \n";
        cout << "Marker size " << params.aruco_markerSize << endl;
        cout << "min distance " << params.minBaseLine << endl;
        cout << "min err Ratio " << params.aruco_minerrratio_valid << endl;
        cout << "fx=" << image_params.fx() << ", fy=" << image_params.fy() << endl;
        cout << "cx=" << image_params.cx() << ", cy=" << image_params.cy() << endl;
        cout << "distorsion = \n" << image_params.Distorsion << endl;

//        //Ok, lets start
        ucoslam::TimerAvrg Fps;
        char k = 0;
        for (int ni = 0; ni < nImages; ++ni) {
            in_image = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
            int currentFrameIndex = ni+1;
            Slam.process(in_image, image_params, currentFrameIndex);
//            cout<<"Image "<<currentFrameIndex<<" fps="<<1./Fps.getAvrg()<<endl;
            k = TViewer->show(TheMap, in_image, Slam.getCurrentPose_f2g(),
                              "#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1. / Fps.getAvrg()));

            if (k == 27)
                break;
        }


        cerr << "The markermap is saved to markermap.yml" << endl;
        TheMap->saveToMarkerMap("markermap.yml");


        //
        //requires to repeat the sequence to evaluate the error???
        if (!cml["-outlog"]) return 1;


        Slam.resetTracker();
        Slam.setMode(ucoslam::Slam::MODE_LOCALIZATION);
        std::map<int, cv::Mat> frame_pose_map;  // set of poses and the frames they were detected

        Fps.reset();
        k = 0;
        for (int ni = 0; ni < nImages; ++ni) {
            in_image = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
            int currentFrameIndex = ni+1;
            cv::Mat pose = Slam.process(in_image, image_params, currentFrameIndex);
            if (!pose.empty())
                frame_pose_map.insert({currentFrameIndex, pose});
            k = TViewer->show(TheMap, in_image, Slam.getCurrentPose_f2g(),
                              "Tracking Only #" + std::to_string(currentFrameIndex) + " fps=" +
                              to_string(1. / Fps.getAvrg()));
            if (k == 27)
                break;
        }

        //save the results
        savePosesToFile(cml("-outlog"), frame_pose_map);
        cerr << "The poses have been saved to " << argv[4] << endl;


    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
    }
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    if(!fTimes.is_open())
    {
        cout<<"Bag is failed!\n";
        return;
    }
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image0/";
    string strPrefixRight = strPathToSequence + "/image1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;
    M_in.convertTo(M,CV_64F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<double>(0,3);
    ty=M.at<double>(1,3);
    tz=M.at<double>(2,3);
}

void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp)
{
    std::ofstream file(filename);
    double qx, qy, qz, qw, tx, ty, tz;
    for (auto frame : fmp)
    {
        if (!frame.second.empty())
        {
            cv::Mat minv=frame.second.inv();
            getQuaternionAndTranslationfromMatrix44(minv, qx, qy, qz, qw, tx, ty, tz);
            file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                 << qw << endl;
        }
    }
}

