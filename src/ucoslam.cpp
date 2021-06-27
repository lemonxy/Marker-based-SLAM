#include "ucoslam.h"
#include "stuff/io_utils.h"
using namespace std;
namespace  ucoslam {
Params::Params(){
    global_optimizer= "g2o";//which global optimizer to use
    minBaseLine=0.1;
    keyFrameCullingPercentage=0.95;
    thRefRatio=0.7;
}
void Params::toStream(ostream &str){

    str.write((char*)&detectMarkers,sizeof(detectMarkers));
     str.write((char*)&effectiveFocus,sizeof(effectiveFocus));

    str.write((char*)&forceInitializationFromMarkers,sizeof(forceInitializationFromMarkers));

    str.write((char*)&removeKeyPointsIntoMarkers,sizeof(removeKeyPointsIntoMarkers));
    str.write((char*)&minDescDistance,sizeof(minDescDistance));
    str.write((char*)&baseline_medianDepth_ratio_min,sizeof(baseline_medianDepth_ratio_min));
    str.write((char*)&aruco_markerSize,sizeof(aruco_markerSize));
    str.write((char*)&projDistThr,sizeof(projDistThr));
     str.write((char*)&maxVisibleFramesPerMarker,sizeof(maxVisibleFramesPerMarker));
    str.write((char*)&minNumProjPoints,sizeof(minNumProjPoints));
    str.write((char*)&keyFrameCullingPercentage,sizeof(keyFrameCullingPercentage));
    str.write((char*)&fps,sizeof(fps));
    str.write((char*)&thRefRatio,sizeof(thRefRatio));
    str.write((char*)&maxFeatures,sizeof(maxFeatures));
    str.write((char*)&nOctaveLevels,sizeof(nOctaveLevels));
    str.write((char*)&scaleFactor,sizeof(scaleFactor));


    str.write((char*)&aruco_minerrratio_valid,sizeof(aruco_minerrratio_valid));
    str.write((char*)&aruco_minNumFramesRequired,sizeof(aruco_minNumFramesRequired));
    str.write((char*)&aruco_allowOneFrameInitialization,sizeof(aruco_allowOneFrameInitialization));
    str.write((char*)&aruco_maxRotation,sizeof(aruco_maxRotation));

      str.write((char*)&minBaseLine,sizeof(minBaseLine));
    str.write((char*)&runSequential,sizeof(runSequential));
    toStream__(global_optimizer,str);
    aruco_DetectorParams.toStream(str);


}


void Params::fromStream(istream &str){
    str.read((char*)&detectMarkers,sizeof(detectMarkers));
     str.read((char*)&effectiveFocus,sizeof(effectiveFocus));

    str.read((char*)&forceInitializationFromMarkers,sizeof(forceInitializationFromMarkers));
    str.read((char*)&removeKeyPointsIntoMarkers,sizeof(removeKeyPointsIntoMarkers));
    str.read((char*)&minDescDistance,sizeof(minDescDistance));
    str.read((char*)&baseline_medianDepth_ratio_min,sizeof(baseline_medianDepth_ratio_min));
    str.read((char*)&aruco_markerSize,sizeof(aruco_markerSize));
    str.read((char*)&projDistThr,sizeof(projDistThr));
     str.read((char*)&maxVisibleFramesPerMarker,sizeof(maxVisibleFramesPerMarker));
    str.read((char*)&minNumProjPoints,sizeof(minNumProjPoints));
    str.read((char*)&keyFrameCullingPercentage,sizeof(keyFrameCullingPercentage));
    str.read((char*)&fps,sizeof(fps));
    str.read((char*)&thRefRatio,sizeof(thRefRatio));
    str.read((char*)&maxFeatures,sizeof(nOctaveLevels));
    str.read((char*)&nOctaveLevels,sizeof(nOctaveLevels));
    str.read((char*)&scaleFactor,sizeof(scaleFactor));


    str.read((char*)&aruco_minerrratio_valid,sizeof(aruco_minerrratio_valid));
    str.read((char*)&aruco_minNumFramesRequired,sizeof(aruco_minNumFramesRequired));
    str.read((char*)&aruco_allowOneFrameInitialization,sizeof(aruco_allowOneFrameInitialization));
    str.read((char*)&aruco_maxRotation,sizeof(aruco_maxRotation));
     str.read((char*)&minBaseLine,sizeof(minBaseLine));
    str.read((char*)&runSequential,sizeof(runSequential));

    fromStream__(global_optimizer,str);
    aruco_DetectorParams.fromStream(str);



}

uint64_t  Params::getSignature()const{
    return 2*detectMarkers+3*removeKeyPointsIntoMarkers+minDescDistance+100*baseline_medianDepth_ratio_min+100*aruco_markerSize;
}

void Params::saveToYMLFile(const string &path){
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened())throw std::runtime_error("Could not open "+path);
     fs<<"effectiveFocus"<<effectiveFocus;
    fs<<"maxFeatures" <<maxFeatures;
     fs<<"nOctaveLevels" <<nOctaveLevels;
    fs<<"scaleFactor" <<scaleFactor;
    fs<<"runSequential" <<runSequential;
    fs<<"detectMarkers" <<detectMarkers;
    fs<<"forceInitializationFromMarkers"<<forceInitializationFromMarkers;
    fs<<"aruco_allowOneFrameInitialization" <<aruco_allowOneFrameInitialization;


    fs<<"minBaseLine" <<minBaseLine;
     fs<<"removeKeyPointsIntoMarkers" <<removeKeyPointsIntoMarkers;
    fs<<"minDescDistance" <<minDescDistance;
    fs<<"baseline_medianDepth_ratio_min" <<baseline_medianDepth_ratio_min;
    fs<<"aruco_markerSize" <<aruco_markerSize;
    fs<<"projDistThr" <<projDistThr;
    fs<<"maxVisibleFramesPerMarker" <<maxVisibleFramesPerMarker;
    fs<<"minNumProjPoints" <<minNumProjPoints;
    fs<<"keyFrameCullingPercentage" <<keyFrameCullingPercentage;
    fs<<"fps" <<fps;
    fs<<"thRefRatio" <<thRefRatio;
    fs<<"aruco_minerrratio_valid" <<aruco_minerrratio_valid;

    fs<<"aruco_minNumFramesRequired" <<aruco_minNumFramesRequired;
    fs<<"aruco_maxRotation" <<aruco_maxRotation;

    fs<<"global_optimizer" <<global_optimizer;

    aruco_DetectorParams.save(fs);

}


void Params:: readFromYMLFile(const string &filePath){



    //first
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error("CameraParameters::readFromXMLFile could not open file:"+filePath);
    attemtpRead("detectMarkers",detectMarkers,fs);
    attemtpRead("AprilTag", aprilTag, fs);
    attemtpRead("effectiveFocus",effectiveFocus,fs);
    attemtpRead("maxFeatures",maxFeatures,fs);
     attemtpRead("removeKeyPointsIntoMarkers",removeKeyPointsIntoMarkers,fs);
    attemtpRead("minDescDistance",minDescDistance,fs);
    attemtpRead("baseline_medianDepth_ratio_min",baseline_medianDepth_ratio_min,fs);
    attemtpRead("aruco_markerSize",aruco_markerSize,fs);
    attemtpRead("projDistThr",projDistThr,fs);
     attemtpRead("maxVisibleFramesPerMarker",maxVisibleFramesPerMarker,fs);
    attemtpRead("minNumProjPoints",minNumProjPoints,fs);
    attemtpRead("keyFrameCullingPercentage",keyFrameCullingPercentage,fs);
    attemtpRead("fps",fps,fs);
    attemtpRead("thRefRatio",thRefRatio,fs);
    attemtpRead("nOctaveLevels",nOctaveLevels,fs);
    attemtpRead("scaleFactor",scaleFactor,fs);
    attemtpRead("aruco_minerrratio_valid",aruco_minerrratio_valid,fs);
    attemtpRead("aruco_minNumFramesRequired",aruco_minNumFramesRequired,fs);
    attemtpRead("aruco_allowOneFrameInitialization",aruco_allowOneFrameInitialization,fs);
    attemtpRead("minBaseLine",minBaseLine,fs);
    attemtpRead("runSequential",runSequential,fs);
    attemtpRead("global_optimizer",global_optimizer,fs);
    attemtpRead("aruco_maxRotation",aruco_maxRotation,fs);
    attemtpRead("max_makr_rep_err", max_makr_rep_err, fs);
    attemtpRead("min_distR", min_distR, fs);
    attemtpRead("max_theta", max_theta, fs);
    attemtpRead("delta_theta", delta_theta, fs);


    aruco_DetectorParams.load(fs);




}


}
