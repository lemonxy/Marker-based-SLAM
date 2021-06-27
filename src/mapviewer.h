#ifndef _MapViewer_ucoslam_H
#define _MapViewer_ucoslam_H
#include "ucoslam_exports.h"
#include "map.h"
#include <mutex>
#include <thread>

namespace ucoslam{

/**Base Class for implementing map viewers
 */
class UCOSLAM_API MapViewer{
public:
    MapViewer();
    virtual ~MapViewer(){}
    virtual int show(   std::shared_ptr<Map> map,const cv::Mat &cameraImage_f2g=cv::Mat(),const cv::Mat &cameraPose=cv::Mat(),string userMessage=""){return -1;}
    virtual int show(   Slam *slam, const cv::Mat &cameraImage,string userMessage=""){return -1;}

    virtual void set(const std::string & param,const std::string & val){}//sets a parameter
     //
    void showKeyFrames(bool v){bShowkeyFrames=v;}
    void showFramePoses(bool v){bShowFramePoses=v;}

    virtual cv::Mat getImage(){return cv::Mat();}//returns the image generated after calling show
    virtual void getImage(cv::Mat &image){ }//returns the image generated after calling show


    //Factory : Creates the viewer and returns it. First, tries with Qt, then Pangolin one
    static std::shared_ptr<MapViewer> create(string create_string="Pangolin Qt ");

    bool & exitOnUnusedKey(){ return bExitOnUnUnsedKey;}


protected:
    bool bExitOnUnUnsedKey=true;
    bool bShowkeyFrames,bShowFramePoses;
};
}
#endif
