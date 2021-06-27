#include "mapviewer.h"
#include "viewers/slam_sglcvviewer.h"
namespace ucoslam{
MapViewer::MapViewer(){
    bShowkeyFrames=true;
    bShowFramePoses=false;
}

std::shared_ptr<MapViewer> MapViewer::create(string create_string ){

    vector<string> vlist;
    stringstream sstr(create_string);
    while(!sstr.eof()){
        string cmd;
         if( sstr>>cmd)  vlist.push_back(cmd);
    }

    for(auto viewertype:vlist){
        if (viewertype.find("Cv")!=std::string::npos)
            return std::make_shared<slam_sglcvviewer>();
        if( viewertype.find("Dummy")!=std::string::npos)
            return std::make_shared<MapViewer>();//else returns this dummy one

    }
    return std::make_shared<slam_sglcvviewer>();//else returns this   one


}

}
