#include "globaloptimizer_cvba_ucosba.h"
#include "../slam.h"
#include "../utils.h"
namespace ucoslam{


void GlobalOptimizerCVBA_UCO::setParams(Slam &w,const ParamSet &p)throw(std::exception){
    _params=p;
    _params.markerSize=w.markerSize();
    convert(w,_params,_bdata);
}

void GlobalOptimizerCVBA_UCO::convert(const Slam &sl,  ParamSet &p, cvba::BundlerData<float> &bdata)

{

    const Map &map=sl.TheMap;
    const FrameSet &fset=sl.TheFrameSet;
    const ImageParams &ip=sl.TheImageParams;




    //determine the frames to be used
    //if non indicated, use all
    if (p.used_frames.size()==0)
        for(const auto &f:fset){
            assert(fset.is(f.idx));;//to remove
            p.used_frames.insert(f.idx);
        }
    //add also the fixed ones (in case they are not in the used_frames map
    for(auto f:p.fixed_frames) p.used_frames.insert(f);
    //if not enough used frames, leave now
    if (_params.used_frames.size()<2)return;


    //determine the points and markers to be used
    mappoints_used.clear();
    mapmarkers_used.clear();

    for(auto f:p.used_frames){//go through frames, and add the id of the mappoints projecting in them
        if ( p.fixed_frames.count(f)==0){//is in a non fixed frame
            for(auto id:fset[f].ids){//go through keypoints assignaions
                if (id!=std::numeric_limits<uint32_t>::max()){//if  valid
                    //count how many times it appears
                    if (mappoints_used.count(id)==0) mappoints_used [id]=1;//first time
                    else mappoints_used [id]=mappoints_used [id]+1;
                }
            }
            //now, the valid markers
            for(const auto &m:fset[f].markers){
                assert(map.map_markers.count(m.id) );
                if ( mapmarkers_used.count(m.id)==0  && map.map_markers.at(m.id).pose_g2m.isValid()){
                    //is it visible in at least two of the markers we are employing?
                    int nvisframes=0;
                    for(auto mf:map.map_markers.at(m.id).frames)
                        if ( p.used_frames.count(mf)) nvisframes++;
                    //yes, add it
                    if (nvisframes>1) mapmarkers_used.insert({m.id,0});
                }
            }
        }
    }

    //remove points with less than 2 views
    vector<uint32_t> points2remove;
    for(auto mp:mappoints_used)
        if (mp.second<2) points2remove.push_back(mp.first);
    for(auto pr:points2remove) mappoints_used.erase(pr);

    //create a relation between the frame idx and an index in the vector of bundler data
    frame_index.clear();
    uint32_t idx=0;
    for(auto fidx:p.used_frames) frame_index.insert({fidx,idx++});

    //now, start adding data to BundlerData and set in the vectors they position in the BundlerData.points to revert the process
    bdata.resize(p.used_frames.size(),mappoints_used.size()+mapmarkers_used.size()*4);
    int currBDataPoint=0;
    for(auto &mp: mappoints_used){
            mp.second=currBDataPoint;//set position in vector
            bdata.points[currBDataPoint]=map.map_points.at(mp.first).getCoordinates();
            //now find in which cameras it is found and set the projections
            for(const auto &f_id:map.map_points.at(mp.first).frames){
                if ( p.used_frames.count(f_id.first)!=0){
                    assert(frame_index.count(f_id.first));
                    auto fbdataidx=frame_index[f_id.first ];
                    bdata.imagePoints[fbdataidx][currBDataPoint]= fset.at( f_id.first).und_kpts[f_id.second].pt;
                    bdata.visibility[fbdataidx][currBDataPoint]=1;
                }
            }
            currBDataPoint++;
    }
    //for each marker, its index position in the bundlerdata vector
     //go with the markers now
    for(auto &marker_id:   mapmarkers_used){
        marker_id.second= currBDataPoint;
        const auto &marker3dpoints= map.map_markers.at(marker_id.first) .get3DPoints();
        for(int c=0;c<4;c++)  bdata.points[currBDataPoint+c]=marker3dpoints[c];
        //now, set the visibility
        for(const auto &frameidx: map.map_markers.at(marker_id.first).frames){
            if ( p.used_frames.count(frameidx)!=0){
                assert(frame_index.count(frameidx));
                auto fbdataidx=frame_index[frameidx];
                const auto &marker_proj=fset.at(frameidx).getMarker(marker_id.first);
                for(int c=0;c<4;c++){
                    bdata.imagePoints[fbdataidx][currBDataPoint+c]=marker_proj[c];
                    bdata.visibility[fbdataidx][currBDataPoint+c]=1;
                }
            }
        }
        currBDataPoint+=4;
    }

    //now, the camera data
    for(auto frameidx:p.used_frames){
        const auto & frame=fset.at(frameidx);
        auto bdata_fidx=frame_index[frameidx];
        sl.TheImageParams.CameraMatrix.copyTo(bdata.cameraMatrix[bdata_fidx]);
        sl.TheImageParams.Distorsion.copyTo(bdata.distCoeffs[bdata_fidx]);
        bdata.distCoeffs[bdata_fidx]=cv::Mat::zeros(1,5,CV_32F);
        bdata.R[bdata_fidx]= frame.pose_f2g.getRvec();
        bdata.T[bdata_fidx]= frame.pose_f2g.getTvec();
    }
//add last as fixed. Do it here and not before or its points and markers might not be included
    if (p.fixFirstFrame)
        p.fixed_frames.insert(fset.front().idx);

}
void __x__saveToPcd(string filepath,const vector<cv::Point3f> &points)throw(std::exception){
    ofstream filePCD(filepath.c_str(),ios::binary);
    if (!filePCD) throw std::runtime_error("could not open:"+filepath);
     filePCD<<"# .PCD v.7 - Point Cloud Data file format"<<endl;
    filePCD<<"VERSION .7"<<endl;
    filePCD<<"FIELDS x y z "<<endl;
    filePCD<<"SIZE 4 4 4 "<<endl;
    filePCD<<"TYPE F F F "<<endl;
    filePCD<<"COUNT 1 1 1 "<<endl;
    filePCD<<"VIEWPOINT 0 0 0 1 0 0 0"<<endl;
    filePCD<<"WIDTH "<<points.size() <<endl;
    filePCD<<"HEIGHT "<<1<<endl;
    filePCD<<"POINTS "<<points.size() <<endl;
    filePCD<<"DATA binary"<<endl;
        //swap rgb
        filePCD.write ( ( char* ) &points[0],sizeof ( cv::Point3f)*points.size() );
}
void GlobalOptimizerCVBA_UCO::optimize()throw(std::exception) {
    if (_params.used_frames.size()<2)return;

    auto bundler=cvba::Bundler::create("uco_sba");
    cvba::Bundler::Params params;
    params.fixAllCameraIntrinsics=false;


    for(auto fidx:_params.fixed_frames){
        params.fixCameraExtrinsics(frame_index.at(fidx));
        cout<<"Fixed camera "<<fidx<<endl;
    }
    params.iterations=_params.nIters;
    params.verbose=_params.verbose;
    params.nThreads=1;
    params.type=cvba::Bundler::MOTIONSTRUCTURE;
    params.minError=0.1;
    bundler->setParams(params);
    _debug_msg_("prev-rep:"<<_bdata.avrg_repj_error());
    __x__saveToPcd("pprev.pcd",_bdata.points);
    bundler->run(_bdata);
    _debug_msg_("posst-rep:"<<_bdata.avrg_repj_error());

    __x__saveToPcd("ppost.pcd",_bdata.points);
 }

void GlobalOptimizerCVBA_UCO::getResults(Slam &w)throw(std::exception){
    //get data back from bdata
    if (_params.used_frames.size()<2)return;

      Map &map=w.TheMap;
      FrameSet &fset=w.TheFrameSet;
      //ImageParams &ip=w.TheImageParams;

    //now, start adding data to BundlerData and set in the vectors they position in the BundlerData.points to revert the process
    for(const auto &mp:mappoints_used){
             map.map_points[mp.first].setCoordinates( _bdata.points[  mp.second ]);
    }

    //for markers, need to determine the 6d params from the 4 points   here: Use horn transform

    auto orgMarker3dPoints=Marker::get3DPoints(se3(),w.markerSize(),false);
    for(auto marker_id:   mapmarkers_used){
        auto startBdp= marker_id.second;
        vector<cv::Point3f> OptimizedPointsMarker(4);
        for(int c=0;c<4;c++) OptimizedPointsMarker[c]=_bdata.points[startBdp+c];
        auto &marker=map.map_markers.at(marker_id.first);
        double err;
        marker.pose_g2m=rigidBodyTransformation_Horn1987(orgMarker3dPoints, OptimizedPointsMarker,&err);
    }

    //now, set the frames' locations
    for(auto frameidx:_params.used_frames){
        auto & frame=fset.at(frameidx);
        auto bdata_fidx=frame_index[frameidx];
        frame.pose_f2g.setRotation(_bdata.R[bdata_fidx] );
        frame.pose_f2g.setTranslation(_bdata.T[bdata_fidx] );

    }

    //force to change the image params
    _bdata.cameraMatrix[0].copyTo(w.TheImageParams.CameraMatrix);
    _bdata.distCoeffs[0].copyTo(w.TheImageParams.Distorsion);
//must update the location of the markers with only one view
//not sure how it will play when only optimizing a subset of frames
//assert(false);
}

void GlobalOptimizerCVBA_UCO::optimize(Slam &w,const ParamSet &p )throw(std::exception)
{

    setParams(w,p);
     optimize();
    getResults(w);

}

void GlobalOptimizerCVBA_UCO::saveToStream_impl(std::ostream &str){

}

void GlobalOptimizerCVBA_UCO::readFromStream_impl(std::istream &str)
{

}

}
