#include "utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "stuff/debug.h"
#include "basic_types/marker.h"
#include "basic_types/frame.h"
#include "optimization/ippe.h"
using namespace std;
namespace ucoslam
{

    /**
    * Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
    */
    cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
    cv::Mat M;
    cv::Mat R,T;
    R_.copyTo ( R );
    T_.copyTo ( T );
    if ( R.type() ==CV_64F ) {
        assert ( T.type() ==CV_64F );
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );

        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R64;
            R.convertTo ( R64,CV_64F );
            R.copyTo ( R33 );
        }
        for ( int i=0; i<3; i++ )
            Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
        M=Matrix;
    } else if ( R.depth() ==CV_32F ) {
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R32;
            R.convertTo ( R32,CV_32F );
            R.copyTo ( R33 );
        }

        for ( int i=0; i<3; i++ )
            Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
        M=Matrix;
    }

    if ( forceType==-1 ) return M;
    else {
        cv::Mat MTyped;
        M.convertTo ( MTyped,forceType );
        return MTyped;
    }
    }

    /**
    * @brief getRTfromMatrix44
    * @param M
    * @param R
    * @param T
    * @param useSVD
    */
    void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T,bool useSVD) {

    assert ( M.cols==M.rows && M.cols==4 );
    assert ( M.type() ==CV_32F || M.type() ==CV_64F );
    //extract the rotation part
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    if (useSVD){
        cv::SVD svd ( r33 );
        cv::Mat Rpure=svd.u*svd.vt;
        cv::Rodrigues ( Rpure,R );
    }else
        cv::Rodrigues ( r33,R );

    T.create ( 1,3,M.type() );
    if ( M.type() ==CV_32F )
        for ( int i=0; i<3; i++ )
            T.ptr<float> ( 0 ) [i]=M.at<float> ( i,3 );
    else
        for ( int i=0; i<3; i++ )
            T.ptr<double> ( 0 ) [i]=M.at<double> ( i,3 );
    }

    void remove_unused_matches(std::vector<cv::DMatch> &matches ){
    matches.erase(std::remove_if(matches.begin(),matches.end(), [](const cv::DMatch &m){return m.trainIdx==-1 || m.queryIdx==-1;}), matches.end());
    }
    void remove_bad_matches(std::vector<cv::DMatch> &matches ,const vector<bool> &vBadMatches){
    assert(matches.size()==vBadMatches.size());
    for(size_t i=0;i<matches.size();i++)
        if (vBadMatches[i]) matches[i].trainIdx=-1;
    matches.erase(std::remove_if(matches.begin(),matches.end(), [](const cv::DMatch &m){return m.trainIdx==-1 || m.queryIdx==-1;}), matches.end());

    }


    void filter_ambiguous_query(  std::vector<cv::DMatch> &matches ){
    if (matches.size()==0)return;
    //determine maximum values of queryIdx
    int maxT=-1;
    for(auto m:matches)   maxT=std::max(maxT,m.queryIdx);

    //now, create the vector with the elements
    vector<int> used(maxT+1,-1);
    vector<cv::DMatch> best_matches(maxT);
    int idx=0;
    bool needRemove=false;

    for(auto &match:matches ){
        if (used[match.queryIdx]==-1){
            used[match.queryIdx]=idx;
        }
        else{
            if ( matches[ used[match.queryIdx] ].distance>match.distance){
                matches[ used[match.queryIdx] ].queryIdx=-1;//annulate the other match
                used[match.queryIdx]=idx;
                needRemove=true;
            }
            else{
                match.queryIdx=-1;//annulate this match
                needRemove=true;
            }
        }
        idx++;
    }


    if (needRemove) remove_unused_matches(matches);

    }


    void filter_ambiguous_train(  std::vector<cv::DMatch> &matches ){
    if (matches.size()==0)return;
    //determine maximum values of train
    int maxT=-1;
    for(auto m:matches)
        maxT=std::max(maxT,m.trainIdx);

    //now, create the vector with the elements
    vector<int> used(maxT+1,-1);
    vector<cv::DMatch> best_matches(maxT);
    int idx=0;
    bool needRemove=false;

    for(auto &match:matches ){
        if (used[match.trainIdx]==-1){
            used[match.trainIdx]=idx;
        }
        else{
            if ( matches[ used[match.trainIdx] ].distance>match.distance){
                matches[ used[match.trainIdx] ].trainIdx=-1;//annulate the other match
                used[match.trainIdx]=idx;
                needRemove=true;
            }
            else{
                match.trainIdx=-1;//annulate this match
                needRemove=true;

            }
        }
        idx++;
    }

    if (needRemove) remove_unused_matches(matches);

    }


    /**Basic matching filtering using Brute force with Hamming
    * @brief match_frames
    * @param f1
    * @param f2
    * @param nn_match_ratio
    * @return
    */
    vector<cv::DMatch> match_frames(const Frame &f1,const Frame &f2, double nn_match_ratio ){
    vector<cv::DMatch> matches;
    #if 0
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
    vector< vector<cv::DMatch> > matches;
    matcher->knnMatch(f1.desc, f2.desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance  < nn_match_ratio * matches[i][1].distance)
            matches_step2.push_back(matches[i][0]);
    }
    filter_ambiguous_train(matches_step2);

    #else

    cv::flann::Index index;
    index.build(f1.desc, cv::flann::HierarchicalClusteringIndexParams (32,cvflann::FLANN_CENTERS_RANDOM,1,10),cvflann::FLANN_DIST_HAMMING);
    cv::Mat indices,distances;
    index.knnSearch(f2.desc,indices,distances,2,cv::flann::SearchParams(15,0,true));
    assert(distances.type()==CV_32S);
    for(int i = 0; i < distances.rows; i++) {
        if(distances.at<int32_t>(i,0) < nn_match_ratio * float(distances.at<int32_t>(i,1))) {
            if ( (f1.und_kpts[indices.at<int32_t>(i,0)].octave-f2.und_kpts[i].octave)<2 ){
                cv::DMatch match;match.distance=distances.at<int32_t>(i,0);
                match.queryIdx=indices.at<int32_t>(i,0);
                match.trainIdx=i;
                matches.push_back(match);
            }
        }
    }
    filter_ambiguous_query(matches);

    #endif


    //check consistency
    _debug_exec(10,
    for(auto m:matches){
        assert(m.queryIdx<f1.desc.rows);
        assert(m.trainIdx<f2.desc.rows);
    });
    return matches;

    }

    vector<cv::DMatch> match_desc(const cv::Mat & desc1,const cv::Mat & desc2, double nn_match_ratio ){
    _debug_msg("Need improvementzzxxxxxxxxxxxxxxxxxxxXXxzzz!!",1);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector< vector<cv::DMatch> > matches;
    matcher->knnMatch(desc1, desc2, matches, 2);

    vector<cv::DMatch> matches_step2;
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matches_step2.push_back(matches[i][0]);
        }
    }



      filter_ambiguous_train(matches_step2);
      return matches_step2;

    }

    /**
    * @brief filter_matches
    * @param f1
    * @param f2
    * @param matches
    * @param method
    * @param result
    * @param ransacReprojThreshold
    * @param ip
    * @return
    */


    vector<cv::DMatch> filter_matches(const vector<cv::Point2f> &p1, const vector<cv::Point2f> &p2, const vector<cv::DMatch> &matches,  FILTER_METHOD method, cv::Mat &result,float ransacReprojThreshold){
    cv::Mat inlier_mask;
    vector<cv::DMatch> inlier_matches;
    if (method==FM_HOMOGRAPHY)
        result=cv::findHomography ( p1, p2, cv::RANSAC, ransacReprojThreshold, inlier_mask);
    else      {
        cerr<<"JKKJ"<<endl;
        result=cv::findFundamentalMat ( p1,p2,cv::FM_RANSAC,ransacReprojThreshold,0.99 ,inlier_mask );
    }

    if (!result.empty()){
        for(unsigned i = 0; i < matches.size(); i++)
            if(inlier_mask.at<uchar>(i))
                inlier_matches.push_back(matches[i]);

    }
    return inlier_matches;
    }


    vector<cv::DMatch> filter_matches(const vector<cv::KeyPoint> &f1, const vector<cv::KeyPoint> &f2, const vector<cv::DMatch> &matches,  FILTER_METHOD method, cv::Mat &result,float ransacReprojThreshold  ){
    //remove outliers by finding homography
    std::vector<cv::Point2f> p1, p2;
    p1.reserve(matches.size());
    p2.reserve(matches.size());
    for(auto m:matches){
        p1.push_back(  f1[ m.queryIdx].pt  );
        p2.push_back(  f2[ m.trainIdx].pt  );
    }
    return filter_matches(p1,p2,matches,method,result,ransacReprojThreshold);
    }


    double  reprj_error(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp, const cv::Mat &rt44, vector<cv::Point2f> *projections, vector<float> *errors){
    //    for (int i = 0; i < points2d.size(); ++i) {
    //        cout<<points2d[i].x<<", "<<points2d[i].y<<endl;
    //    }
    std::vector<cv::Point2f> prepj_;
    if (projections==0) projections=&prepj_;
    project(objPoints,imp.CameraMatrix,rt44,*projections);

    if (errors!=0) errors->resize(projections->size());
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<projections->size();i++){
        if ( !isnan(objPoints[i].x)){
            float err=cv::norm( points2d[i]-(*projections)[i]);
             if (errors) (*errors)[i]=err;
             sum+= err;
             nvalid++;
        }
    }
    return sum/double(nvalid);

    }

    double  reprj_error( const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp,const cv::Mat &rv,const cv::Mat &tv){

    std::vector<cv::Point2f> prepj;
    project(objPoints,imp.CameraMatrix,getRTMatrix(rv,tv),prepj);
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<prepj.size();i++){
        if ( !isnan(objPoints[i].x)){
             sum+= cv::norm( points2d[i]-prepj[i]);
            nvalid++;
        }
    }
    return sum/double(nvalid);
    }

    void undistortPoints(   vector<cv::Point2f> &points_io,const ImageParams &ip,vector<cv::Point2f> *out){
    std::vector<cv::Point2f> pout;
    if (out==0) out=&pout;

    assert(ip.CameraMatrix.type()==CV_32F);
    cv::undistortPoints ( points_io, *out,ip.CameraMatrix, ip.Distorsion);//results are here normalized. i.e., in range [-1,1]

    float fx=ip.CameraMatrix.at<float> ( 0,0 );
    float fy=ip.CameraMatrix.at<float> ( 1,1 );
    float cx=ip.CameraMatrix.at<float> ( 0,2 );
    float cy=ip.CameraMatrix.at<float> ( 1,2 );

    if (out==&pout){
        for ( size_t i=0; i<pout.size(); i++ ) {
            points_io[i].x=pout[i].x*fx+cx;
            points_io[i].y=pout[i].y*fy+cy;
        }
    }
    else{
        for ( size_t i=0; i<pout.size(); i++ ) {
            (*out)[i].x=(*out)[i].x*fx+cx;
            (*out)[i].y=(*out)[i].y*fy+cy;
        }
    }
    }

    double  reprj_error( const ImageParams &ip,const Frame &f1,const Frame &f2, const std::vector<cv::DMatch> &matches, const std::vector<cv::Point3f> &objPoints,std::vector<double> *repj_err,const cv::Mat &t21)throw(std::exception){
    if (f1.und_kpts.size()==0 || f2.und_kpts.size()==0) throw std::runtime_error("No undistorted key points");
    if (objPoints.size()!= matches.size()) throw std::runtime_error("object points and matches must have equal size");


    //compute the reproj err
    std::vector<cv::Point2f> prepj_0,prepj_1;
    cv::Mat rv[2],tv[2];
    if (!t21.empty()){
        rv[0]=cv::Mat::zeros(1,3,CV_32F);
        tv[0]=cv::Mat::zeros(1,3,CV_32F);
        getRTfromMatrix44(t21,rv[1],tv[1]);
    }
    else{
        rv[0]=f1.pose_f2g.getRvec();
        tv[0]=f1.pose_f2g.getTvec();
        rv[1]=f2.pose_f2g.getRvec();
        tv[1]=f2.pose_f2g.getTvec();
    }


    project(objPoints,ip.CameraMatrix,getRTMatrix(rv[0],tv[0]) ,prepj_0);
    project(objPoints,ip.CameraMatrix,getRTMatrix(rv[1],tv[1]) ,prepj_1);

    if (repj_err!=0) repj_err->resize(objPoints.size());
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<matches.size();i++){
        if (!isnan(objPoints[i].x)){
            const auto &m=matches[i];
            auto err1=cv::norm( f1.und_kpts[m.queryIdx].pt-prepj_0[i]);
            auto err2=cv::norm( f2.und_kpts[m.trainIdx].pt-prepj_1[i]);
            sum+=err1+err2 ;
            //cout<<err1<<" "<<err2<<endl;
            if(repj_err!=0)  repj_err->at(i)=(0.5*(err1+err2));
            nvalid+=2;
        }
        else repj_err->at(i)=std::numeric_limits<float>::max();
    }
    return sum/double(nvalid);

    }


    cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst,bool mbFixScale){
    auto ComputeCentroid=[](cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,CV_REDUCE_SUM);
        C = C/P.cols;
        for(int i=0; i<P.cols; i++)
            Pr.col(i)=P.col(i)-C;
    };

    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    //Create the P1 an P2 matrices
    cv::Mat P1(3,org.size(),CV_32F);
    cv::Mat P2(3,org.size(),CV_32F);
    for(size_t i=0;i<org.size();i++){
        P1.at<float>(0,i)=org[i].x;
        P1.at<float>(1,i)=org[i].y;
        P1.at<float>(2,i)=org[i].z;
        P2.at<float>(0,i)=dst[i].x;
        P2.at<float>(1,i)=dst[i].y;
        P2.at<float>(2,i)=dst[i].z;
    }


        // Step 1: Centroid and relative coordinates

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        if (norm(vec)<1e-7)return cv::Mat::eye(4,4,CV_32F);

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        cv::Mat mR12i(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale
        float ms12i;

        if(!mbFixScale)
        {
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            ms12i = nom/den;
        }
        else
            ms12i = 1.0f;

        // Step 7: Translation

        cv::Mat  mt12i(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
    //        return mT12i;

    //        // Step 8.2 T21

        cv::Mat mT21i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        cv::Mat tinv = -sRinv*mt12i;
        tinv.copyTo(mT21i.rowRange(0,3).col(3));
        return mT21i;
    }


    uint countCommonMapPoints(const Frame & f1, const Frame & f2)
    {
    uint ncommon = 0;
    auto idsF = f1.ids;

    if (f1.idx != f2.idx)
    {
       auto idsKF = f2.ids;
       for (uint i = 0; i < idsF.size(); i++)
           for (uint j = 0; j < idsKF.size(); j++)
           {
              if (idsF[i] != std::numeric_limits<uint32_t>::max() && idsF[i] == idsKF[j])
                  ncommon++;
           }
    }
    else
       ncommon = f1.ids.size();

    return ncommon;
    }



    void savePointsToPCD(const std::vector<cv::Point3f> &points, string filename, cv::Scalar color)
    {
    auto convertP3DtoV4=[](cv::Point3f p,cv::Scalar color){

        float fcolor;uchar *c=(uchar*)&fcolor;

        for(int i=0;i<3;i++)c[i]=color[i];
        return cv::Vec4f(p.x,p.y,p.z,fcolor);
    };

    vector<cv::Vec4f> pcdpoints;

    std::ofstream filePCD ( filename, std::ios::binary );
    if (!filePCD.is_open())
    {
      std::cerr << "Could not open file: "  << filename << std::endl;
    }

    // Prepare points
    pcdpoints.reserve(points.size());
    for (uint i = 0; i < points.size(); i++)
    {
      pcdpoints.push_back(convertP3DtoV4(points[i], color));
    }


    // Start PCD file
    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";


    filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));

    filePCD.close();
    }

    cv::Point2f project(const cv::Point3f &p3d,const cv::Mat &cameraMatrix_32f,const cv::Mat &RT44__32f){
    assert(RT44__32f.type()==CV_32F);
    assert(cameraMatrix_32f.type()==CV_32F);
    cv::Point3f res;
    const float *rt=RT44__32f.ptr<float>(0);
    res.x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
    res.y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
    res.z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
    //now, project
    const float *cam=cameraMatrix_32f.ptr<float>(0);
    cv::Point2f r2d;
    r2d.x= (cam[0]*res.x/res.z)+cam[2];
    r2d.y= (cam[4]*res.y/res.z)+cam[5];
    return r2d;
    }

    void project(const vector<cv::Point3f> &vp3d,const cv::Mat &cameraMatrix_32f,const cv::Mat &RT44__32f,vector<cv::Point2f> &p2dv){
    assert(RT44__32f.type()==CV_32F);
    assert(RT44__32f.total()==16);
    assert(cameraMatrix_32f.type()==CV_32F);
    assert(cameraMatrix_32f.total()==9);
    p2dv.resize(vp3d.size());
    const float *rt=RT44__32f.ptr<float>(0);
    const float *cam=cameraMatrix_32f.ptr<float>(0);
    //    cout<<"matched points: \n";
    for(size_t i=0;i<vp3d.size();i++){
        const auto &p3d=vp3d[i];
        float x,y,z;
        x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
        y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
        z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
        //now, project
        auto &p2d=p2dv[i];
        p2d.x= (cam[0]*x/z)+cam[2];
        p2d.y= (cam[4]*y/z)+cam[5];
    //        cout<<p2d.x<<", "<<p2d.y<<endl;

    }
    }
    //triangulation method in
    //@INPROCEEDINGS{Recker2012,
    //author={Recker, Shawn and Hess-Flores, Mauricio and Duchaineau, Mark A. and Joy, Kenneth I.},
    //booktitle={Applied Imagery Pattern Recognition Workshop (AIPR), 2012 IEEE},
    //title={Visualization of scene structure uncertainty in multi-view reconstruction},
    //year={2012},
    //pages={1-7},
    //doi={10.1109/AIPR.2012.6528216},
    //ISSN={1550-5219},}

    cv::Point3f angular_error_minimization(cv::Point3f point, const vector<se3> &poses_f2g){
    struct cam{
        cv::Point3f direction,position;
    };

    auto angular_gradient=[]( const vector<cam>& cameras , const cv::Point3f &point)
    {
        cv::Point3f g = cv::Point3f(0,0,0);
        for(auto const &c:cameras){
            cv::Point3f v = point - c.position;

            double denom2 = v.dot(v);
            double denom = sqrt(denom2);
            double denom15 = pow(denom2, 1.5);
            double vdotw = v.dot(c.direction);
            g.x += (-c.direction.x/denom) + ((v.x*vdotw)/denom15);
            g.y += (-c.direction.y/denom) + ((v.y*vdotw)/denom15);
            g.z += (-c.direction.z/denom) + ((v.z*vdotw)/denom15);
        }
        return g;
    };

    //create cameras
    vector<cam> cameras;
    for(auto p:poses_f2g){
        cv::Mat mPose=p.convert().inv();
        cam c;
        c.position =mPose*cv::Point3f(0,0,0);  //obtain camera center in global reference system
        auto v1=mPose*cv::Point3f(0,0,1);
        auto vd=v1-c.position;
        c.direction=  vd/cv::norm(vd);
         cameras.push_back(c);
      }


    double precision = 1e-25;
        cv::Point3f g_old;
      cv::Point3f x_old;
      cv::Point3f x_new =point;;
      cv::Point3f grad = angular_gradient(cameras,x_new);
      double epsilon = .001;
      double diff;
      int count = 150;
      do {
        x_old = x_new;
        g_old = grad;
        x_new = x_old - epsilon * g_old;
        grad = angular_gradient(cameras, x_new);
        cv::Point3f sk = x_new - x_old;
        cv::Point3f yk = grad - g_old;
        double skx = sk.x;
        double sky = sk.y;
        double skz = sk.z;
        diff = skx*skx+sky*sky+skz*skz;
        //Compute adaptive step size (sometimes get a divide by zero hence
        //the subsequent check)
        epsilon = diff/(skx*yk.x+sky*yk.y+skz*yk.z);
        epsilon = (epsilon != epsilon) ||
          (epsilon == numeric_limits<double>::infinity()) ? .001 : epsilon;
        --count;
      } while(diff > precision && count-- > 0);
      if(isnan(x_new.x ) || isnan(x_new.y ) || isnan(x_new.z )) {
        return point;
      }
      return  x_new;
    }


    ///两帧初始化
    cv::Mat  ARUCO_initialize(const std::vector<aruco::Marker> &markers1,
                                      const std::vector<aruco::Marker> &markers2,
                                      const ucoslam::ImageParams &cp, float markerSize,
                                      float  minCornerPixDist,float repj_err_Thres, float minDistance,
                                      std::map<uint32_t,se3> &_marker_poses)
    {

    const std::vector<aruco::Marker> &ms1_in=markers1;
    const std::vector<aruco::Marker> &ms2_in=markers2;

    //find matches of markers in both images
    vector< std::pair<uint32_t,uint32_t> > matches;

    for(size_t i=0;i<ms1_in.size();i++){
        for(size_t j=0;j<ms2_in.size();j++)
            if ( ms1_in[i].id==ms2_in[j].id) matches.push_back(make_pair(i,j));
    }
    if (matches.size()==0)return cv::Mat();

    //create vectors with matched elements so that the elements are in order in both ms1 and ms2
    std::vector<aruco::Marker> ms1,ms2;
    for(auto m:matches){
        ms1.push_back( ms1_in[m.first]);
        ms2.push_back( ms2_in[m.second]);
    }
    std::vector<cv::Point2f> p2d_v1,p2d_v2;
    for(auto m: ms1)p2d_v1.insert(p2d_v1.end(),m.begin(),m.end());
    for(auto m: ms2)p2d_v2.insert(p2d_v2.end(),m.begin(),m.end());

    ///compute average pixel distance to avoid computing from too near views
    float avrgPixDist=0;//std::numeric_limits<float>::max();
    for(size_t i=0;i<p2d_v1.size();i++)
    {
        avrgPixDist+= float(cv::norm(p2d_v1[i]-p2d_v2[i]));
    //        cout<<"("<<p2d_v1[i].x<<", "<<p2d_v1[i].y<<") - ";
    //        cout<<"("<<p2d_v2[i].x<<", "<<p2d_v2[i].y<<")"<<endl;
    }

    avrgPixDist/=float(p2d_v1.size());
    _debug_msg("avrgPixDist=" << avrgPixDist,10);
    //  cout<<"avrgPixDist="<<avrgPixDist<<endl;
    //too near?
    if (avrgPixDist< minCornerPixDist*cp.CamSize.width ) return cv::Mat();

    //compte the marker poses (both the good and other one)
     std::vector<std::vector<std::pair<cv::Mat,double>>> marker_poses_v1,marker_poses_v2;
     _debug_msg(cp.Distorsion,10);
    for(size_t i=0;i<ms1.size();i++){
    //        cout<<"Marker pose: "<<ms1[i].id<<endl;
        marker_poses_v1.push_back(IPPE::solvePnP_(markerSize,ms1[i],cp.CameraMatrix,cp.Distorsion));
        marker_poses_v2.push_back(IPPE::solvePnP_(markerSize,ms2[i],cp.CameraMatrix,cp.Distorsion));
    }


    ///BIT ADD
    double avg_theta = 0; int n = 0;
    for (size_t i = 0; i < marker_poses_v1.size(); ++i)
    {
        cv::Mat t1 = marker_poses_v1[i][0].first.rowRange(0,3).colRange(0,3);
        double x1 = t1.at<float>(0);
        double y1 = t1.at<float>(1);
        double z1 = t1.at<float>(2);
        double theta1 = (acos(z1/(sqrt(x1*x1+y1*y1+z1*z1)))*180)/3.1415926;
        cv::Mat t2 = marker_poses_v2[i][0].first.rowRange(0,3).colRange(0,3);
        double x2 = t2.at<float>(0);
        double y2 = t2.at<float>(1);
        double z2 = t2.at<float>(2);
        double theta2 = (acos(z2/(sqrt(x2*x2+y2*y2+z2*z2)))*180)/3.1415926;
        double delta_theta = abs(theta2-theta1);
        avg_theta += delta_theta;
        n++;
    }
    avg_theta /=n;
    cout<<"avg_theta = "<<avg_theta<<endl;
    if (avg_theta >= 2)
    {
        for (size_t i = 0; i < marker_poses_v1.size(); ++i)
        {
            cv::Mat T1a = marker_poses_v1[i][0].first;
            cv::Mat T2a = marker_poses_v1[i][1].first;
            for (size_t j = 0; j < marker_poses_v1.size(); ++j)
            {
                if (i == j) continue;
                cv::Mat T1b = marker_poses_v1[j][0].first;
                cv::Mat T2b = marker_poses_v1[j][1].first;
            }
        }
    }
    _debug_exec(10,
        for(size_t i=0;i<marker_poses_v1.size();i++)
             cout<<marker_poses_v1[i][1].second/marker_poses_v1[i][0].second<< "-" <<marker_poses_v2[i][1].second/marker_poses_v2[i][0].second<<endl;
    );

    auto get_marker_points=[](float ms){
        return vector<cv::Point3f>({ cv::Point3f(-ms/2.,ms/2.,0), cv::Point3f(ms/2.,ms/2.,0),
                    cv::Point3f(ms/2.,-ms/2.,0),cv::Point3f(-ms/2.,-ms/2.,0)});
    };




    //     auto repj_err=[&](const cv::Mat &rt_totest){
    //        vector<cv::Point3f> p3d;
    //        vector<double> errv;
    //        auto err=triangulate(p2d_v1,cp,cv::Mat::eye(4,4,CV_32F), p2d_v2,cp,rt_totest,p3d,&errv);
    //        return err;

    //    };

    ///定义误差函数
    auto repj_err=[&](const cv::Mat &rt_totest){
      //for each marker, get the 3d points in camera 2, and project to camera 1 using rt_totest
     double sumerr=0;
     for(size_t i=0;i<ms1.size();i++){
         vector<cv::Point3f> p3d=get_marker_points(markerSize);//3d points in marker ref system
         //move points to v2
         for(auto &p:p3d) p=marker_poses_v1[i][0].first*p; //move to v1
         for(auto &p:p3d) p=rt_totest*p; //move to v2
         //now, project and get the error
    //         cout<<"matched Marker id: "<<ms2[i].id<<endl;
         sumerr+=reprj_error( p3d, ms2[i],cp,cv::Mat::eye(4,4,CV_32F));
     }
     return sumerr/float(ms1.size());

    };

    //now, calculate all possible solutions and evaluate them
    struct pinfo{
        cv::Mat v1_c2m,v2_c2m;
        cv::Mat rt; // from 1 -> 2, i.e., from global -> 2
        double err;
        unsigned int c1, c2;
        int id;
    };
    vector<pinfo> sol_err;
    for(size_t i=0;i<ms1.size();i++){
        for(size_t j=0;j<2;j++){
            for(size_t k=0;k<2;k++){
                pinfo pi;
                pi.v1_c2m=marker_poses_v1[i][k].first;
                pi.v2_c2m=marker_poses_v2[i][j].first;
                pi.rt=   pi.v2_c2m*pi.v1_c2m.inv();

                pi.err=repj_err(pi.rt);
                pi.c1 = (unsigned int)j;
                pi.c2 = (unsigned int)k;
                pi.id = ms1[i].id;
    //                cout<<"pi.err"<<pi.err<<endl;
                if(!std::isnan(pi.err) && !std::isinf(pi.err)) sol_err.push_back(pi);
            }
        }
    }

    if (sol_err.size()==0)return cv::Mat();

    std::sort(sol_err.begin(),sol_err.end(),[](const pinfo&a,const pinfo &b){return a.err<b.err;});
     _debug_exec(10, for(auto pi:sol_err)cout<<"pi.err="<<pi.err<<" ";cout<<endl;);
     //has a low repoj error?
        for (int l = 0; l < sol_err.size(); ++l) {
            cv::Mat t = sol_err[l].rt.rowRange(0,3).col(3);
            cout<<"id: "<<sol_err[l].id<<", err: "<<sol_err[l].err<<", t = "<<norm(t)
                <<", "<<sol_err[l].c1<<" : "<<sol_err[l].c2<<endl;
        }
     auto best_sol=sol_err.front();
     //repj_err_Thres = 2.5
     //误差判断1：重投影小于2.5
    //     cout<<"best err "<<best_sol.err<<endl;
     if( best_sol.err < repj_err_Thres)
     {
         //has enough distance between the views??
         auto curBaseLine=cv::norm(best_sol.rt.rowRange(0,3).colRange(3,4));
         cout<<"curBaseLine "<<curBaseLine<<endl;
         if (curBaseLine>=minDistance || avg_theta > 2.5)//基线条件2 >0.1
         {
            ///恢复Marker位姿
    //            cout<<"Init minDistance "<<minDistance<<endl;
             for(size_t i=0;i<ms1.size();i++){
                 double ratio1 = marker_poses_v1[i][0].second / marker_poses_v1[i][1].second;
    //                 double ratio2 = marker_poses_v2[i][0].second / marker_poses_v2[i][1].second;
    //                 cout<<"ratio1 = "<<ratio1<<endl;
    //                 cout<<"ratio2 = "<<ratio2<<endl;
                 //solution a, project and get error
                 auto p3d=get_marker_points(markerSize);
                 //move points to v2 and then back to v1
                 cv::Mat pose1 = best_sol.rt.inv()*marker_poses_v2[i][0].first;
                 for(auto &p:p3d) p=pose1*p;
                 //finall, get repr err in v1
                 auto err1= reprj_error( p3d, ms1[i],cp,cv::Mat::eye(4,4,CV_32F));
                 //repeat in inverse order
                 p3d=get_marker_points(markerSize);
                 //move points to v1 and then back to v2
                 cv::Mat pose2=best_sol.rt*marker_poses_v1[i][0].first;
                 for(auto &p:p3d) p=pose2*p;
                 //finall, get repr err in v1
                 auto err2= reprj_error( p3d, ms2[i],cp,cv::Mat::eye(4,4,CV_32F));
                 _debug_msg("err1_2:"<<err1<<" "<<err2,10);
                 //get the best solution
                 if (err1<err2){
                      _marker_poses.insert({ms1[i].id,  se3(pose1) });
                 }
                 else
                     _marker_poses.insert({ms1[i].id, se3(marker_poses_v1[i][0].first)});


             }
             cout<<"两帧初始化成功!"<<endl;
             return best_sol.rt;
         }

     }
     return   cv::Mat();
    }

    ///求Marker位姿
    ///marker_views: 同一个Marker在不同帧中的像素坐标
    cv::Mat ARUCO_bestMarkerPose(const vector<aruco::Marker> &marker_views,
                                const vector<se3> &frameposes_f2g,
                                const ucoslam::ImageParams &cp,
                                float markerSize)
    {

        assert( marker_views.size()>=2);

        //create all possible solutions from frame to global
        struct poseinfo{
            double err=0;
            cv::Mat pose_g2m;
        };
        vector<poseinfo> solutions;

        ///Step1：计算同一个Marker在多帧下的位姿
        for(size_t i=0;i<marker_views.size();i++){
            auto ss=  IPPE::solvePnP_(markerSize,marker_views[i],cp.CameraMatrix,cp.Distorsion);
            poseinfo pi;

            pi.pose_g2m= frameposes_f2g[i].convert().inv() * ss[0].first; // g2f  * f2m
            solutions.push_back(pi);
            //            pi.pose_g2m= frameposes_f2g[i].convert().inv() * ss[1].first; // g2f  * f2m
            //            solutions.push_back(pi);
        }
        //now, compute the reproj error and get the best
        auto p3d=ucoslam::Marker::get3DPoints(se3(),markerSize,false);
        ///Step2：计算每个位姿在多帧下的总投影平均误差，选择误差最小的
        for( auto &sol:solutions)
        {
            //        sol.err=std::numeric_limits<float>::min();
            for(size_t i=0;i<frameposes_f2g.size();i++)
                sol.err+= reprj_error(p3d,marker_views[i],cp,frameposes_f2g[i].convert()*sol.pose_g2m);
            //sol.err=std::max( sol.err,  reprj_error(p3d,marker_views[i],cp,frameposes_f2g[i].convert()*sol.pose_g2m));
            sol.err/=float(frameposes_f2g.size());
            cout<<"new Marker err "<<sol.err<<endl;
        }
        std::sort(solutions.begin(),solutions.end(),[](const poseinfo&a,const poseinfo&b){return a.err<b.err;});
        _debug_msg("repj  err="<<solutions.front().err<<" "<<solutions.back().err,10);
        return solutions.front().pose_g2m;
    }

    ///BIT ADD
    /// 1, marker_views: the unknown marker's observed in each frame
    /// 2, vvMarker_Pose: valid marker pose in frame
    /// 3, vvMarkers: valide markers' observed in each frame
    cv::Mat ARUCO_bestMarkerPose_BIT(const vector<aruco::Marker> &marker_views,
                                 vector<vector<se3> > vvMarker_poses,
                                 vector<uint32_t> vframes,
                                 vector<uint32_t> vframesFseqId,
                                 const vector<vector<aruco::Marker> > vvMarkers,
                                 const vector<se3> &frameposes_f2g,
                                 const ucoslam::ImageParams &cp,
                                 float markerSize,
                                 vector<double> vmarkers_thetas,
                                 float theta_thredhold)
    {

//        assert( marker_views.size()>=2);
        if (marker_views.size() < 3) return cv::Mat();
        //create all possible solutions from frame to global
        struct poseinfo{
            double err=0.0;
            cv::Mat pose_g2m;
            double theta_err = 0.0;
            int si = 0;
            uint32_t frameId=0;
            double fov = 0.0;
        };
        vector<poseinfo> solutions;

        ///Step1：计算同一个Marker在多帧下的位姿
        for(size_t i=0;i<marker_views.size();i++){
            auto ss=  IPPE::solvePnP_(markerSize,marker_views[i],cp.CameraMatrix,cp.Distorsion);
            poseinfo pi1, pi2;

            pi1.pose_g2m= frameposes_f2g[i].convert().inv() * ss[0].first; // g2f  * f2m
            pi1.si = 0;
            pi1.frameId = vframesFseqId[i];
            pi1.fov = vmarkers_thetas[i];
            solutions.push_back(pi1);
            pi2.pose_g2m= frameposes_f2g[i].convert().inv() * ss[1].first; // g2f  * f2m
            pi2.si = 1;
            pi2.frameId = vframesFseqId[i];
            pi2.fov = vmarkers_thetas[i];
//            solutions.push_back(pi2);
        }
        //now, compute the reproj error and get the best
        ///Step2：计算每个位姿在多帧下的总投影平均误差，选择误差最小的
        for( auto &sol:solutions)
        {
            ///******
            double totalSum = 0.0;
            cv::Mat Rm = sol.pose_g2m.rowRange(0,3).colRange(0,3);
            for (size_t i=0; i<vvMarkers.size(); i++)
            {
                int n = 0;
                double sum = 0.0;
                vector<aruco::Marker> vMarkers = vvMarkers[i];
                vector<se3> vMarker_poses = vvMarker_poses[i];
                ///1
                auto ss_1=  IPPE::solvePnP_(markerSize,marker_views[i],cp.CameraMatrix,cp.Distorsion);
                for (size_t j=0; j < vMarkers.size(); j++)
                {
                    n++;
                    cv::Mat Rj = vMarker_poses[j].getRotation3x3();
                    ///2
                    auto ss_2=  IPPE::solvePnP_(markerSize,vMarkers[j],cp.CameraMatrix,cp.Distorsion);
                    double minDelta = std::numeric_limits<double>::max();
                    for (int k = 0; k < 2; ++k)
                    {
                        cv::Mat R1 = ss_1[k].first.rowRange(0,3).colRange(0,3);
                        for (int l = 0; l < 2; ++l)
                        {
                            cv::Mat R2 = ss_2[l].first.rowRange(0,3).colRange(0,3);
                            cv::Mat deltaR = Rm.t()*Rj-R1.t()*R2;
                            double delta = cv::norm(deltaR);
                            if (delta < minDelta) minDelta = delta;
                        }
                    }
                    sum += minDelta;
                }
                sum = sum/(float)n;
                totalSum += sum;
            }
            sol.theta_err = totalSum;
        }
        std::sort(solutions.begin(),solutions.end(),
                [](const poseinfo&a,const poseinfo&b){return a.theta_err<b.theta_err;});
        for (auto sol : solutions)
        {
            cout<<"frameId = "<<sol.frameId<<",theta: "<<sol.fov<<", si = "<<sol.si<<", err: "<<sol.theta_err<<endl;
        }

        /*if (solutions.front().theta_err < 0.25)
            return solutions.front().pose_g2m;
        else
            return cv::Mat();*/

        _debug_msg("repj  err="<<solutions.front().err<<" "<<solutions.back().err,10);
        if (solutions[0].frameId != solutions[1].frameId)
        {
            double distR = cv::norm(solutions[0].pose_g2m.rowRange(0,3).colRange(0,3) -
                                    solutions[1].pose_g2m.rowRange(0,3).colRange(0,3));
            cout<<"distR: "<<distR<<endl;
            if (distR < 0.1) return solutions[0].pose_g2m;
        }
        else
        {
            double distR = cv::norm(solutions[0].pose_g2m.rowRange(0,3).colRange(0,3) -
                                    solutions[1].pose_g2m.rowRange(0,3).colRange(0,3));
            cout<<"same id distR: "<<distR<<endl;

            double distR1 = cv::norm(solutions[0].pose_g2m.rowRange(0,3).colRange(0,3) -
                                    solutions[2].pose_g2m.rowRange(0,3).colRange(0,3));
            double distR2 = cv::norm(solutions[1].pose_g2m.rowRange(0,3).colRange(0,3) -
                                    solutions[2].pose_g2m.rowRange(0,3).colRange(0,3));
            cout<<"dist1: "<<distR1<<", dist2: "<<distR2<<endl;
            if (distR1 < distR2) return solutions[0].pose_g2m;
            else return solutions[1].pose_g2m;
        }
        /*if (solutions.front().fov < theta_thredhold)//solutions.front().theta_err < 0.5 &&
        {
            return solutions.front().pose_g2m;
        }
        else
            return cv::Mat();*/
    }

    vector<int> outlierFiltering(const vector<float> &data,int ntimes,float *mean_out,float *stddev_out){
    //calculate mean and dev
    float sum=0,sq_sum=0,nvalid=0;
    for(auto datum:data){
        if (!isnan(datum)){
            sum+=datum;
            sq_sum+= datum*datum;
            nvalid++;
        }
    }
    double mean = sum / nvalid;
    double variance = sq_sum / nvalid - mean * mean;
    double stddev=sqrt(variance);


    float thres=mean+2*stddev;

    vector<int> outliers;
    outliers.reserve(float(data.size())*0.2);
    for(size_t i=0;i<data.size();i++){
        if (!isnan(data[i])){
            if (data[i]>thres)
                outliers.push_back( i);
        }
        else  outliers.push_back( i);
    }
    if (mean_out!=0)*mean_out=mean;
    if (stddev_out!=0) *stddev_out=stddev;
    return outliers;
    }
    cv::Mat getFastProjectK( const  cv::Mat &CameraMatrix,const cv::Mat &RT){
    assert(CameraMatrix.type()== RT.type() &&  RT.type() ==CV_32F);
    //precomputed projection matrix lhs
    //  |fx 0   cx |   |1 0 0 0|
    //  |0  fy  cy | * |0 1 0 0|
    //  |0  0   1  |   |0 0 1 0|
    float m[16]={CameraMatrix.at<float>(0,0),0,CameraMatrix.at<float>(0,2),0,
                 0,CameraMatrix.at<float>(1,1),CameraMatrix.at<float>(1,2),0,
                 0,0,1,0,
                 0,0,0,1};
    cv::Mat   LPm (4,4,CV_32F,m);
    return LPm*RT;
    }

    cv::Mat computeF12(const cv::Mat &RT1,const cv::Mat &CameraMatrix1, const cv::Mat &RT2,const cv::Mat &_CameraMatrix2)
    {

      cv::Mat K2;

      if (_CameraMatrix2.empty())
          K2=CameraMatrix1;
      else K2=_CameraMatrix2;
      cv::Mat R1w = RT1(cv::Range(0,3),cv::Range(0,3));//pKF1->GetRotation();
      cv::Mat t1w = RT1(cv::Range(0,3),cv::Range(3,4));// ;pKF1->GetTranslation();
      cv::Mat R2w = RT2(cv::Range(0,3),cv::Range(0,3));//pKF2->GetRotation();
      cv::Mat t2w = RT2(cv::Range(0,3),cv::Range(3,4));//pKF2->GetTranslation();

      cv::Mat R12 = R1w*R2w.t();
      cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

      float d[9]={0, -t12.at<float>(2), t12.at<float>(1),
                  t12.at<float>(2),               0,-t12.at<float>(0),
                  -t12.at<float>(1),  t12.at<float>(0),              0};

      cv::Mat t12x(3,3,CV_32F,d);// = SkewSymmetricMatrix(t12);




      cv::Mat F12=CameraMatrix1.t().inv()*t12x*R12*K2.inv();
      return F12;
    }



    int   triangulate_(const cv::Mat &RT, std::vector<cv::KeyPoint> &kp1,std::vector<cv::KeyPoint> &kp2,  const cv::Mat &CameraMatrix, vector<cv::Point3f> &vP3D,  const vector<float> &scaleFactors,vector<bool> &vbGood,float maxChi2 )
    {
      auto  Triangulate=[](const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
      {
          cv::Mat A(4,4,CV_32F);

          A.row(0) = kp1.x*P1.row(2)-P1.row(0);
          A.row(1) = kp1.y*P1.row(2)-P1.row(1);
          A.row(2) = kp2.x*P2.row(2)-P2.row(0);
          A.row(3) = kp2.y*P2.row(2)-P2.row(1);

          cv::Mat u,w,vt;
          cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
          x3D = vt.row(3).t();
          if(x3D.at<float>(3)==0) return false;

          x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
          return true;
      };

      vector<float> invScaleFactors;
      for(auto f:scaleFactors)           invScaleFactors.push_back(1./(f*f));
      cv::Mat R=RT(cv::Range(0,3),cv::Range(0,3));
      cv::Mat t=RT(cv::Range(0,3),cv::Range(3,4));

      // Calibration parameters
      const float fx1 = CameraMatrix.at<float>(0,0);
      const float fy1 = CameraMatrix.at<float>(1,1);
      float invfy1=1./fy1;
      float invfx1=1./fx1;
      float invfy2=invfy1;
      float invfx2=invfx1;

      const float cx1 = CameraMatrix.at<float>(0,2);
      const float cy1 = CameraMatrix.at<float>(1,2);

      const float cx2 = cx1;
      const float cy2 = cy1;

      vP3D.resize(kp1.size());
      vbGood=vector<bool>(kp1.size(),false);

      // Camera 1 Projection Matrix K[I|0]
      cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
      CameraMatrix.copyTo(P1.rowRange(0,3).colRange(0,3));

      cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

      // Camera 2 Projection Matrix K[R|t]
      cv::Mat P2(3,4,CV_32F);
      R.copyTo(P2.rowRange(0,3).colRange(0,3));
      t.copyTo(P2.rowRange(0,3).col(3));
      P2 = CameraMatrix*P2;

      cv::Mat O2 = -R.t()*t;

      int nGood=0;

      cv::Point3f invalid3d(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
      for(size_t i=0;i<kp1.size();i++)
      {
          vP3D[i]=invalid3d;
          // Check parallax between rays
          cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1[i].pt.x-cx1)*invfx1, (kp1[i].pt.y-cy1)*invfy1, 1.0);
          cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2[i].pt.x-cx2)*invfx2, (kp2[i].pt.y-cy2)*invfy2, 1.0);

          cv::Mat ray1 = xn1;
          cv::Mat ray2 = R*xn2;
          const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

          if (!( cosParallaxRays>0 &&  cosParallaxRays<0.9998) )
              continue;

          cv::Mat p3dC1;

          if (!Triangulate(kp1[i].pt,kp2[i].pt,P1,P2,p3dC1)) continue;

          if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
              continue;

          // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          if(p3dC1.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          cv::Mat p3dC2 = R*p3dC1+t;

          if(p3dC2.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check reprojection error in first image
          float im1x, im1y;
          float invZ1 = 1.0/p3dC1.at<float>(2);
          im1x = fx1*p3dC1.at<float>(0)*invZ1+cx1;
          im1y = fy1*p3dC1.at<float>(1)*invZ1+cy1;

          float chi2 =  invScaleFactors[kp1[i].octave] *(im1x-kp1[i].pt.x)*(im1x-kp1[i].pt.x)+(im1y-kp1[i].pt.y)*(im1y-kp1[i].pt.y);
          //          float chi2 =  (im1x-p1[i].x)*(im1x-p1[i].x)+(im1y-p1[i].y)*(im1y-p1[i].y);

          if(chi2>maxChi2)
              continue;

          // Check reprojection error in second image
          float im2x, im2y;
          float invZ2 = 1.0/p3dC2.at<float>(2);
          im2x = fx1*p3dC2.at<float>(0)*invZ2+cx1;
          im2y = fy1*p3dC2.at<float>(1)*invZ2+cy1;

          chi2 = invScaleFactors[kp2[i].octave]  * (im2x-kp2[i].pt.x)*(im2x-kp2[i].pt.x)+(im2y-kp2[i].pt.y)*(im2y-kp2[i].pt.y);

          if(chi2>maxChi2)
              continue;


          vbGood[i]=true;
          vP3D[i] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
          nGood++;
      }


      return nGood;
    }

    double computeDeltaTheta(vector<aruco::Marker> markers1,
                            vector<aruco::Marker> markers2,
                            const ucoslam::ImageParams &cp,
                            float markerSize)
    {
      vector< std::pair<uint32_t,uint32_t> > matches;

      for(size_t i=0;i<markers1.size();i++)
      {
          for(size_t j=0;j<markers2.size();j++)
              if ( markers1[i].id==markers2[j].id)
                  matches.push_back(make_pair(i,j));
      }
      std::vector<aruco::Marker> ms1,ms2;
      cout<<"matches : ";
      for(auto m:matches)
      {
          cout<<markers1[m.first].id<<", ";
          ms1.push_back( markers1[m.first]);
          ms2.push_back( markers2[m.second]);
      }
      cout<<endl;

      std::vector<std::vector<std::pair<cv::Mat,double>>> marker_poses_v1,marker_poses_v2;
      for(size_t i=0;i<ms1.size();i++){
          marker_poses_v1.push_back(IPPE::solvePnP_(markerSize,ms1[i],cp.CameraMatrix,cp.Distorsion));
          marker_poses_v2.push_back(IPPE::solvePnP_(markerSize,ms2[i],cp.CameraMatrix,cp.Distorsion));
      }
      double avg_theta = 0; int n = 0;
      for (size_t i = 0; i < marker_poses_v1.size(); ++i)
      {
          cv::Mat t1 = marker_poses_v1[i][0].first.rowRange(0,3).col(3);
          double z1 = t1.at<float>(2);
          double theta1 = (acos(z1/cv::norm(t1))*180)/3.1415926;
          cout<<"theta1 = "<<theta1;
          cv::Mat t2 = marker_poses_v2[i][0].first.rowRange(0,3).col(3);
          double z2 = t2.at<float>(2);
          double theta2 = (acos(z2/cv::norm(t2))*180)/3.1415926;
          cout<<",theta2 = "<<theta2<<endl;
          double delta_theta = abs(theta2-theta1);
          avg_theta += delta_theta;
          n++;
      }
      avg_theta /=n;
    }

}
