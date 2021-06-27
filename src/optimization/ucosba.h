#ifndef UCOSBA_H
#define UCOSBA_H

#include <vector>
#include <iostream>
#include "sparselevmarq.h"

namespace ucosba {

//A 2d point
struct Point2f{
    Point2f(){}
    Point2f(float _x,float _y):x(_x),y(_y){}
    float x,y;
};
//A 3d point
struct Point3f{
    Point3f(){}
    Point3f(float _x,float _y,float _z):x(_x),y(_y),z(_z){}
    float x,y,z;
};


/**
 * @brief The Se3Transform struct  Represents a se3 transform as a 4x4 matrix
 */
struct Se3Transform {
public:
    //matrix-point  multiplication
    inline  Point3f operator*(const Point3f &p)const{    return  Point3f(  data[0]*p.x +data[1]*p.y +data[2]*p.z+data[3],data[4]*p.x +data[5]*p.y +data[6]*p.z+data[7],data[8]*p.x +data[9]*p.y +data[10]*p.z+data[11]);}

    inline void set(float *ext_data){memcpy(data,ext_data,16*sizeof(float));}
    float data[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};//4x4 transform matrix
    inline Se3Transform & operator=(const Se3Transform &m){memcpy(data,m.data,16*sizeof(float));return *this;}
    //------------
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    Se3Transform operator *(const Se3Transform &m)const;
    float & operator ()(uint32_t r,uint32_t c){ return data[ r*4+c];}
    const float & operator ()(uint32_t r,uint32_t c)const{ return data[ r*4+c];}
    //transform into a 6 dof representation (rx,ry,rz,tx,ty,tz)
    std::vector<float> toRTVec()const;
    void fromRTVec(float rx,float ry,float rz,float tx,float ty,float tz);
    inline void fromRTVec(std::vector<float> rv){assert(rv.size()==6);return fromRTVec(rv[0],rv[1],rv[2],rv[3],rv[4],rv[5]);}
    friend std::ostream &operator<<(std::ostream &str,const Se3Transform &ci);

};


/**
 * @brief The PointProjection struct  represents the projection of a point in one of the camera  views
 */


struct PointProjection{
    Point2f p2d; //2d projection of the point
    int camIndex; //camera index where the pointin the vector BA_Data::cameraSetPoses
    float weight=1;//an optional weight to ponder the influence of this projection
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);


    float chi2=0;//chi2 measure associated to this (is calcualated internally)
};


struct PointProjection2{
    Point2f p2d; //2d projection of the point
    int camIndex;
    int pointIndex; //point index
    float weight=1;//an optional weight to ponder the influence of this projection
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);


    float chi2=0;//chi2 measure associated to this (is calcualated internally)
};

/**
 * @brief The PointInfo struct represents the information available about a point
 */
struct PointInfo{
     Point3f p3d;//3d estimated location
    std::vector<PointProjection> pointProjections;//projections of the point
    bool fixed=false;//indicates whethe the point must be fixed during the optimization
    uint32_t id;//an id you can set.Ids for cameras and points are managed indepedently. So they can repeat



    // -----------------------------------
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
    int64_t sol_index=-1;//index in the solution vector
};


struct CameraInfo{
    Se3Transform pose;
    bool fixed=false;//indicates whether the point should remain fixed during optimization
    uint32_t id;//an id you can set. Ids for cameras and points are managed indepedently. So they can repeat


    //------------
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
    int64_t sol_index=-1;//index in the solution vector
};
/**
 * @brief The CameraParams struct represent the intrinsic camera parameters
 */
struct CameraParams{
    int width=0,height=0;//image dimensions
    /** Camera coefficients
     */
    float fx=-1,fy=-1,cx=-1,cy=-1;
    /**
     * Distortion coefficients (k1,k2,p1,p2,k3) uses either 5 parameters
     */
    float dist[5]={0,0,0,0,0};


    //indicates if this object is valid
    bool isValid()const{return width*height>0 && fx!=-1 && fy!=-1 && cx!=-1 && cy!=-1;}
    //has distortion?
    bool hasDistortion()const{return dist[0]!=0  || dist[1]!=0 || dist[2]!=0 || dist[3]!=0 || dist[4]!=0;}
    //-----------------------------------
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
};

/**
 * @brief The SBA_Data struct is the main data structure for storing the Bundle adjusment Data

 */
struct SBA_Data{

    //Camera parameters
    CameraParams camParams;
    //set of camera poses
    std::vector<CameraInfo> cameraSetInfo;
    //set of points and its projections
    std::vector<PointInfo> pointSetInfo;



    //computes the sum of the chi2
    //The chi of each projection is established in PointProjection::chi2()  (pointSetInfo vector) so that you can evaluate it
    double computeChi2();


    void saveToFile(std::string path)const;
    void readFromFile(std::string path);


    //-----------------------------
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

};

/**
 * @brief The OptimizationParams struct contains the basic optimization parameters
 */
struct OptimizationParams{

    OptimizationParams(){}
    int maxIterations=100;//maximum number of iterations
    float minGlobalChi2=1e-3;//If global chi2 falls below this, stops optimization
    float minStepChi2=1e-4;//if Chi2 decrease between consecutive iterations falls below this, stop
    bool verbose=false;
};




/**
 * @brief The BasicOptimizer class does the optimization of points and camera positions. It does not include the camera parameter optimization
 */
class BasicOptimizer{

public:

    /** Optimizes the data. Please notice that the points projections will be set to its undistorted version
     * @brief optimize perform the optimization
     * @param data in/out data to be optimized
     * @param params optimization parameters
     */
    void optimize(SBA_Data &data,OptimizationParams params=OptimizationParams());

private:
    OptimizationParams _params;


    void  error(const ucoslam::SparseLevMarq<double>::eVector &sol,ucoslam::SparseLevMarq<double>::eVector &err);
    void jacobian(const ucoslam::SparseLevMarq<double>::eVector &sol, Eigen::SparseMatrix<double> &);


    uint32_t _errorSize;//size of the error vector
    SBA_Data *_data;

    inline double hubberMono(double e){
        if (e <= 5.991)   return  e; // inlier
        else   return  4.895303872*sqrt(e) - 5.991; // outlier
    }

    inline double getHubberMonoWeight(double SqErr,double Information){
         return sqrt(hubberMono(Information * SqErr)/ SqErr);
    }
    void fromSolutionToSBAData(const ucoslam::SparseLevMarq<double>::eVector &sol,SBA_Data &BAD);

    std::vector< Eigen::Triplet<double> > triplets;

    std::vector< std::vector<PointProjection2>  > cam_points;
    ucoslam::SparseLevMarq<double> optimizer;
    ucoslam::SparseLevMarq<double>::eVector solution;


     void callback(const ucoslam::SparseLevMarq<double>::eVector &) ;

     int startCamPoseSols;
};



};


#endif
