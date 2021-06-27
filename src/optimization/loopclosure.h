#ifndef LoopClosure_H
#define LoopClosure_H

#include <opencv2/core.hpp>
#include <vector>
#include <map>
namespace ucoslam{
namespace loopclosure{

struct lse3;
//matrix internally used. Can be converted to opencv if desired
//you must set the data in the vector m. It is homogeneous matrix so last row must be {0 0 0 1}
struct lMat{
        float m[16];


        inline lMat(){}
        inline lMat(const lse3 &rtv);
        lMat inv();
        lMat operator*(const lMat &b);
        friend std::ostream &operator<<(std::ostream &str,const lMat &m);

         inline lMat(const cv::Mat &cvm){
            memcpy(m,cvm.ptr<float>(0),16*sizeof(float));
        }
        operator cv::Mat ()const{cv::Mat cvm(4,4,CV_32F);memcpy(cvm.ptr<float>(0),m,16*sizeof(float));return cvm;}
 
};

/**
 *
 * @brief loopClosurePathOptimization_cv
 * @param edges vector with edges between the pair of nodes
 * @param IdClosesLoopA First element closing the loop. This is the one that is really start of optimization. Its pose is twice: once in optimPoses, and anotherone in expectedPose
 * @param IdClosesLoopB Second element closing the loop
 * @param expectedPose Pose expected for the node IdClosesLoopA after optimization.
 * @param optimPoses Set of intial poses for the nodes. This is an input/output map.
 */

void  loopClosurePathOptimization_cv(const std::vector<std::pair<uint32_t,uint32_t> > &edges, uint32_t IdClosesLoopA,uint32_t IdClosesLoopB,cv::Mat expectedPose, std::map<uint32_t, cv::Mat> &optimPoses);
void  loopClosurePathOptimization(const std::vector<std::pair<uint32_t,uint32_t> > &edges,uint32_t IdClosesLoopA,uint32_t IdClosesLoopB,const lMat &expectedPose, std::map<uint32_t, lMat> &optimPoses);


}
}
#endif

