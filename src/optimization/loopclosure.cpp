
#include <opencv2/core.hpp>
#include <vector>
#include <map>
#include <set>
#include "sparselevmarq.h"
#include "loopclosure.h"
using namespace std;
namespace ucoslam 
{
    namespace loopclosure 
    {
        //given the vector (rx,ry,rz,tx,ty,tz) obtains the resulting matrix
        template<typename T>
        inline void createRTMatrix(const T *RxyzTxyz,T *matres){

            T nsqa=RxyzTxyz[0]*RxyzTxyz[0] + RxyzTxyz[1]*RxyzTxyz[1] + RxyzTxyz[2]*RxyzTxyz[2];
            T a=std::sqrt(nsqa);
            T i_a=a?1./a:0;
            T rnx=RxyzTxyz[0]*i_a;
            T rny=RxyzTxyz[1]*i_a;
            T rnz=RxyzTxyz[2]*i_a;
            T cos_a=cos(a);
            T sin_a=sin(a);
            T _1_cos_a=1.-cos_a;
            matres[0]=cos_a+rnx*rnx*_1_cos_a;
            matres[1]=rnx*rny*_1_cos_a- rnz*sin_a;
            matres[2]=rny*sin_a + rnx*rnz*_1_cos_a;
            matres[3]=RxyzTxyz[3];

            matres[4]=rnz*sin_a +rnx*rny*_1_cos_a;
            matres[5]= cos_a+rny*rny*_1_cos_a;
            matres[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
            matres[7]=RxyzTxyz[4];

            matres[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
            matres[9]= rnx*sin_a + rny*rnz*_1_cos_a;
            matres[10]=cos_a+rnz*rnz*_1_cos_a;
            matres[11]=RxyzTxyz[5];

            matres[12]=matres[13]=matres[14]=0;
            matres[15]=1;

        }

        //100x faster

        template<typename T>
        void invTMatrix(T *M,T *Minv){

            Minv[0]=M[0];
            Minv[1]=M[4];
            Minv[2]=M[8];
            Minv[4]=M[1];
            Minv[5]=M[5];
            Minv[6]=M[9];
            Minv[8]=M[2];
            Minv[9]=M[6];
            Minv[10]=M[10];

            Minv[3] = -  ( M[3]*Minv[0]+M[7]*Minv[1]+M[11]*Minv[2]);
            Minv[7] = -  ( M[3]*Minv[4]+M[7]*Minv[5]+M[11]*Minv[6]);
            Minv[11]= -  ( M[3]*Minv[8]+M[7]*Minv[9]+M[11]*Minv[10]);
            Minv[12]=Minv[13]=Minv[14]=0;
            Minv[15]=1;

        }
        //10x faster
        inline void matmul(const float *a,const float *b,float *c){

            c[0]= a[0]*b[0]+ a[1]*b[4]+a[2]*b[8];
            c[1]= a[0]*b[1]+ a[1]*b[5]+a[2]*b[9];
            c[2]= a[0]*b[2]+ a[1]*b[6]+a[2]*b[10];
            c[3]= a[0]*b[3]+ a[1]*b[7]+a[2]*b[11]+a[3];

            c[4]= a[4]*b[0]+ a[5]*b[4]+a[6]*b[8];
            c[5]= a[4]*b[1]+ a[5]*b[5]+a[6]*b[9];
            c[6]= a[4]*b[2]+ a[5]*b[6]+a[6]*b[10];
            c[7]= a[4]*b[3]+ a[5]*b[7]+a[6]*b[11]+a[7];

            c[8]=  a[8]*b[0]+ a[9]*b[4]+a[10]*b[8];
            c[9]=  a[8]*b[1]+ a[9]*b[5]+a[10]*b[9];
            c[10]= a[8]*b[2]+ a[9]*b[6]+a[10]*b[10];
            c[11]= a[8]*b[3]+ a[9]*b[7]+a[10]*b[11]+a[11];
            c[12]=c[13]=c[14]=0;
            c[15]=1;

        }
        //64x faster than using opencv method
        //converts a 4x4 transform matrix to rogrigues and translation vector rx,ry,rz,tx,ty,tz

        inline void Mat44ToRTVec(const float *M44,float *rvec){

            double theta, s, c;
            cv::Point3d r( M44[9] - M44[6], M44[2] - M44[8] , M44[4]  - M44[1]);

            s = std::sqrt((r.x*r.x + r.y*r.y + r.z*r.z)*0.25);
            c = (M44[0]  +M44[5]  +M44[10] - 1)*0.5;
            c = c > 1. ? 1. : c < -1. ? -1. : c;
            theta = acos(c);

            if( s < 1e-5 )
            {
                double t;

                if( c > 0 ) r.x=r.y=r.z=0;
                else
                {
                    t = (M44[0]  + 1)*0.5;
                    r.x = std::sqrt(std::max(t,0.));
                    t = (M44[5] + 1)*0.5;
                    r.y = std::sqrt(std::max(t,0.))*(M44[1] < 0 ? -1. : 1.);
                    t = (M44[10] + 1)*0.5;
                    r.z = std::sqrt(std::max(t,0.))*(M44[2]  < 0 ? -1. : 1.);
                    if( fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (M44[6]  > 0) != (r.y*r.z > 0) )
                        r.z = -r.z;
                    theta /=  sqrt(r.x*r.x+r.y*r.y+r.z*r.z);
                    r *= theta;
                }


            }
            else
            {
                double vth = 1/(2*s);
                vth *= theta;
                r *= vth;
            }

            rvec[0]=r.x;
            rvec[1]=r.y;
            rvec[2]=r.z;
            rvec[3]=M44[3];
            rvec[4]=M44[7];
            rvec[5]=M44[11];
        }

        struct lse3{
            float rt[6];
            inline lse3(){}
            inline lse3(const lMat &lmat){
                Mat44ToRTVec(lmat.m,rt);
            }
            inline lse3(const cv::Mat &lmat){
                Mat44ToRTVec(lmat.ptr<float>(0),rt);
            }

            operator cv::Mat ()const{cv::Mat m(4,4,CV_32F); createRTMatrix(rt,m.ptr<float>(0));return m;}


        };

        lMat lMat::inv(){//inverts
            lMat res;
            invTMatrix(m,res.m);
            return res;
        }
        lMat lMat::operator*(const lMat &b){
            lMat res;
            matmul(m,b.m,res.m);
            return res;
        }
        lMat::lMat(const lse3 &rtv){
            createRTMatrix<float>(rtv.rt,m);
        }

        std::ostream &operator<<(std::ostream &str,const lMat &m){
            int idx=0;
            str<<"[";
            for(int i=0;i<4;i++){
                if (i!=0)str<<" ";
                for(int j=0;j<4;j++){
                    str<<m.m[idx++];
                    if (j!=3) str<<" ";
                }
                if (i!=3) str<<";"<<std::endl;
            }
            str<<"]"<<std::endl;
            return str;
        }

        inline void derivErrorEdge6i(const lse3 & vi, float *m_Mj_inv, float *m_dT, float * derivV, int idx)
        {
            const float delta = 1e-3;
            const float invDelta = 1.0 / (2.f*delta);
            float resV1[6], resV2[6];
            float Mi[16],rMat[16], mul1[16];
            // Delta case: positive
            lse3 vii(vi);
            vii.rt[idx] += delta;


            createRTMatrix(vii.rt, Mi);
            matmul( m_dT, Mi, mul1 );
            matmul(mul1, m_Mj_inv , rMat   );
            Mat44ToRTVec(rMat , resV2);



            // Delta case: negative
            lse3 vii2(vi);
            vii2.rt[idx] -= delta;

            createRTMatrix(vii2.rt, Mi);
            matmul( m_dT, Mi, mul1 );
            matmul(mul1, m_Mj_inv , rMat   );
            Mat44ToRTVec(rMat , resV1);


            // Dif
            for (int k = 0; k < 6; k++)
                derivV[k] = (resV2[k] - resV1[k])*invDelta; // /(2.f*delta);

        }

        inline void derivErrorEdge6j(float * m_Mi, const lse3 & vj, float *m_dT, float * v_derivV, int idx)
        {
            const float delta = 1e-3;
            const float invDelta = 1.0 / (2.f*delta);
            //float deriv = 0;
            float resV1[6], resV2[6];
            float Mj[16],Mj_inv[16],mul1[16],rMat[16];

            // Delta case: positive
            lse3 vjj(vj);
            vjj.rt[idx] += delta;
            {
                createRTMatrix(vjj.rt, Mj);
                invTMatrix(Mj , Mj_inv );
                matmul( m_dT, m_Mi, mul1 );
                matmul(mul1, Mj_inv , rMat  );
                Mat44ToRTVec(rMat , resV2);
            }
            // Delta case: negative
            lse3 vjj2(vj);
            vjj2.rt[idx] -= delta;
            {
                createRTMatrix(vjj2.rt, Mj);
                invTMatrix(Mj , Mj_inv );
                matmul( m_dT, m_Mi, mul1 );
                matmul(mul1, Mj_inv , rMat  );
                Mat44ToRTVec(rMat , resV1);
            }

            // Dif
            for (int k = 0; k < 6; k++)
                v_derivV[k] = (resV2[k] - resV1[k])*invDelta; // /(2.f*delta);
        }


        //@param edges set of edges of the essential graph plus the loopclosure edge that is at the end
        //@param frame that is to be inserted and creates the loop closure
        //@param expectedPose of the frame
        //THe loop closure edge is the last one in edges vector
        /*
        * in/out: edges_in
        * in: IdClosesLoopA,当前帧id
        * in: IdClosesLoopB,闭环帧id
        * in: expectedPose,当前帧的位姿(无漂移)
        * in/out: optimPoses,本质图中帧的原始位姿（有漂移）
        * out: optimPoses,本质图优化后的位姿（无漂移）
        */
        void  loopClosurePathOptimization(const vector<pair<uint32_t,uint32_t> > &edges_in, 
                                uint32_t IdClosesLoopA,uint32_t IdClosesLoopB, 
                                const lMat &expectedPose, std::map<uint32_t, lMat> &optimPoses)
        {

            //    cout<<"B"<<endl;
            //    cout<<optimPoses[IdClosesLoopB]<<endl;
            //    cout<<"A"<<endl;
            //    cout<<optimPoses[IdClosesLoopA]<<endl;
            //    cout<<"expected"<<endl;
            //    cout<<expectedPose<<endl;
            vector<pair<uint32_t,uint32_t> > edges=edges_in;
            // Useful variables
            uint32_t maxIdx = 0;
            //if last edge not added, then addit
            bool found=false;
            for(auto e:edges)
                if (e.first==IdClosesLoopA && e.second==IdClosesLoopB){found=true;break;}

            if (!found)
                edges.push_back({IdClosesLoopA,IdClosesLoopB});
            auto nEdges = edges.size();

            // Set of nodes
            std::set<uint32_t> theNodes;

            // Relative position of pairs of cameras, given edges
            std::vector<lMat> dTs;

            ///***计算本质图中与当前帧的相对位姿
            for (auto edge:edges)
            {
                auto i = edge.first;
                auto j = edge.second;

                lMat Ti, Tj;

                // Current pair of poses

                Ti = optimPoses.at(i) ;
                Tj = optimPoses.at(j) ;


                if ( i==IdClosesLoopA&& j==IdClosesLoopB)  // Use expectedPose
                {
                    Ti = expectedPose;
                }

                /// Relative pose计算原始相对位姿
                lMat dTij = Tj * Ti.inv();

                dTs.push_back(dTij);

                // Add nodes to set
                theNodes.insert(i);
                theNodes.insert(j);

                if (i > maxIdx)
                    maxIdx = i;
                if (j > maxIdx)
                    maxIdx = j;

                //std::cout << "(" << i << ", " << j << ") ";
            }

            auto nNodes = theNodes.size();
        //   std::cout << "Number of nodes found: " << nNodes << std::endl;


        //    std::cout << "Loop idx: " << IdClosesLoopB << std::endl;

            map<uint32_t,int> optimIdx;//[maxIdx+1]; // Where parameters start for each frame in optim vector
            ///开始优化,优化本质图的位姿
            // Prepare optimization
            ucoslam::SparseLevMarq<float> levm;          // TODO: choose optim params
            ucoslam::SparseLevMarq<float>::Params parsLM;
            parsLM.verbose=false;
            parsLM.tau=0.01;
            parsLM.min_step_error_diff = float(nNodes)* (1e-04/400.f); //1e-4;             // TODO: to be tuned
            parsLM.maxIters = 100; // TODO: to be tuned
            levm.setParams(parsLM);
            ucoslam::SparseLevMarq<float>::eVector sol;

            sol.resize(6*nNodes);          // Six params per keyframe (pose): skip fixed cam

            // Init solutions with current poses: skip first one [fixed]
            int solIx = 0;
            for (auto n:theNodes)
            {
                //auto Ti = TheFrameSet[*it].pose_f2g.rt;
                assert(optimPoses.count(n));
                lse3 pose=optimPoses.at(n);
                for (int k = 0; k < 6; k++)
                    sol(solIx*6 + k) = pose.rt[k];
                optimIdx[n] = solIx*6;
                solIx++;
            }


            //------------------------------------------------------------------
            auto errorPoseGraphFix_M44VeryFast = [&](const ucoslam::SparseLevMarq<float>::eVector &sol_, ucoslam::SparseLevMarq<float>::eVector &err)
            {
                //        ucoslam::ScopeTimer errTimer("errorPoseGraphFix"); // DEBUG
                int nerrors=6;//select solution
                err.resize(nEdges*nerrors);//
                struct usedMat{
                    float m[16];
                    bool valid;
                };

                std::map< uint32_t,usedMat> vSi,vSi_inv;
                for(auto n: theNodes) vSi[n].valid=false;
                vSi_inv=vSi;

                float mul1[16],mul2[16];
                // Loop over edges
                for (unsigned int eix = 0; eix < nEdges; eix++)
                {
                    auto ii = edges[eix].first;
                    auto jj = edges[eix].second;

                    // Convert to matrix if not yet computed
                    if ( !vSi[ii].valid){
                        createRTMatrix(sol_.data()+(optimIdx[ii]), vSi[ii].m);
                        vSi[ii].valid=true;
                    }
                    if ( !vSi_inv[jj].valid){
                        if (!vSi[jj].valid ){
                            createRTMatrix(sol_.data()+(optimIdx[jj]), vSi[jj].m);
                            vSi[jj].valid=true;
                        }
                        invTMatrix(vSi[jj].m,vSi_inv[jj].m);
                        vSi_inv[jj].valid=true;
                    }
                    matmul( dTs[eix].m, vSi[ii].m , mul1 );
                    matmul( mul1, vSi_inv[jj].m , mul2 );
                    //version using only 6 error elements
                    auto eidx=eix*nerrors;
                    Mat44ToRTVec(mul2 ,mul1);
                    for(int i=0;i<6;i++)
                        err(eidx + i) = mul1[i];

                }

            };

            //---------------------------------------------------------------
            // JACOBIAN approximation
            auto JacobianApprox6=[&](const ucoslam::SparseLevMarq<float>::eVector  & sol_,  Eigen::SparseMatrix<float> & jac)
            {
                int errSize = 6;         // Number of componets per error
                auto nErrores = nEdges * errSize;
                jac.resize(nErrores, sol_.size());
                vector<Eigen::Triplet<float> > triplets;
                triplets.reserve(nErrores);

                // Create temporal vector with extended solution
                // Derivative computation
                float * derivV = new float [errSize];		

                for (uint32_t eix = 0; eix < nEdges; eix++)
                {
                    auto ii = edges[eix].first;
                    auto jj = edges[eix].second;


                    lse3 lSi,lSj;
                    memcpy(&lSi.rt,sol_.data()+(optimIdx[ii]),6*sizeof(float));
                    memcpy(&lSj.rt,sol_.data()+(optimIdx[jj]),6*sizeof(float));


                    // Derivatives for first node in edge



                    if (ii!=IdClosesLoopB){//do not move IdClosesLoopB
                        lMat Mj(lSj);
                        lMat Mj_inv=Mj.inv();
                        for (int vix = 0; vix < 6; vix++)
                        {
                            //derivErrorEdge12(Si, Sj, dTs[eix], derivV, vix);
                            derivErrorEdge6i(lSi, Mj_inv.m, dTs[eix].m, derivV, vix);
                            //std::cout << derivV << std::endl;
                            auto ci = optimIdx[ii]+vix;
                            int initPos = eix*errSize;
                            for (auto k = 0; k < errSize; k++)
                            {
                                if (fabs(derivV[k]) < 1e-4) // Skip if very small
                                    continue;

                                //auto v = derivV[k];
                                if (ci >= 0)
                                    triplets.push_back( Eigen::Triplet<float>(initPos + k,ci, derivV[k]) );

                            } // k
                        } // vix


                    }
                    if (jj!=IdClosesLoopB){//do not move IdClosesLoopB
                        lMat Mi(lSi);// = Si.convert();
                        // Repeat for second node in edge
                        for (int vix = 0; vix < 6; vix++)
                        {
                            derivErrorEdge6j(Mi.m, lSj, dTs[eix].m, derivV, vix);
                            //std::cout << derivV << std::endl;

                            int initPos = eix*errSize;
                            auto cj = optimIdx[jj]+vix;

                            for (auto k = 0; k < errSize; k++)
                            {
                                if (cj >= 0)
                                    triplets.push_back( Eigen::Triplet<float>(initPos + k,cj, derivV[k]) );
                            } // k
                        } // vix

                    }
                }

                // Output
                jac.setFromTriplets(triplets.begin(),triplets.end());

                // Release memory
                delete [] derivV;

            };

            /* ================ END ERROR FUNCTION ================ */

            levm.solve(sol, std::bind(errorPoseGraphFix_M44VeryFast, std::placeholders::_1, std::placeholders::_2), std::bind(JacobianApprox6, std::placeholders::_1, std::placeholders::_2));


            // Generate output
            //std::map<uint32_t, se3> optimPoses;
            for (auto it = theNodes.begin(); it != theNodes.end(); ++it)
            {

                if (*it != IdClosesLoopB)
                {
                    lse3 pose;
                    for (int k = 0; k < 6; k++)
                        pose.rt[k] = sol(optimIdx[*it] + k);
                    optimPoses[*it] = pose;
                }
            }
        //    cout<<"B"<<endl;

        //    cout<<optimPoses[IdClosesLoopB]<<endl;
        //    cout<<"A"<<endl;

        //    cout<<optimPoses[IdClosesLoopA]<<endl;
        //    cout<<"expected"<<endl;
        //    cout<<expectedPose<<endl;

        }

        /*
        * in/out: edges
        * in: IdClosesLoopA,当前帧id
        * in: IdClosesLoopB,闭环帧id
        * in: expectedPose,当前帧的位姿(无漂移)
        * in: optimPoses,本质图中帧的原始位姿（有漂移）
        */
        void  loopClosurePathOptimization_cv(const vector<pair<uint32_t,uint32_t> > &edges, 
                    uint32_t IdClosesLoopA,uint32_t IdClosesLoopB,cv::Mat expectedPose, 
                    std::map<uint32_t, cv::Mat> &optimPoses)
        {
            ///本质图原始位姿
            std::map<uint32_t, lMat> l_optimPoses;
            lMat l_expectedPose=expectedPose;

            for(auto m:optimPoses)
                l_optimPoses[m.first]=m.second;
            
            //IdClosesLoopA: 当前帧id
            //IdCloseLoopB： 闭环帧Id
            //edges: 本质图关系
            loopClosurePathOptimization(edges,IdClosesLoopA,IdClosesLoopB,l_expectedPose,l_optimPoses);

            for(auto m:l_optimPoses)
                optimPoses[m.first]=m.second;

        }
    }
}
