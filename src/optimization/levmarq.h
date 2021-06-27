#ifndef ucoslam_LevMarq_H
#define ucoslam_LevMarq_H
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include <functional>
#include <iostream>
#include <cmath>
#include <omp.h>
#include <ctime>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>
namespace ucoslam{
//Sparse Levenberg-Marquardt method for general problems
//Inspired in
//@MISC\{IMM2004-03215,
//    author       = "K. Madsen and H. B. Nielsen and O. Tingleff",
//    title        = "Methods for Non-Linear Least Squares Problems (2nd ed.)",
//    year         = "2004",
//    pages        = "60",
//    publisher    = "Informatics and Mathematical Modelling, Technical University of Denmark, {DTU}",
//    address      = "Richard Petersens Plads, Building 321, {DK-}2800 Kgs. Lyngby",
//    url          = "http://www.ltu.se/cms_fs/1.51590!/nonlinear_least_squares.pdf"
//}
template<typename T>
class   LevMarq{
public:


    typedef   Eigen::Matrix<T,Eigen::Dynamic,1>  eVector;
    typedef   std::function<void(const eVector  &, eVector &)> F_z_x;
    typedef   std::function<void(const eVector  &,  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &)> F_z_J;
    LevMarq();
    /**
    * @brief Constructor with parms
    * @param maxIters maximum number of iterations of the algoritm
    * @param minError to stop the algorithm before reaching the max iterations
    * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
    * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
    * @param der_epsilon increment to calculate the derivate of the evaluation function
    * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
    */
    LevMarq(int maxIters,T minError,T min_step_error_diff=0,T tau=1 ,T der_epsilon=1e-3);

 /**
 * @brief setParams
 * @param maxIters maximum number of iterations of the algoritm
 * @param minError to stop the algorithm before reaching the max iterations
 * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
 * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
 * @param der_epsilon increment to calculate the derivate of the evaluation function
 * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
 */
    void setParams(int maxIters,T minError,T min_step_error_diff=0,T tau=1 ,T der_epsilon=1e-3);

    /**
 * @brief solve  non linear minimization problem ||F(z)||, where F(z)=f(z) f(z)^t
 * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
 * @param f_z_x evaluation function  f(z)=x
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : x :  output. Data must be returned in T
 * @param f_J  computes the jacobian of f(z)
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : J :  output. Data must be returned in T
 * @return final error
 */
    T solve(  eVector  &z, F_z_x , F_z_J)throw (std::exception);
/// Step by step solve mode


    /**
     * @brief init initializes the search engine
     * @param z
     */
    void init(eVector  &z, F_z_x )throw (std::exception);
    /**
     * @brief step gives a step of the search
     * @param f_z_x error evaluation function
     * @param f_z_J Jacobian function
     * @return error of current solution
     */
    bool step(  F_z_x f_z_x , F_z_J  f_z_J)throw (std::exception);
    bool step(  F_z_x f_z_x)throw (std::exception);
    /**
     * @brief getCurrentSolution returns the current solution
     * @param z output
     * @return error of the solution
     */
    T getCurrentSolution(eVector  &z)throw (std::exception);
    /**
     * @brief getBestSolution sets in z the best solution up to this moment
     * @param z output
     * @return  error of the solution
     */
    T getBestSolution(eVector  &z)throw (std::exception);

  /**  Automatic jacobian estimation
 * @brief solve  non linear minimization problem ||F(z)||, where F(z)=f(z) f(z)^t
 * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
 * @param f_z_x evaluation function  f(z)=x
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : x :  output. Data must be returned in T
 * @return final error
 */
    T solve(  eVector  &z, F_z_x )throw (std::exception);
    //to enable verbose mode
    bool & verbose(){return _verbose;}

//sets a callback func call at each step
    void setStepCallBackFunc(std::function<void(const eVector  &)> callback){_step_callback=callback;}
//sets a function that indicates when the algorithm must be stop. returns true if must stop and false otherwise
    void setStopFunction( std::function<bool(const eVector  &)> stop_function){_stopFunction=stop_function;}

    void  calcDerivates(const eVector & z ,  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &sJ,  F_z_x f_z_x);
private:
    int _maxIters;
    T _minErrorAllowed,_der_epsilon,_tau,_min_step_error_diff;
    bool _verbose;
    //--------
    eVector curr_z,x64;
    T currErr,prevErr,minErr ;
    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>  I,J;
    T mu,v;
    std::function<void(const eVector  &)> _step_callback;
    std::function<bool(const eVector  &)> _stopFunction;

    void  add_missing_diagonal_elements( Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &M)throw (std::exception);
    void  get_diagonal_elements_refs_and_add( Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &M,std::vector<T*> &d_refs,T add_val)throw (std::exception);
    void  mult(const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &lhs, const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &rhs,Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &res);

};
template<typename T>
LevMarq<T>::LevMarq(){
    _maxIters=1000;_minErrorAllowed=0;_der_epsilon=1e-3;_verbose=false;_tau=1;v=5;_min_step_error_diff=0;
 }
/**
* @brief Constructor with parms
* @param maxIters maximum number of iterations of the algoritm
* @param minError to stop the algorithm before reaching the max iterations
* @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
* @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
* @param der_epsilon increment to calculate the derivate of the evaluation function
* step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
*/
template<typename T>

LevMarq<T>::LevMarq(int maxIters,T minError,T min_step_error_diff,T tau ,T der_epsilon ){
    _maxIters=maxIters;_minErrorAllowed=minError;_der_epsilon=der_epsilon;_verbose=false;_tau=tau;v=5;_min_step_error_diff=min_step_error_diff;
 }

/**
* @brief setParams
* @param maxIters maximum number of iterations of the algoritm
* @param minError to stop the algorithm before reaching the max iterations
* @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
* @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
* @param der_epsilon increment to calculate the derivate of the evaluation function
* step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
*/
template<typename T>
void LevMarq<T>::setParams(int maxIters,T minError,T min_step_error_diff,T tau ,T der_epsilon){
    _maxIters=maxIters;
    _minErrorAllowed=minError;
    _der_epsilon=der_epsilon;
    _tau=tau;
    _min_step_error_diff=min_step_error_diff;

}




template<typename T>
void  LevMarq<T>:: calcDerivates(const eVector & z ,  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & J,  F_z_x f_z_x)
{
//#pragma omp parallel for
    for (int i=0;i<z.rows();i++) {
        eVector zp(z),zm(z);
        zp(i)+=_der_epsilon;
        zm(i)-=_der_epsilon;
        eVector xp,xm;
        f_z_x( zp,xp);
        f_z_x( zm,xm);
        eVector dif=(xp-xm)/(2.f*_der_epsilon);
        J.middleCols(i,1)=dif;
    }
}


template<typename T>
T  LevMarq<T>:: solve(  eVector  &z, F_z_x f_z_x)throw (std::exception){
return solve(z,f_z_x,std::bind(&LevMarq<T>::calcDerivates,this,std::placeholders::_1,std::placeholders::_2,f_z_x));
}
template<typename T>
bool  LevMarq<T>:: step(  F_z_x f_z_x)throw (std::exception){
return step(f_z_x,std::bind(&LevMarq<T>::calcDerivates,this,std::placeholders::_1,std::placeholders::_2,f_z_x));
}

template<typename T>
void LevMarq<T>::init(eVector  &z, F_z_x f_z_x )throw (std::exception){
curr_z=z;
I.resize(z.rows(),z.rows());
I.setIdentity();
f_z_x(curr_z,x64);
// std::cerr<<x64.transpose()<<std::endl;
minErr=currErr=prevErr=x64.cwiseProduct(x64).sum();
J.resize(x64.rows(),z.rows());
mu=-1;


}

template<typename T>
void LevMarq<T>::get_diagonal_elements_refs_and_add( Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &M,std::vector<T*> &refs,T add_val)throw (std::exception){
refs.resize(M.cols());
//now, get their references and add mu
for (int k=0; k<M.outerSize(); ++k)
for ( typename Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>::InnerIterator it(M,k); it; ++it)
    if (it.row()== it.col())     {refs[it.row()]= &it.valueRef(); *refs[it.row()]+=add_val;}
}



template<typename T>
void LevMarq<T>::add_missing_diagonal_elements(Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &M)throw (std::exception){
  std::vector<bool> diag(M.rows(),false);
  for (int k=0; k<M.outerSize(); ++k)
     for (  typename Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>::InnerIterator it(M,k); it; ++it)
          if (it.row()== it.col())     diag[it.row()]=true;
  //and add them
  for(size_t i=0;i<diag.size();i++)   if (!diag[i]) M.insert(i,i) =0;

}


#define splm_get_time(a,b) std::chrono::duration_cast<std::chrono::duration<T>>(a-b).count()
template<typename T>
bool LevMarq<T>::step( F_z_x f_z_x, F_z_J f_J)throw (std::exception){

auto t1= std::chrono::high_resolution_clock::now();
f_J(curr_z,J);
Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> Jt=J.transpose();
auto t2= std::chrono::high_resolution_clock::now();
Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> JtJ=Jt*J;
auto t3= std::chrono::high_resolution_clock::now();
eVector  B=-Jt*x64;
auto t4= std::chrono::high_resolution_clock::now();

if(mu<0){//first time only
    T maxv=std::numeric_limits<T>::lowest();
    for(int i=0;i<JtJ.cols();i++)
        if ( JtJ(i,i)>maxv) maxv=JtJ(i,i);
    mu=maxv*_tau;
}


T gain=0;
std::vector<T*> refs;
int ntries=0;
bool isStepAccepted=false;
do{
    //add dumping factor to JtJ.
#if 0 //very efficient in any case, but particularly if initial dump does not produce improvement and must reenter
    if(refs.size()==0){//first time into the do
        add_missing_diagonal_elements(JtJ);
        get_diagonal_elements_refs_and_add(JtJ,refs,mu);
    }
    else for(size_t i=0;i<refs.size();i++)    *refs[i]+= mu-prev_mu;//update mu
    prev_mu=mu;
    Eigen::SimplicialLDLT<Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> > chol(JtJ);  // performs a Cholesky
#else  //less efficient, but easier to understand
    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> A=JtJ+I*mu;
     eVector delta= A.llt().solve(B);

#endif
    eVector  estimated_z=curr_z+delta;
    //compute error
    f_z_x(estimated_z,x64);
    auto err=x64.cwiseProduct(x64).sum();
    auto L=0.5*delta.transpose()*((mu*delta) - B);
    gain= (err-prevErr)/ L(0,0) ;
     //get gain
    if (gain>0 && ((err-prevErr)<0)){
        mu=mu*std::max(T(0.33),T(1.-pow(2*gain-1,3)));
        v=2.f;
        currErr=err;
        curr_z=estimated_z;
        isStepAccepted=true;
    }
    else{ mu=mu*v; v=v*5;}

}while(gain<=0 && ntries++<5);

if (_verbose) std::cout<<std::setprecision(5) <<"Curr Error="<<currErr<<" AErr(prev-curr)="<<prevErr-currErr<<" gain="<<gain<<" dumping factor="<<mu<<std::endl;
//    //check if we must move to the new position or exit
if ( currErr<prevErr)
std::swap ( currErr,prevErr );


if (_verbose) {std::cout<<" transpose="<<splm_get_time(t2,t1)<<" mult1="<< splm_get_time(t3,t2)<<" mult2="<< splm_get_time(t4,t3) <<std::endl;
          // std::cerr<<"solve="<<T(t4-t3)/T(CLOCKS_PER_SEC)<<std::endl;
}
return isStepAccepted;

}


template<typename T>
T  LevMarq<T>:: getCurrentSolution(eVector  &z)throw (std::exception){

z=curr_z;
return currErr;
}
template<typename T>
T  LevMarq<T>::solve( eVector  &z, F_z_x  f_z_x, F_z_J f_J)throw (std::exception){
prevErr=std::numeric_limits<T>::max();
init(z,f_z_x);

if( _stopFunction){
    do{
        step(f_z_x,f_J);
        if (_step_callback) _step_callback(curr_z);
    }while(!_stopFunction(curr_z));

}
else{
    //intial error estimation
    int mustExit=0;
    for ( int i = 0; i < _maxIters && !mustExit; i++ ) {
        if (_verbose)std::cerr<<"iteration "<<i<<"/"<<_maxIters<< "  ";
        bool isStepAccepted=step(f_z_x,f_J);
        //check if we must exit
        if ( currErr<_minErrorAllowed ) mustExit=1;
         if( fabs( prevErr -currErr)<=_min_step_error_diff  || !isStepAccepted) mustExit=2;
        //exit if error increment
        if (currErr<prevErr  )mustExit=3;
        //            if (  (prevErr-currErr)  < 1e-5 )  mustExit=true;
        if (_step_callback) _step_callback(curr_z);
    }

//    std::cout<<"Exit code="<<mustExit<<std::endl;
}
z=curr_z;
return currErr;

}
}

#endif
