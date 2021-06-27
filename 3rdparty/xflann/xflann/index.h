/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2016  Rafael Muñoz Salinas (rmsalinas@uco.es). All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/
#ifndef _XFLANN_INDEX_H
#define _XFLANN_INDEX_H
#include "exports.h"
#include "types.h"
#include <memory>
#include <vector>
namespace XFLANN_API xflann{

class cpu_x86;
/** Class representing an index. Internally dedices which index to use according to the parameters used
 */
class Index{

public:

    /**Empty constructor
    */
    Index();

    /**
     * @brief build the index using the passed features and the indicated params.
     * @param features input data to create index
     * @param params indexing params. Either of type KdTreeParamSet or  KdMeansParamSet
     * @param desc_name optional parameter indicating the descriptor name so it can be saved to disk along the rest of data
     */
    Index(Matrix  features, const ParamSet &params)throw(std::runtime_error);
    /**
     * @brief build the index using the passed features and the indicated params.
     * @param features input data to create index
     * @param params indexing params. Either of type KdTreeParamSet or  KdMeansParamSet
     * @param desc_name optional parameter indicating the descriptor name so it can be saved to disk along the rest of data
     */
    void  build(Matrix  features, const ParamSet &params)throw(std::runtime_error);
    /**
     * @brief performs search (knn or  radius)
     * @param features query elements
     * @param nn number of neighbors to be search
     * @param indices output matrix. Must be already reserved with size features.rows x nn . Must be of integer type: XFLANN_X32S.
     * @param distances output matrix. Must be already reserved with size features.rows x nn . Must be of float type: XFLANN_X32F.
     * @param search_params optional search params (not used at this point)
     * @param Tcpu optional cpu info. It can be used to disable certain optimizations(avx,mmx or sse). Used normally in debug mode to test the speed of this method againts other without considering the hardware optimizations.
     */
    void search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu_x86> Tcpu=std::shared_ptr<cpu_x86>())throw(std::exception);



#if (defined XFLANN_OPENCV  ||  defined __OPENCV_CORE_HPP__ || defined OPENCV_CORE_HPP)
    //specialized version for opencv
    inline void search(const cv::Mat &features, int nn,  cv::Mat &indices,  cv::Mat &distances, const ParamSet& search_params,std::shared_ptr<cpu_x86> Tcpu=std::shared_ptr<cpu_x86>());

#endif

    /**
     * @brief saveToFile Saves this index from a file
     * @param filepath output file path
     */
    void saveToFile(const std::string &filepath)throw(std::runtime_error);
    /**
     * @brief readFromFile reads the index from a file generated with saveToFile
     * @param filepath input file path
     */
    void readFromFile(const std::string &filepath)throw(std::runtime_error);
    /**
     * @brief saveToStream saves the index to a binary stream
     * @param str
     */
    void toStream(std::ostream &str)throw(std::runtime_error);
    /**
     * @brief readFromStream reads the index from a binary stream
     * @param str
     */
    void fromStream(std::istream &str)throw(std::runtime_error);

    /**Clears this object from data*/
    void clear();

private:
    void _search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu_x86> Tcpu=std::shared_ptr<cpu_x86>())throw(std::exception);

    std::shared_ptr< impl::IndexImpl  > _impl;
    void parallel_search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu_x86> Tcpu=std::shared_ptr<cpu_x86>())throw(std::exception);

    //sort results
    template<typename DType>
    void sort(Matrix  indices,Matrix distances){
        for(int r=0;r<indices.rows;r++){
            auto idx_ptr=indices.ptr<int>(r);
            auto dist_ptr=distances.ptr<DType>(r);
            for(int i=0;i<indices.cols-1;i++){
                if ( idx_ptr[i]!=-1)
                    for(int j=i+1;j<indices.cols;j++){
                        if ( dist_ptr[i]>dist_ptr[j]){
                            std::swap(dist_ptr[i],dist_ptr[j]);
                            std::swap(idx_ptr[i],idx_ptr[j]);
                        }
                    }
            }
        }
    }
};
#if (defined XFLANN_OPENCV  ||  defined __OPENCV_CORE_HPP__ || defined OPENCV_CORE_HPP)
    //specialized version for opencv
void Index::search(const cv::Mat &features, int nn,  cv::Mat &indices,  cv::Mat &distances, const ParamSet& search_params,std::shared_ptr<cpu_x86> Tcpu){
    indices.create(features.rows,nn,CV_32S);
    if ( features.type()==CV_8U)
        distances.create(features.rows,nn,CV_32S);
    else
        distances.create(features.rows,nn,CV_32F);
    _search(features,nn,indices,distances,search_params,Tcpu);

}

#endif
}
#endif
