#ifndef _ucoslam_MinMaxBags_H
#define _ucoslam_MinMaxBags_H
#include "heap.h"
namespace ucoslam{
/** Class that represent where you can only add X elements and only these with min values are kept.
 * If the number of elements introduced is greater than the limit, the bag only kepts the lowest values
 */
template<typename T>
class MinBag{
    size_t _maxSize=0;
    ucoslam::Heap<T,std::greater<T>> heap;
public:

    MinBag(int maxSize=-1) {setMaxSize(maxSize);}
    void setMaxSize(int maxSize) {_maxSize=maxSize;}
    inline void push(const T  &v){
        assert(_maxSize>0);
        if( heap.size()>=_maxSize){
            if ( heap.array[0] > v) {
                heap.pop();
                heap.push(v);
            }
        }
        else
            heap.push(v);
    }
    inline T pop(){return heap.pop();}
    inline bool empty()const{return heap.empty();}
};

/** Class that represent where you can only add X elements and only these with max values are kept.
 * If the number of elements introduced is greater than the limit, the bag only kepts the highest values
 */
template<typename T>
class MaxBag{
    size_t _maxSize=0;
    ucoslam::Heap<T,std::less<T>> heap;
public:

    MaxBag(int maxSize=-1) {setMaxSize(maxSize);}
    void setMaxSize(int maxSize) {_maxSize=maxSize;}
    inline void push(const T  &v){
        assert(_maxSize>0);
        if( heap.size()>= _maxSize){
            if ( heap.array[0] < v) {
                heap.pop();
                heap.push(v);
            }
        }
        else
            heap.push(v);
    }
    inline T pop(){return heap.pop();}
    inline bool empty()const{return heap.empty();}
};
}

#endif
