#ifndef ucoslam_TSQueue_H
#define ucoslam_TSQueue_H
#include <vector>
#include <mutex>
#include <condition_variable>
namespace ucoslam{
//A thread safe queue to implement producer consumer

template<typename T>
class TSQueue
{
public:
    void push(T val) {
        while (true) {
            std::unique_lock<std::mutex> locker(mu);
            cond.wait(locker, [this](){return buffer_.size() < size_;});
            if (buffer_.size()==1)buffer_[0]=val;
            else buffer_.push_back(val);
            locker.unlock();
            cond.notify_all();
            return;
        }
    }
    void  pop(T&v) {
        while (true)
        {
            std::unique_lock<std::mutex> locker(mu);
            cond.wait(locker, [this](){return buffer_.size() > 0;});
            v= buffer_.back();
            buffer_.pop_back();
            locker.unlock();
            cond.notify_all();
            return ;
        }
    }

    bool empty(){
        std::unique_lock<std::mutex> locker(mu);
        return buffer_.size()==0;
    }

    TSQueue() {}


public:
   // Add them as member variables here
    std::mutex mu;
    std::condition_variable cond;

   // Your normal variables here
    std::vector<T> buffer_;
    const unsigned int size_ = 10;
};
}
#endif
