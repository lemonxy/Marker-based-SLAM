#ifndef UCOSLAM_TIMERS_H
#define UCOSLAM_TIMERS_H


#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include "debug.h"
namespace ucoslam{

//timer
struct ScopeTimer
{
    std::chrono::high_resolution_clock::time_point begin,end;

    std::string name;
    bool use;
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    inline ScopeTimer(std::string name_,bool use_=true,SCALE _sc=MSEC)
    {
#ifdef USE_TIMERS
        name=name_;
        use=use_;
        sc=_sc;
        begin= std::chrono::high_resolution_clock::now();
#endif
    }
    inline ~ScopeTimer()
    {
#ifdef USE_TIMERS
        if (use &&debug::Debug::showTimer){
            end= std::chrono::high_resolution_clock::now();
            double fact=1;
            std::string str;
            switch(sc)
            {
            case NSEC:fact=1;str="ns";break;
            case MSEC:fact=1e6;str="ms";break;
            case SEC:fact=1e9;str="s";break;
            };

            std::cout << "Time ("<<name<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())/fact<<str<<std::endl; ;
        }
#endif
    }
};

struct ScopedTimerEvents
{
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    std::vector<std::chrono::high_resolution_clock::time_point> vtimes;
    std::vector<std::string> names;
    std::string _name;

 inline   ScopedTimerEvents(const std::string &name="",bool start=true,SCALE _sc=MSEC){
#ifdef USE_TIMERS
        if(start) add("start");sc=_sc;_name=name;
#endif
    }

   inline void add(const std::string &name){
#ifdef USE_TIMERS
        vtimes.push_back(std::chrono::high_resolution_clock::now());
        names.push_back(name);
#endif
    }
    inline void addspaces(std::vector<std::string> &str ){
        //get max size
        size_t m=0;
        for(auto &s:str)m=(std::max)(size_t(s.size()),m);
        for(auto &s:str){
            while(s.size()<m) s.push_back(' ');
        }
    }

   inline ~ScopedTimerEvents(){
#ifdef USE_TIMERS
        if (!debug::Debug::showTimer) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };

        add("total");
        addspaces(names);
        for(size_t i=1;i<vtimes.size();i++){
            std::cout<<"Time("<<_name<<")-"<<names[i]<<" "<< double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[i-1]).count())/fact<<str<<" "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[0]).count())/fact<<str<<std::endl;
        }
#endif
    }
};

struct Timer{
    enum SCALE {NSEC,MSEC,SEC};

    std::chrono::high_resolution_clock::time_point _s;
    double sum=0,n=0;
    std::string _name;
    Timer(){}

    Timer(const std::string &name):_name(name){}
    void setName(std::string name){_name=name;}
   inline void start(){_s=std::chrono::high_resolution_clock::now();}
   inline void end()
    {
#ifdef USE_TIMERS
        auto e=std::chrono::high_resolution_clock::now();
        sum+=double(std::chrono::duration_cast<std::chrono::nanoseconds>(e-_s).count());
        n++;
#endif
    }

   inline void print(SCALE sc=MSEC){
#ifdef USE_TIMERS
       if (!debug::Debug::showTimer) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };
        std::cout<<"Time("<<_name<<")= "<< ( sum/n)/fact<<str<<std::endl;
#endif
    }

};


struct TimerAvrg{
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;

    TimerAvrg(int _n=30)
    {
        n=_n;
        times.reserve(n);
    }
    inline void start(){
        begin= std::chrono::high_resolution_clock::now();

    }

    inline void stop(){
        end= std::chrono::high_resolution_clock::now();

        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if ( times.size()<n) times.push_back(duration);
        else{
            times[curr]=duration;
            curr++;
            if (curr>=times.size()) curr=0;
        }
    }

    void reset(){
        times.clear();
        curr=0;
    }
//returns time in seconds
   inline double getAvrg(){
        double sum=0;
        for(auto t:times) sum+=t;
        return sum/double(times.size());
    }
};

}


#endif
