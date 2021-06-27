#ifndef __Debug_H
#define __Debug_H
#include <iostream>
#include <fstream>
#include <ctime>
#include "ucoslam_exports.h"
#include <map>
#include <string>
namespace ucoslam{

namespace debug{
class UCOSLAM_API  Debug{
private:
    static  int level;//0(no debug), 1 medium, 2 high
    static bool isInited;

    static std::map<std::string,std::string> strings;

public:
    static bool showTimer;
    static void init();
    static void setLevel(int l);
    static int getLevel();



    static void addString(std::string &label,std::string &data);
    static std::string getString(std::string &str);
    static bool  isString(const std::string &str);


static std::string getFileName(std::string filepath){
    //go backwards until finding a separator or start
    int i;
    for( i=filepath.size()-1;i>=0;i--){
        if ( filepath[i]=='\\' || filepath[i]=='/') break;
    }
    std::string fn;fn.reserve( filepath.size()-i);
    for(size_t s=i;s<filepath.size();s++)fn.push_back(filepath[s]);
    return fn;
}

#ifdef WIN32
  #define __func__ __FUNCTION__
#endif

};


//#define FORCEMESSAGES
//message print
#ifdef PRINT_DEBUG_MESSAGES
#define _debug_exec(level,x) {if (debug::Debug::getLevel()>=level){x}}
#define _debug_exec_( x) x

#ifndef WIN32
    #define _debug_msg(x,level) {debug::Debug::init();\
    if (debug::Debug::getLevel()>=level)\
              std::cout<<"#" <<debug::Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

    #define _debug_msg_(x) {debug::Debug::init();\
    if (debug::Debug::getLevel()>=5)\
          std::cout<<"#" <<debug::Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

#else
     #define _debug_msg(x,level) {\
     debug::Debug::init();\
     if (debug::Debug::getLevel()>=level)\
       std::cout<<__func__<<":"<< debug::Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#define _debug_msg_(x) {\
debug::Debug::init();\
if (debug::Debug::getLevel()>=5)\
  std::cout<<__func__<<":"<< debug::Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#endif

#else
#define _debug_msg(x,level)
#define _debug_msg_(x)
#define _debug_exec(level,x)
#define _debug_exec_(x)

#endif



}

}

#endif

