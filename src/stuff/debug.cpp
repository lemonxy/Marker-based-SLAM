#include "debug.h"
#include <fstream>
namespace ucoslam{
namespace debug{
int Debug::level=0;
std::map<std::string,std::string> Debug::strings;
bool Debug::showTimer=false;

void Debug::addString(std::string &label, std::string &data){
    strings.insert(make_pair(label,data));
}

std::string Debug::getString(std::string &str){
    auto it=strings.find(str);
    if (it==strings.end())return "";
    else return it->second;
}

bool Debug::isString(const std::string &str){
 return strings.count(str)!=0;
}

bool Debug::isInited=false;

void Debug::setLevel ( int l ) {
    level=l;
    isInited=false;
    init();
}
int Debug::getLevel() {
    init();
    return level;
}
void Debug::init() {
    if ( !isInited ) {
        isInited=true;
        if ( level>=1 ) {
        }
    }

}


}
}

