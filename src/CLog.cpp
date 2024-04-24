#include "include/CLog.h"
#include <iostream>

using namespace LIOSAM;

#define OPEN_LOG        1

std::mutex CLog::lock;
std::function<void(const std::string)> CLog::logger = [](const std::string str){

#if OPEN_LOG
    std::cout << str;
#endif
};

void CLog::registLogger(std::function<void(const std::string)> logger) {

    CLog::logger = logger;
}

CLog::CLog() {

    (*this) << "[3DSLAM]";
    CLog::lock.lock();
}

CLog::~CLog() {

    CLog::logger(this->str());
    CLog::lock.unlock();
}

