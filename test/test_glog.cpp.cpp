//
// Created by rain on 17-11-22.
//

#include <string>
#include <iostream>

#include <glog/logging.h>

using namespace std;

int main(int argc,char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::WARNING);
    FLAGS_log_dir = "../log";
    FLAGS_colorlogtostderr = true;
    LOG(INFO) << "google glog test";

    CHECK_EQ(string("abc")[1], 'b'); // CHECK_EQ, CHECK_NE, CHECK_LE, CHECK_LT, CHECK_GE, CHECK_GT
    CHECK_NE(1, 2) << "there are something wrong";
    string* Pointer;
    CHECK_NOTNULL(Pointer);

    for(int i = 1; i <= 100;i++)
    {
        LOG_IF(INFO,i==100)<<"LOG_IF(INFO,i==100)  google::COUNTER="  << google::COUNTER << "  i=" << i;
        LOG_EVERY_N(INFO,10)<<"LOG_EVERY_N(INFO,10)  google::COUNTER=" << google::COUNTER << "  i=" << i;
        LOG_IF_EVERY_N(WARNING,(i>50),10) << "LOG_IF_EVERY_N(INFO,(i>50),10)  google::COUNTER=" << google::COUNTER << "  i="<<i;
        LOG_FIRST_N(ERROR,5) << "LOG_FIRST_N(INFO,5)  google::COUNTER="<<google::COUNTER << "  i=" << i;
    }


    google::ShutdownGoogleLogging();

    return 0;
}

