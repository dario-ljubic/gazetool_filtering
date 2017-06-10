#pragma once

#include <iostream>
#include <queue>
#include <fstream>

// ros include
#include "ros/ros.h"

// user packages
#include "gazetool/GazeHyps.h"

class subscribeAndPublish
{
    
public:
    subscribeAndPublish();
    void callback(const gazetool::GazeHyps& msg);
    void initialize();
    void gazeFilter();
    void wait();
    
    gazetool::GazeHyps output;
    
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    
    std::queue<float> verGaze_y;
    std::queue<float> horGaze_y;
    std::queue<float> verGaze_u;
    std::queue<float> horGaze_u;
    std::queue<int> mutGaze;
    std::queue<int> frame;
    bool hold = false;
    
};

class writeFiltData
{
    
public:
    writeFiltData();
    writeFiltData(std::ofstream& fout, const char* path);
    void writeEstHeader(std::ofstream& fout);
    void dumpEst(std::ofstream& fout, gazetool::GazeHyps& filtMsg);
    void closeFile(std::ofstream& fout);
    ~writeFiltData();
    
};