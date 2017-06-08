#pragma once

#include "ros/ros.h"
#include "gazetool/GazeHyps.h"
#include <iostream>
#include<queue>
#include <fstream>

class subscribeAndPublish
{
    
public:
    subscribeAndPublish();
    void callback(const gazetool::GazeHyps& msg);
    void initialize();
    void gazeFilter();
    
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
    
    //gazetool::GazeHyps output;
    
};

class writeFiltData : public std::ofstream
{
    
public:
    writeFiltData(std::ofstream& fout, const char* path);
    void writeEstHeader(std::ofstream& fout);
    void dumpEst(std::ofstream& fout, gazetool::GazeHyps& filtMsg);
    ~writeFiltData();
    
};