#pragma once

#include <iostream>
#include <queue>
#include <fstream>

// ros include
#include "ros/ros.h"

// user packages
#include "gazetool/GazeHyps.h"

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

class subscribeAndPublish
{
    
public:
    subscribeAndPublish();
    void callback(const gazetool::GazeHyps& msg);
    void initialize();
    void gazeFilter();
    void wait();
    void run(bool write, const char* path);
    //void run();
    
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
    int nanLimit = 50; // approx 2.5 seconds of nan data is allowed
    int nanHorNum = 0;
    int nanVerNum = 0;
    
    // write to file
    bool writeToFile = false;
    std::ofstream logFile;
    writeFiltData writeObj;
};