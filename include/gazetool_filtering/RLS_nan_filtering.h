#include <iostream>

// ROS
#include <ros/ros.h>

// user defined
#include "gazetool/GazeHyps.h"

class filterRLS{
public:
    filterRLS();
    ~filterRLS();
    void run();    
    
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    
    double pastHorGaze = 0;
    double pastVerGaze = 0;
    
    gazetool::GazeHyps msgOutput;
    
    void gazeCallback(const gazetool::GazeHyps& msgInput);
    
};