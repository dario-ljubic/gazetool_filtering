#include <gazetool_filtering/RLS_nan_filtering.h>

filterRLS::filterRLS(){
    sub = n.subscribe("gazeHyps_rls", 1, &filterRLS::gazeCallback, this);
    pub = n.advertise<gazetool::GazeHyps>("gazeHyps_filtered", 1);
}    

filterRLS::~filterRLS(){
}

void filterRLS::gazeCallback(const gazetool::GazeHyps& msgInput){
        
    msgOutput.frame = msgInput.frame;
    
    if (std::isnan(msgInput.horGaze)) msgOutput.horGaze = pastHorGaze;
    else {
        msgOutput.horGaze = msgInput.horGaze;
        pastHorGaze = msgInput.horGaze;
    }
    
    if (std::isnan(msgInput.verGaze)) msgOutput.verGaze = pastVerGaze;
    else {
        msgOutput.verGaze = msgInput.verGaze;
        pastVerGaze = msgInput.verGaze;
    }
    
    msgOutput.mutGaze = msgInput.mutGaze;
    
    pub.publish(msgOutput);
}

void filterRLS::run(){
    // ros::Rate r(20);
    while (ros::ok()) ros::spin(); //ros::spinOnce();
}

// ----------------------------------------------------------------------------

int main(int argc, char **argv){
        
        // If one is using RLS filter in Gazetool, then this filter
        // will just get rid of nan values
    
        ros::init(argc, argv, "RLS_nan_filtering");
        
        filterRLS filter;
        filter.run();
        
        return 0;
}