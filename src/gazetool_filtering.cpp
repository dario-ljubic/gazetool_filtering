#include <gazetool_filtering/subscribeAndPublish.h>

//WARNING Sometimes, when running the node, segmentation fault (core dumped) appears due to
// still unknown reason/s. It is presumed that the implementation with queues is causing the
// problem. Therefore, implementation of the lists instead of queues should be considered.

int main(int argc, char **argv){
    
//     if (argc < 2) {
//         std::cerr << "Not enough input parameters!" << std::endl;
//         return 1;
//     }
    
    ros::init(argc, argv, "gazetool_filter");
    
    subscribeAndPublish filterObj;
    filterObj.initialize();
    
    ros::Rate r(20); //TODO check the rate in which whis this while loop will execute

    std::ofstream logFile;
    writeFiltData writeObj(logFile, "text.csv");
    
    while (ros::ok()){
        
        ros::spinOnce();
        
        filterObj.gazeFilter();
        
        //NOTE: frames are for some reasons not written correctly. It takes about 100 frames for it to 
        // write without having zeros written.
        writeObj.dumpEst(logFile, filterObj.output);
        
        r.sleep();
    }

    logFile.close();
    
    return 0;
}