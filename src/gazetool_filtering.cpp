#include <gazetool_filtering/subscribeAndPublish.h>

//WARNING Sometimes, when running the node, segmentation fault (core dumped) appears due to
// still unknown reason/s. It is presumed that the implementation with queues is causing the
// problem. Therefore, implementation of the lists instead of queues should be considered.

int main(int argc, char **argv){
    
    std::ofstream logFile;
    
    bool write;
    char *path;
    writeFiltData writeObj;
    char option;
    
//    std::cout << argc << std::endl;
//    std::cout << *argv[0] << std::endl;
//    std::cout << *argv[1] << std::endl;
//    std::cout << *argv[2] << std::endl;
//    std::cout << *argv[3] << std::endl;
//     
    if (argc == 1) write = false;
    else if (argc > 1) {
        if (argc < 3){
            std::cerr << "Not enough input parameters!" << std::endl;
            return 1;
        }
        else if (argc > 3){
            std::cerr << "Too many input parameters!" << std::endl;
            return 1;
        }
    }
    else {
        option = *argv[1];
        switch(option){
            case('w'):
            {
                write = true;
                path = argv[2];
                writeObj = writeFiltData(logFile, path);
                break;
            }
            //TODO: Add more options if needed
            default:
            {
                std::cerr << "Invalid option!" << std::endl;
                return 1;
            }
        }
             
    }
    ros::init(argc, argv, "gazetool_filter");
    
    subscribeAndPublish filterObj;
    filterObj.initialize();
    
    ros::Rate r(20); //TODO check the rate in which whis this while loop will execute

    //writeFiltData writeObj(logFile, "/home/larics/schunk_ws/src/gazetool_filtering/log/text.csv");
    
    while (ros::ok()){
        
        //ros::spinOnce();
        
        filterObj.gazeFilter();
        
        //NOTE: frames are for some reasons not written correctly. It takes about 100 frames for it to 
        // write without having zeros written.
        
        if (write) writeObj.dumpEst(logFile, filterObj.output);
        //writeObj.dumpEst(logFile, filterObj.output);
        
        r.sleep();
    }
    
    if (write) writeObj.closeFile(logFile);
    //logFile.close();
    
    return 0;
}