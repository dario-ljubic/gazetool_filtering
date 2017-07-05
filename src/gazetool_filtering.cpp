#include <gazetool_filtering/subscribeAndPublish.h>

//WARNING Sometimes, when running the node, segmentation fault (core dumped) appears due to
// still unknown reason/s. It is presumed that the implementation with queues is causing the
// problem.
// EDIT: It seems that the problem is fixed, but more testing is needed to be sure!

int main(int argc, char **argv){
    
    std::ofstream logFile;
    
    bool write;
    char *path;
    //writeFiltData writeObj;
    char option;
     
    if (argc == 1) write = false;
    else if (argc > 1) {
        if (argc < 3){
            std::cerr << "Not enough input parameters!" << std::endl;
            return 1;
        }
        if (argc > 3){
            std::cerr << "Too many input parameters!" << std::endl;
            return 1;
        }
        if (argc == 3){
            option = *argv[1];
            switch(option){
                case('w'):
                {
                    write = true;
                    path = argv[2];
                    //std::cout << path << std::endl;
                    //writeObj = writeFiltData(logFile, path);
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
    }    
    
    ros::init(argc, argv, "gazetool_filter");
    
    subscribeAndPublish filterObj;
    
    filterObj.run(write, path);
    
    return 0;
}