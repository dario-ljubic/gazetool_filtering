#include <gazetool_filtering/subscribeAndPublish.h>

subscribeAndPublish::subscribeAndPublish()
{   
    // publish to a topic
    pub = n.advertise<gazetool::GazeHyps>("gazeHyps_filtered", 1000); //TODO: check the necessary queue size
    // name of the topic has to be the same as the name of the topic to which gazetool is publishing unfiltered data
    sub = n.subscribe("gazeHyps_raw", 1000, &subscribeAndPublish::callback, this); //TODO: check the necessary queue size, currently 5 sec of data is preserved
    
}

void subscribeAndPublish::callback(const gazetool::GazeHyps& msg)
{   
    // if the incoming data is nan then take last non nan element.
    //TODO: if there are many consecutative nans, throw an error!
    if (std::isnan(msg.verGaze)) verGaze_u.push(verGaze_u.front());
    else verGaze_u.push(msg.verGaze);
    
    if (std::isnan(msg.horGaze)) horGaze_u.push(verGaze_u.front());
    else horGaze_u.push(msg.horGaze);
    
    mutGaze.push(msg.mutGaze);
    frame.push(msg.frame);
}

void subscribeAndPublish::gazeFilter()
{
    
    float Tpass, Ts; //TODO: add variable fps to the GazeHyps msg so that the average frequency can be used insted of assumed one!
    float b0, b1, a0, a1;
    
    Tpass = 0.6; // filter parameter, change for different pass frequencies!
    Ts = 1./20; // sampling time, under assumption that gazetool is providing 20 fps
    
    // based on the transfer function given in the README.md, coefficients are as follows:
    b0 = 1;
    b1 = 1;
    a0 = 1 + (2*Tpass)/Ts;
    a1 = 1 - (2*Tpass)/Ts;
    
    output.horGaze = b1/a0 * (float) horGaze_u.front() - a1/a0 * (float) horGaze_y.front();  
    output.verGaze = b1/a0 * (float) verGaze_u.front() - a1/a0 * (float) verGaze_y.front();
    
    //TODO: check if there is a need to implement lists instead of queues, because of the error when loop rate 
    // is higher than 20 hz. In case there is a need for lists, just shift the data, do not pop it.
    
    horGaze_u.pop(); 
    horGaze_y.pop();
    verGaze_u.pop();
    verGaze_y.pop();
    
    // and add the elements from the current step
    output.horGaze = output.horGaze + b0/a0 * (float) horGaze_u.front();
    output.verGaze = output.verGaze + b0/a0 * (float) verGaze_u.front();
    output.frame = frame.front();
    output.mutGaze = mutGaze.front();
    
    frame.pop();
    mutGaze.pop();
    
    // push new elements on the queue
    horGaze_y.push(output.horGaze);
    verGaze_y.push(output.verGaze);
    
    pub.publish(output);
}

void subscribeAndPublish::initialize()
{
    // initial values t(-1) = 0.
    verGaze_u.push(0);
    verGaze_y.push(0);
    horGaze_u.push(0);
    horGaze_y.push(0);
    
    // initial values for frame and mutual gaze. Frame starts at 0 so that filtered data can start at frame 0.
    output.frame = 0;
//     output.lid = 0;
    output.mutGaze = false;
}

//----------------------------------------------------------------------------------------------------------------------//

writeFiltData::writeFiltData(std::ofstream& logFile, const char* path)
{
    logFile.open(path);
    writeEstHeader(logFile);
}

void writeFiltData::writeEstHeader(std::ofstream& fout) {
    fout << "Frame" << "\t"
//          << "Id" << "\t"
//          << "Label" << "\t"
//          << "Lid" << "\t"
         << "HorizGaze" << "\t"
         << "VertGaze" << "\t"
         << "MutualGaze"
         << std::endl;
}

void writeFiltData::dumpEst(std::ofstream& fout, gazetool::GazeHyps& filtMsg) {
    if (fout.is_open()) {
        fout << filtMsg.frame << "\t"
//              << msg.id << "\t"
//              << msg.label << "\t"
//              << msg.lid << "\t"
             << filtMsg.horGaze << "\t"
             << filtMsg.verGaze << "\t"
             << filtMsg.mutGaze
             << std::endl;
    }
}

writeFiltData::~writeFiltData(){
    
}