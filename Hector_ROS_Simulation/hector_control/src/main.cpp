#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/rt_rc_interface.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"


bool running = true;
int xbox_port=0;
rc_control_settings rc_control_new;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}
void run_xbox_sbus() {
  printf("[run_xbox] starting...\n");
//   xbox_port = init_xbox_js();

  while(true){
    if (xbox_port > 0) {
        js_complete(xbox_port);
    }
    usleep(5000);
  }
}
int main(int argc, char ** argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    
    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;

    ioInter = new CheatIO(robot_name);
    ros::Rate rate(1000);

    double dt = 0.001;
    Biped biped;
    biped.setBiped();

    LowLevelController* legController = new LowLevelController(biped);
    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   

    std::cout << "setup state etimator" << std::endl;                                                             
    // initialized the xbox
    memset(&rc_control_new, 0, sizeof(rc_control_settings));
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&rc_control_new,&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);
    // xbox initial

    xbox_port = init_xbox_js();
    std::thread* sbus_thread = new std::thread(&run_xbox_sbus);

    signal(SIGINT, ShutDown);
    
    while(running)
    {
        _FSMController->run();
        get_rc_control_settings(&rc_control_new);
        rate.sleep();
    }
    
    system("stty sane");  //Terminal back to normal
    delete _controlData;
    return 0;

}
