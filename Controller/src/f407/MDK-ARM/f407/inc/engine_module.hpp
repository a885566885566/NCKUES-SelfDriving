#ifndef ENGINE_MODULE_H 
#define ENGINE_MODULE_H 

#include "subsystem.hpp"
#include "communication.hpp"
#include "pid_control.hpp"

class EngineSubsystem:public Subsystem{
  private:
    DriverCommunicator driver;
  public:
    COMMU_DATA velocity_cmd;
    COMMU_DATA current_cmd;
    MOTOR_INFO info;
    EngineSubsystem(CAN_HandleTypeDef* hcan_t);
    void set_command(float cmd);
    void reset();
    void prepare();
    void updateMsg();
    void update();
};

#endif
