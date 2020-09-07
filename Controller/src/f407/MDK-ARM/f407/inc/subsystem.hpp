#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

/* The order of this definition should not be changed,
 * */
typedef enum {STATE_RESET, STATE_SYSTEM_UP, STATE_READY, STATE_MANUAL, STATE_AUTO, STATE_ERROR}SYSTEM_STATE;

class Subsystem{
  private:
  protected:
  public:    
    SYSTEM_STATE state_req;
    SYSTEM_STATE state;
  
    Subsystem();
    virtual void reset() = 0;
    bool set_state(SYSTEM_STATE req);
    virtual void update() = 0;
    virtual ~Subsystem();
};

#endif
