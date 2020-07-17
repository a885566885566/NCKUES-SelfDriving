#include "utils.h"
#include "Arduino.h"

bool motor_direction_key = CONF_MOTOR_CW;
double CURRENT_SENSOR_BASE = 0;

void pin_init(){
  /* ---- OUTPUT ---- */
  pinMode(PIN_LA, OUTPUT);
  pinMode(PIN_VE, OUTPUT);
  pinMode(PIN_TD, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_RELEASE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_LA, LOW);
  digitalWrite(PIN_VE, LOW);
  digitalWrite(PIN_TD, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_RELEASE, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  
  pinMode(PIN_REV, INPUT);
  pinMode(PIN_TG, INPUT);
  pinMode(PIN_CUR_A, INPUT);
}

void system_init(){
  /* Set dead time */
  digitalWrite(PIN_TD, CONF_DEAD_TIME);
  digitalWrite(PIN_LA, LOW);
  digitalWrite(PIN_VE, LOW);

  /* Current sensor initialization */
  CURRENT_SENSOR_BASE = 0; 
  for (int i =0; i<50; i++){
    CURRENT_SENSOR_BASE += analogRead(PIN_CUR_A);
    delay(10);
  }
  CURRENT_SENSOR_BASE /= 50;
}

bool system_check(){

}

/*  */
void motor_key(bool key, bool mode=CONF_MOTOR_CW){
  digitalWrite(PIN_RELEASE, key);
  motor_direction_key = mode;
}

/* Input velocity in ve command */
void write_motor(double velocity){
  if (motor_direction_key == CONF_MOTOR_CW)
      velocity = constrain(velocity, 0, CONF_PWM_LIM);
  else
      velocity = constrain(velocity, -CONF_PWM_LIM, 0);
  // Set direction
  digitalWrite(PIN_LA, LOW);
  if(velocity > 0) digitalWrite(PIN_DIR, CONF_MOTOT_DIR);
  else digitalWrite(PIN_DIR, !CONF_MOTOT_DIR);
  analogWrite(PIN_VE, fabs(velocity));
}

/* Read motor current, return in ampere. 
 * expontial decay filter built in. */
double read_current(){
  // expontial dacay filter
  static double analog = CURRENT_SENSOR_BASE;
  analog = analog * CONF_CURRENT_FILTER_DECAY + analogRead(PIN_CUR_A) * (1-CONF_CURRENT_FILTER_DECAY);
  return (analog - CURRENT_SENSOR_BASE)*150.0/512;
}

/* Setup buzzer pin */
void utils_beep_init(volatile BEEPER_CONFIG* beeper, uint16_t port){
  beeper->beepPort = port;
}

/* Setup buzzer mode */
void utils_beep_set(volatile BEEPER_CONFIG* beeper, uint16_t mode, uint16_t dur, uint16_t times){
  beeper->starting = millis();
  beeper->mode = mode;
  beeper->dur = dur;
  beeper->times = times;
}

/* Buzzer updater */
void utils_beep_update(volatile BEEPER_CONFIG* beeper){
  // No beeper running
  if(beeper->mode == 0) return;
  uint32_t diff = millis() - beeper->starting;
  switch(beeper->mode){
    case 1:
      if( diff > 2 * beeper->times * beeper->dur ){
        beeper->mode = 0; 
        digitalWrite(beeper->beepPort, LOW);
      }
      else if( (diff/beeper->dur)%2 == 1 )
        digitalWrite(beeper->beepPort, HIGH);
      else
        digitalWrite(beeper->beepPort, LOW);
      break;
      
    default:
      break;
  }
}
  