#define ledPin 13
#define motor1EncoderA A1
#define motor1EncoderB A0
#define motor1Dir 8
#define motor1PWM 5

#define resolution 13517

volatile long motor1Counter = 0;
volatile int motor1Pos = 0;
volatile int motor1Rot = 0;
volatile int motor1Pos_pre = 0;
volatile int motor1Rot_pre = 0;
volatile double motor1Vel = 0;

#define KP 1
#define KI 0.1
volatile double motor1Vcmd = 0;
volatile double PWM = 0;

void timer1Setup(){
  TCCR1A = 0;
  TCCR1B = 0; 
  TCCR1B |= (1<<WGM12);  // CTC mode; Clear Timer on Compare
  TCCR1B |= (1<<CS10);  // Prescaler == 1
  TIMSK1 |= (1 << OCIE1A);  // enable CTC for TIMER1_COMPA_vect
  TCNT1=0;  // counter 歸零
  OCR1A = 32;
}

void timer2Setup(){
  TCCR2A = 0;
  TCCR2B = 0; 
  TCCR2B |= (1<<WGM22);  // CTC mode; Clear Timer on Compare
  TCCR2B |= (1<<CS22);  // Prescaler = 256
  TIMSK2 |= (1 << OCIE2A);  // enable CTC for TIMER1_COMPA_vect
  TCNT2=0;  // counter 歸零 
  OCR2A = 62;
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(motor1EncoderA, INPUT);
  pinMode(motor1EncoderB, INPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  
  cli();  // 禁止中斷
  timer1Setup();  // 130 kHz
  timer2Setup();  // 1 kHz
  sei();  // 允許中斷
}
void driveMotor1(int speed){
  if(speed > 0){
    digitalWrite(motor1Dir, HIGH);
    analogWrite(motor1PWM, speed);
  }
  else{
    digitalWrite(motor1Dir, LOW);
    analogWrite(motor1PWM, -speed);
  }
}
double dx = 0.05;
double x = 0;

void loop() {
  if(x>5 || x<-5) dx *= -1;
    x += dx;
  
  motor1Vcmd = x;
  /*
  Serial.print(x);
  Serial.print(", ");
  */
  /*
  Serial.print("Counter= ");
  Serial.print(motor1Counter);
  Serial.print(", ");
  Serial.print("POS= ");
  Serial.print(motor1Pos);
  Serial.print(", ");
  Serial.print(motor1Rot);
  Serial.print(", Vel=");
  Serial.print(motor1Vel);
  Serial.print(", PWM=");
  Serial.println(PWM);*/
  
  Serial.print(motor1Vel);
  Serial.print(",");
  Serial.print(PWM/32);
  Serial.print(",");
  Serial.println(x);
  delay(10);
}

ISR(TIMER1_COMPA_vect)
{
  static bool A_pre = false;
  static bool B_pre = false;
  static bool A = false;
  static bool B = false;
  /*
  bool A = digitalRead(motor1EncoderA);
  bool B = digitalRead(motor1EncoderB);
  */
  A = PINC & B00000010;
  B = PINC & B00000001;
  if(!A && !B){
    if(A_pre && !B_pre)       motor1Counter++;
    else if(!A_pre && B_pre)  motor1Counter--;
  }
  else if(!A && B){
    if(!A_pre && !B_pre)      motor1Counter++;
    else if(A_pre && B_pre)   motor1Counter--;
  }
  else if(A && B){
    if(!A_pre && B_pre)       motor1Counter++;
    else if(A_pre && !B_pre)  motor1Counter--;
  }
  else if(A && !B){
    if(A_pre && B_pre)        motor1Counter++;
    else if(!A_pre && !B_pre) motor1Counter--;
  }
  A_pre = A;
  B_pre = B;
/*
  while(motor1Counter > resolution){
    motor1Counter -= resolution;
    motor1Rot++;
  }
  while(motor1Counter < -resolution){
    motor1Counter += resolution;
    motor1Rot--;
  }
  motor1Pos = motor1Counter;
  */
}

ISR(TIMER2_COMPA_vect){
  static double err = 0;
  static double G = 0;
  static double In = 0;
  
  while(motor1Counter > resolution){
    motor1Counter -= resolution;
    motor1Rot++;
  }
  while(motor1Counter < -resolution){
    motor1Counter += resolution;
    motor1Rot--;
  }
  motor1Pos = motor1Counter;

  int delPos = motor1Pos - motor1Pos_pre;
  int delRot = motor1Rot - motor1Rot_pre;
  motor1Vel = 1000.0 * delPos / resolution + 1000.0 * delRot;
  motor1Pos_pre = motor1Pos;
  motor1Rot_pre = motor1Rot;

  err = motor1Vcmd - motor1Vel;
  G = err * KP;
  In = In + 0.001 * err * KI;

  G =  (G>30  ? 30 : (G< -30 ? -30 : G));
  In = (In>30 ? 30 : (In<-30 ? -30 : In));
  PWM += G + In;
  PWM = (PWM>245 ? 245 : (PWM<-245 ? -245 : PWM));
  driveMotor1((int)PWM);
}
