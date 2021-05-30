#include <BTS7960.h>
#include <Encoder.h>
#include <PID_v1.h>

double last_time = micros();
double tick_time = 0;
long old_position = 0;

double Setpoint, Output;
double Input = 0;
double Kp=1.3, Ki=15, Kd=0.01;

bool twist_flag = false;

double wheel_base_length = 0.34;
double wheel_base_width = 0.22;
double wheel_circumfrance = 0.4273;

double twist = 0.2;
double vel = 0.0;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;

BTS7960 motorController(6, 4, 5, 11); //en_l en_r l_pwm r_pwm
Encoder myEnc(3, 2);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


double calculate_rpm(){
   long new_position = myEnc.read();
   long position_change;
   double RPM;

   if (new_position != old_position) {
       tick_time = (micros() - last_time);
       position_change = old_position - new_position;
       RPM = 1 / ((double(tick_time / position_change) * 18538)/1000000/60); //10041 18538 = ticks per rev, 1 rev = 42.73cm
       old_position = new_position;
       last_time = micros();   
   }
   else{
       RPM = 0.0;
   }
   return RPM;
}



double covert_vel_rpm(double vel){
  double RPM; 
  RPM = (vel / wheel_circumfrance) * 60;
  return RPM;
}

void setup(){
   motorController.Enable();
   Setpoint = 0;
   myPID.SetSampleTime(20);
   myPID.SetMode(AUTOMATIC);
   myPID.SetOutputLimits(-255,255);
   Serial.begin(115200);
   inputString.reserve(200);
}

void loop(){
  char debug = 0;
  double OutputMag;

  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:
    vel = inputString.toFloat();
    Serial.println(vel);
    inputString = "";
    stringComplete = false;
  }
  
  Setpoint = covert_vel_rpm(vel);
  Input = calculate_rpm();
  myPID.Compute();


  if (0 > Setpoint){
    motorController.TurnLeft(-Output); 
  }
  else{
    motorController.TurnRight(Output); // forwards
  }
       
  if (debug == 1){
    Serial.print("Output_PWM:");
    Serial.print(Output);
    Serial.print(",");
    Serial.print("Input_RPM:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("Setpoint:");
    Serial.println(Setpoint);
  }
        
  if (debug == 2){
    Serial.print("Ticks:");
    Serial.println(myEnc.read());
  }
  
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
