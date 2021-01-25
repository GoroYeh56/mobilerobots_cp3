
///////////// ROS Communication here. /////////////////
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>

// Pin definition
#define enA 5
#define in1 8
#define in2 9
#define enB 6
#define in3 10
#define in4 11

#define REDUCTION_RATIO 120
#define ENCODER_CPR 16
//#define T 100 // sampling period
#define max_setpoint 255
#define  min_setpoint 0

ros::NodeHandle nh;      // Input value from RPi.
double Left_PWM, Right_PWM; // PWM signals for Motor.
double Left_setpoint, Right_setpoint;
double Lcontrol_signal;
double Rcontrol_signal;
///////// Get Encoder Feedback (Interrupt)

//The sample code for driving one way motor encoder
#include <PID_v1.h>
const byte Lencoder0pinA = 2;//A pin -> the interrupt pin 0
const byte Lencoder0pinB = 7;//B pin -> the digital pin 4
const byte Rencoder0pinA = 3;//A pin -> the interrupt pin 1
const byte Rencoder0pinB = 4;//B pin -> the digital pin 5

byte Lencoder0PinALast;
byte Rencoder0PinALast;

double Lduration, Rduration;//the number of the pulses
double Lduration_last, Rduration_last;
double Labs_duration, Rabs_duration;

boolean LDirection;//the rotation direction
boolean Lresult;

boolean RDirection;
boolean Rresult;

///////// PID Configuration //////////////

double LKp =5;
double LKi =2;
double LKd =0.1;
double RKp =5;
double RKi =2;
double RKd =0.1;

PID LeftPID(&Labs_duration, &Lcontrol_signal, &Left_setpoint, LKp, LKi, LKd, DIRECT);
PID RightPID(&Rabs_duration, &Rcontrol_signal, &Right_setpoint, RKp, RKi, RKd, DIRECT);


// command: LeftPWM: 200  RightPWM: 218

//std_msgs::Int64 ack_msg;
////////////// CALLBACK function /////////////
// Upon receiving left,right PWM => Convert to motor actuate value
void Input_Left_PWM_Callback( const std_msgs::Int64& msg){

  Left_PWM = (double)msg.data;
  Left_setpoint = abs(Left_PWM);
  
//  ack_msg.data = Left_PWM;
//  publisher.publish(&int_mul_result);
}

void Input_Right_PWM_Callback( const std_msgs::Int64& msg){

  Right_PWM = (double)msg.data;
  Right_setpoint = abs(Right_PWM);

//  publisher.publish(&int_mul_result);
}



ros::Subscriber<std_msgs::Int64> Left_sub("Input_Left_PWM", &Input_Left_PWM_Callback );
ros::Subscriber<std_msgs::Int64> Right_sub("Input_Right_PWM", &Input_Right_PWM_Callback );




int T = 100;

//////// TODO: Threads programming in arduino.
//////// Two Threads to parallel drive motors.


////////// Thread Library /////////////
#include <Thread.h>
#include <StaticThreadController.h>

Thread* LeftThread = new Thread();
Thread* RightThread = new Thread();

// callback for LeftWheel_Thread
void LeftThreadCallback(){
   analogWrite(enA, Lcontrol_signal);
   Labs_duration=abs(Lduration);
   Lresult= LeftPID.Compute();//PID conversion is complete and returns 1
   if(Lresult) Lduration = 0; //Count clear, wait for the next count
   
}

// callback for RightWheel_Thread
void RightThreadCallback(){
   analogWrite(enB, Rcontrol_signal);
   Rabs_duration=abs(Rduration);
   Rresult= RightPID.Compute();//PID conversion is complete and returns 1
   if(Rresult) Rduration = 0; //Count clear, wait for the next count
}

StaticThreadController<2> controller (LeftThread, RightThread);

void setup() {
  
  nh.initNode();
  nh.subscribe(Left_sub);
  nh.subscribe(Right_sub);
  // nh.advertise(publisher);
  
  pinMode(enA,OUTPUT) ;   //we have to set PWM pin as output
  pinMode(in1,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in2,OUTPUT) ;  
  pinMode(enB,OUTPUT) ;   //we have to set PWM pin as output
  pinMode(in3,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in4,OUTPUT) ;

  
  //Initialize left/right wheel pwm values.
  Left_PWM = 100;
  Right_PWM = 100;

  Left_setpoint = 0;
  Right_setpoint = 0;
  
  LeftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  LeftPID.SetSampleTime(T);//Set PID sampling frequency is 100ms

  RightPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  RightPID.SetSampleTime(T);//Set PID sampling frequency is 100ms
  
  
  EncoderInit();

  /////////// Multi-threading //////////////
  LeftThread->onRun(LeftThreadCallback);
  RightThread->onRun(RightThreadCallback);
  LeftThread->setInterval(10); // 10 ms
  RightThread->setInterval(10);
  

  

  // For debugging
//   Serial.begin(9600);
   nh.spinOnce();
}

void loop()
{
  //// 1. Determine the Direction.
  Check_Direction();

  //// 2. Compute PID value : Lcontrol_signal & Rcontrol_signal & Drive motor.
  controller.run();

  //// 3. Listen to ROS Topics
  nh.spinOnce();
}



  ///////// Done ///////
  
  //PID_Control();
//
//   Labs_duration=abs(Lduration);
//   Lresult= LeftPID.Compute();//PID conversion is complete and returns 1
//   if(Lresult)
//   {
//      //  Serial.print("Pluse: ");
//      //  Serial.println(Lduration);
//      Lduration = 0; //Count clear, wait for the next count
//   }
//
//   
//   Rabs_duration=abs(Rduration);
//   Rresult= RightPID.Compute();//PID conversion is complete and returns 1
//   if(Rresult)
//   {
//    //  Serial.print("Pluse: ");
//    //  Serial.println(Rduration);
//      Rduration = 0; //Count clear, wait for the next count
//   }  

  /////////// 3. Drive motor.
//   controller.run();
  
//  analogWrite(enA,abs(Lcontrol_signal)) ;
//  analogWrite(enB,abs(Rcontrol_signal)) ;
  
  
  /////////// 4. Listen to ROS Topics
//  nh.spinOnce();
  


  /*setting pwm of the motor to 255
  we can change the speed of rotaion
  by chaning pwm input but we are only
  using arduino so we are using higest
  value to driver the motor  */


//For Clock wise motion , in1 = High , in2 = Low

void Check_Direction(){

  // Forward
  if(Left_PWM<0 && Right_PWM<0){
    //Serial.print("Forward: ");
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,LOW) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,LOW) ;
  }
  // Backward
  else if(Left_PWM>0 && Right_PWM>0){
    digitalWrite(in1,LOW) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,LOW) ;
    digitalWrite(in4,HIGH) ;
     
  }
  // Turn Right
  else if(Left_PWM > 0 && Right_PWM<0){
    digitalWrite(in1,LOW) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,LOW) ;
  }  
  // Turn Left
  else if(Left_PWM<0 && Right_PWM>0){
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,LOW) ;
    digitalWrite(in3,LOW) ;
    digitalWrite(in4,HIGH) ;
  }  
  // Brake
  else if(Lcontrol_signal==0 && Rcontrol_signal==0){
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,HIGH) ; 
  }
  
}


////////////// Encoder /////////////
void EncoderInit()
{
  LDirection = true;//default -> Forward
  RDirection = true;

  // Initialize
  Lduration_last = Rduration_last = 0;

  pinMode(Lencoder0pinB,INPUT);
  pinMode(Rencoder0pinB,INPUT);            // Interrupt pin 0 : = LencoderpinA = digital 2
                                           // Interrupt pin 1 : = RencoderpinA = digital 3
  attachInterrupt(0, LwheelSpeed, CHANGE); // Trigger Interrupt on Change
  attachInterrupt(1, RwheelSpeed, CHANGE);
}

void LwheelSpeed()
{
  int Lstate = digitalRead(Lencoder0pinA);
  if((Lencoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(Lencoder0pinB);
    if(val == LOW && LDirection)
    {
      LDirection = false; //Reverse
    }
    else if(val == HIGH && !LDirection)
    {
      LDirection = true;  //Forward
    }
  }
  Lencoder0PinALast = Lstate;

  if(!LDirection)  Lduration++;
  else  Lduration--;

}

void RwheelSpeed()
{
  int Rstate = digitalRead(Rencoder0pinA);
  if((Rencoder0PinALast == LOW) && Rstate==HIGH)
  {
    int val = digitalRead(Rencoder0pinB);
    if(val == LOW && RDirection)
    {
      RDirection = false; //Reverse
    }
    else if(val == HIGH && !RDirection)
    {
      RDirection = true;  //Forward
    }
  }
  Rencoder0PinALast = Rstate;

  if(!RDirection)  Rduration++;
  else  Rduration--;

}





//////////////// Unused //////////////

//unsigned long last_time;
//double Lspeed, Rspeed;
//double Ltotal_error, Rtotal_error;
//double Lerror, Rerror;
//double Llast_error, Rlast_error;
//double Ldelta_error, Rdelta_error;


//void PID_Control(){
//T
//  unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
//
//  int delta_time = current_time - last_time; //delta time interval 
//
//  // Sould compute new PWM value
//  if (delta_time >= T){
//
//    // 1. Convert duration to velocity (r.p.s)
//    Lspeed = (Lduration - Lduration_last)/(delta_time/1000*REDUCTION_RATIO*ENCODER_CPR);
//    Rspeed = (Rduration - Rduration_last)/(delta_time/1000*REDUCTION_RATIO*ENCODER_CPR);
//
//   
//    // 2. Error = setpoint - feedback
//    Lerror = Left_PWM - Lspeed; // we should get sensed_output = encoderLeft, encoderRight
//    Rerror = Right_PWM - Rspeed; // we should get sensed_output = encoderLeft, encoderRight
//
//    // 3. For Integral term : summantion
//    Ltotal_error += Lerror; //accumalates the error - integral term  
//    Rtotal_error += Rerror; //accumalates the error - integral term
//
//    // 4. For Differentiation term : subtraction
//    Ldelta_error = Lerror - Llast_error; //difference of error for derivative term  // Differentiation Term
//    Rdelta_error = Rerror - Llast_error; //difference of error for derivative term
//
//
//    // 5. Restrict Integral term.
////    if (Ltotal_error >= max_control) Ltotal_error = max_control;
////    else if (Ltotal_error <= min_control) Ltotal_error = min_control;
////
////    if (Rtotal_error >= max_control) Ltotal_error = max_control;
////    else if (Rtotal_error <= min_control) Ltotal_error = min_control;
//
//
//    // 6. Restrict the magnitude of control signal.
//    Lcontrol_signal = LKp*Lerror + (LKi)*Ltotal_error*T + (LKd)*(Ldelta_error/T); //PID control compute
//    if (Lcontrol_signal >= max_control) Lcontrol_signal = max_control;
//    else if (Lcontrol_signal <= min_control) Lcontrol_signal = min_control;
//
//    Rcontrol_signal = RKp*Rerror + (RKi)*Rtotal_error*T + (RKd)*(Rdelta_error/T); //PID control compute
//    if (Rcontrol_signal >= max_control) Rcontrol_signal = max_control;
//    else if (Rcontrol_signal <= min_control) Rcontrol_signal = min_control;
//
//    // 7. Update last error (for next delta_error)
//    Llast_error = Lerror;
//    Rlast_error = Rerror;
//
//    // 8. Update last duration (for next speed)
//    Lduration_last = Lduration;
//    Rduration_last = Rduration;
//
//    // 9. Update last time (for next iteration)
//    last_time = current_time;
//
//  } 
//}
