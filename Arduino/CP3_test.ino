
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

#define LtouchPin 12
#define RtouchPin 13
#define GoalTouchPin A1 // analog pin but input is okay!

#define REDUCTION_RATIO 120
#define ENCODER_CPR 16
#define T 100 // sampling period
#define max_setpoint 255
#define  min_setpoint 0

ros::NodeHandle nh;      // Input value from RPi.
double Left_PWM, Right_PWM; // PWM signals for Motor.
double Left_setpoint, Right_setpoint;
double Lcontrol_signal;
double Rcontrol_signal;

//////////////// TODO1. Add Light detector ./////////////////
int sensorPin = A0;    // select the input pin for the potentiometer
int Light_Value = 0;  // variable to store the value coming from the sensor

///////////////// TODO 2. Add Touch sensor detector //////////////
// int GoalTouchPin = A1; // analog pin but input is okay!


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

double LKp =1;
double LKi =5;
double LKd =0;
double RKp =1;
double RKi =5;
double RKd =0;

PID LeftPID(&Labs_duration, &Lcontrol_signal, &Left_setpoint, LKp, LKi, LKd, DIRECT);
PID RightPID(&Rabs_duration, &Rcontrol_signal, &Right_setpoint, RKp, RKi, RKd, DIRECT);

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


// Sampling time of PID controller
// int T = 100;



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

  // Touch Sensor (if TOUCHED -> input == HIGH )
  pinMode(LtouchPin,INPUT);
  pinMode(RtouchPin,INPUT);
  pinMode(GoalTouchPin, INPUT);
  
//  pinMode(GoalTouchPin, INPUT_PULLUP);

  //Initialize left/right wheel pwm values.
  Left_PWM = 0;
  Right_PWM = 0;
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


///////////////// TODO 3. Program Flow (Free RTOS?) //////////////
// 1. advance a little
// 2. check light sensor? (Detect and Determine Direction)
// 3. If touch: interrupt -> Reverse and Turn Left or Right
// 4. Compute PID and generate PWM value.
// Let say : Setpoint = 80 or 100 


// Four task here: 
// 1. Check Direction & run (drive motor)
// 2. Detect Light sensor
// 3. Reach Goal yet?
// 4. Touch sensor interrupt.


/////////////// Sensors Detecting /////////////////

int ReadLightSensor() {
  // read the value from the sensor:
  int val = analogRead(sensorPin);
  Serial.print("Light sensor ");
  Serial.println(val);
  return val;
  
}

bool Reach_Goal = false; //initial
int Ltouch_val;
int Rtouch_val;
int Goal_touch_val;

void Check_Touched(){
    Ltouch_val = digitalRead(LtouchPin);
    Rtouch_val = digitalRead(RtouchPin);
    Goal_touch_val = digitalRead(GoalTouchPin);

    if(Goal_touch_val==HIGH && Light_Value<50){ // Reach GOAL and should STOP!
        Reached_Goal = true; // should stop
    }else{
        // should reverse?
        if(Ltouch_val == HIGH || Rtouch_val == HIGH || Goal_touch_val==HIGH){
            Reverse();
            delay(500); // 0.5 second
            Turn_left(); // 30 degree
            delay(500); // 0.5 second
            /////// TODO : Optimize ////////
        }
    }
}


/////////////// Motion Control Needed /////////////////

///// Note: just change the Left_PWM / Right_PWM
///// the Threads would PID compute itself and drve motor!
///// Including convert Lsetpoint to abs(Left_PWM)
int LreverseSpeed = -80;
int RreverseSPeed = -80;

int LadvanceSpeed = 80;
int RadvanceSPeed = 80;

int LturnLeftSpeed = -70;
int RturnLeftSpeed = 70;

int LturnRightSpeed = 70;
int RturnRIghtSpeed = -70;

void Reverse(){
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,LOW) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,LOW) ;
    Left_PWM = LreverseSpeed;
    Right_PWM = RreverseSPeed;

}

void Advance(){
    digitalWrite(in1,LOW) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,LOW) ;
    digitalWrite(in4,HIGH) ;
    Left_PWM = LadvanceSpeed;
    Right_PWM = RadvanceSPeed;    
}

void Turn_left(){
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,LOW) ;
    digitalWrite(in3,LOW) ;
    digitalWrite(in4,HIGH) ;
    Left_PWM = LturnLeftSpeed;
    Right_PWM = RturnLeftSpeed;  
}

void Turn_Right(){
    digitalWrite(in1,LOW) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,LOW) ;
    Left_PWM = LturnRightSpeed;
    Right_PWM = RturnRightSpeed; 
}

void STOP(){
    Left_PWM = 0;
    Right_PWM = 0;    
    // or directly drive motor. (Uncomment Below)
    // analogWrite(enA, 0);
    // analogWrite(enB, 0);
}



void loop()
{

    if(Reach_Goal==true){
        STOP();
        controller.run();

    }
    else{
        // 1. Read Light Sensor.
        // Lighter : the value would be smaller
        Light_Value = ReadLightSensor();

        // 2. Forward or Turn right (search light source)
        if(Ligh_Value <= 450){
            Advance();
        }else{
            // Turn Right a little.
            Turn_Right();
        }

        // 3. Check TOUCHED?  => Should use Interrupt.
        Check_Touched();

        // 4. Compute PID value ( Lcontrol_signal & Rcontrol_signal) & Drive motors
        controller.run();
    }
}
   //// Determine the Direction.
//    Check_Direction();
   //// 3. Listen to ROS Topics
//    nh.spinOnce(); // No use or only listen to START and STOP  /Command (std::String.h)







////////////// Encoder ///////////// 
///////// Get Encoder Feedback (Interrupt)

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


////////////// NO USE in Checkpoint 3 /////////////////

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
  else if(Left_PWM==0 && Right_PWM==0){
    digitalWrite(in1,HIGH) ;
    digitalWrite(in2,HIGH) ;
    digitalWrite(in3,HIGH) ;
    digitalWrite(in4,HIGH) ; 
  }
  
}
