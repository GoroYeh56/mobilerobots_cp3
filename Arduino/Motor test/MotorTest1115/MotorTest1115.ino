const int pwm = 5 ;  //initializing pin 2 as pwm
const int enB = 6 ;  //initializing pin 2 as pwm
const int in_1 = 8 ;
const int in_2 = 9 ;
const int in_3 = 10 ;
const int in_4 = 11 ;


//For providing logic to L298 IC to choose the direction of the DC motor 

void setup()
{
pinMode(pwm,OUTPUT) ;   //we have to set PWM pin as output
pinMode(in_1,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_2,OUTPUT) ;
pinMode(enB,OUTPUT) ;   //we have to set PWM pin as output
pinMode(in_3,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_4,OUTPUT) ;

}

void loop()
{
//For Clock wise motion , in_1 = High , in_2 = Low
//
//digitalWrite(in_1,LOW) ;
//digitalWrite(in_2,HIGH) ;
//analogWrite(pwm,100) ;

digitalWrite(in_3,LOW) ;
digitalWrite(in_4,HIGH) ;
analogWrite(enB,100) ;

//digitalWrite(in_1,HIGH) ;
//digitalWrite(in_2,LOW) ;
//analogWrite(pwm,100) ;
//
//digitalWrite(in_3,HIGH) ;
//digitalWrite(in_4,LOW) ;
//analogWrite(enB,100) ;



/*setting pwm of the motor to 255
we can change the speed of rotaion
by chaning pwm input but we are only
using arduino so we are using higest
value to driver the motor  */

//Clockwise for 3 secs
//delay(3000) ;     

////For brake
//digitalWrite(in_1,HIGH) ;
//digitalWrite(in_2,HIGH) ;
//delay(1000) ;
//
////For Anti Clock-wise motion - IN_1 = LOW , IN_2 = HIGH
//digitalWrite(in_1,LOW) ;
//digitalWrite(in_2,HIGH) ;
//delay(3000) ;
//
////For brake
//digitalWrite(in_1,HIGH) ;
//digitalWrite(in_2,HIGH) ;
//delay(1000) ;
 }
