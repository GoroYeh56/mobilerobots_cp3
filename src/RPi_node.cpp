#include "ros/ros.h"          // 加入ROS公用程序
#include "std_msgs/Int64.h"  // 所要publish的message header，在此是std_msgs package底下的String.msg
#include <sstream>
#include <iostream>


///////////// Check point 3 //////////////
#include <wiringPi.h>
#include <std_msgs/Int16.h>
//light receive pin 3
const short int lightpin = 3;
ros::Time previous_time;
ros::Time current_time;
ros::Publisher light_pub;
std_msgs::Int16 light_data;
unsigned short int light_rev = 0;


// TODO : Pack msg to 1 topic (typedef struct .left_PWM / .right_PWM)
//////////// Check point 2 ////////////////
// 1. Publish Topic: /Input_Left_PWM
//                   /Input_Right_PWM
std_msgs::Int64 input_left_number;
std_msgs::Int64 input_right_number;
ros::Publisher Left_pub;
ros::Publisher Right_pub;


//////////// DON'T NEED THIS CALLBACK NOW ///////////////
// void MUL_result_Callback(const std_msgs::Int64::ConstPtr& msg)
// {
//   ROS_INFO("message from Arduino: %ld", msg->data);
//   // std_msgs::Int64 input_number;  // 建立暫存的message，先將資料存入此變數，再進行publish
    
//     // static int num = 0;

//     // input_number.data = num;   // 寫入msg message中的data欄位 
//   std::cin>>input_number.data;
//     // key : put "delay" before ros::spinOnce(); to allow Arduino to handle data and send back.
//   RPi_pub.publish(input_number);
//     // loop_rate.sleep(); 
//   ROS_INFO("user's input is %ld", input_number.data); 
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RPi");  //一開始必須先初始化，指定node名稱為talker

  /* 該node與ROS系統通訊的存取點(handle)，建構子會初始化該node，
     當離開此scope，解構子會關閉該node */
  ros::NodeHandle n;     

  /* advertise()會將建立topic的資訊告訴master node，回傳一個Publisher物件(在此為chatter_pub)
     之後可使用該物件的 publish() 方法進行publish
     而預計publish的message為std_msgs package的String.msg (std::msgs::String)   
     指定的topic名稱為chatter
     10指的是message queue，若publish太快，超過1000個message，新publish的message會被捨棄
  */ 
  Left_pub = n.advertise<std_msgs::Int64>("Input_Left_PWM", 10);
  Right_pub = n.advertise<std_msgs::Int64>("Input_Right_PWM", 10);

  light_pub = n.advertise<std_msgs::Int16>("light_data", 1);

  // ros::Subscriber RPi_sub = n.subscribe("MUL_result", 10, MUL_result_Callback);  

  //use this command whithout sudo  set environment
  setenv("WIRINGPI_GPIOMEM", "1", 1);
  //library setup function
  wiringPiSetup () ;
  pinMode (lightpin, INPUT) ;


  ros::Rate loop_rate(10);   // 10Hz

  // int count = 0;
  while (ros::ok())
  {
  // std_msgs::Int64 input_number;  // 建立暫存的message，先將資料存入此變數，再進行publish
    
    // static int num = 0;

    // input_number.data = num;   // 寫入msg message中的data欄位 
  
    light_rev = digitalRead(lightpin) ;
    light_data.data = light_rev;
    ROS_INFO("light_receive : %d ", light_rev);
    light_pub.publish(light_data);
    // ros::spinOnce();
    // loop_rate.sleep();



  ROS_INFO("Input Left PWM: "); 
  std::cin>>input_left_number.data;
  ROS_INFO("Input Right PWM: "); 
  std::cin>>input_right_number.data;
    // key : put "delay" before ros::spinOnce(); to allow Arduino to handle data and send back.
  Left_pub.publish(input_left_number);
  Right_pub.publish(input_right_number);
    // loop_rate.sleep(); 
  ROS_INFO("user's left input is %ld", input_left_number.data); 
  ROS_INFO("user's right input is %ld", input_right_number.data); 
    // ros::spinOnce();   // 呼叫一次 callback function，在subscriber才有用
    // num++;
  loop_rate.sleep(); 

  }
  // ros::spin();
  return 0;
}

