#include <ros.h>
#include <std_msgs/Int32.h>
#define ir_pin 4        // IR PIN Number
#define def_led 13      // DEFAULT LED that comes with arduino's PIN Number
// CREATING ROS NODE
ros::NodeHandle nh;

// IR INPUT VARIABLE THAT IS PUBLISHED 
std_msgs::Int32 ir_input;

// PUBLISHER CHATTER PUBLISHING TO TOPIC CHATTER 
ros::Publisher chatter("chatter", &ir_input);

// SETTING UP ROS NODE AND ARDUINO PINS - RUNS ONLY ONCE
void setup() {
  
  nh.initNode();
  nh.advertise(chatter); 
  pinMode(ir_pin,INPUT);
  pinMode(def_led,OUTPUT);//LED
}

// MAIN LOOP - KEEPS ON RUNNING
void loop() {

// WHEN ANYTHING IS TOO CLOSE
if(digitalRead(ir_pin)==LOW){
  // IR INPUT DATA MUST BE TURNED TO 1
  ir_input.data = 1;
  // TURNING THE DEFAULT LED ON ARDUINO TO HIGH
  digitalWrite(def_led,HIGH);
}

// WHEN NOTHING IS TOO CLOSE
else{
  // IR INPUT DATA MUST BE TURNED TO 0
  ir_input.data = 0;
  // TURNING THE DEFAULT LED ON ARDUINO TO LOW
  digitalWrite(def_led,LOW);
}

  // PUBLISH THE IR INPUT
  chatter.publish( &ir_input );
  
  // Using spinOnce() as it is already in a loop 
  nh.spinOnce();
  
  // Delaying so that output isn't spammed 
  delay(200);
}