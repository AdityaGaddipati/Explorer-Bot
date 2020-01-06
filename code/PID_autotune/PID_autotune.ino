#include <ros.h>
#include <ros/time.h>
//#include <std_msgs/Int64.h>
//#include<std_msgs/Int64MultiArray.h>
#include<geometry_msgs/Vector3.h>
#include "motion.c"
#include "interrupt.c"
#include <PID_AutoTune_v0.h>

double output = 187;
double input=0.28,setpoint = 0.28;

//int itr = 0;
//float ave=0,sum=0,dev=0;

PID_ATune aTune(&input, &output);

volatile long left_count = 0;
volatile long right_count = 0;
volatile bool right_chB;
volatile bool left_chB;

long _currentLeftcount=0,_currentRightcount=0;
long _previousLeftcount=0,_previousRightcount=0;
int deltaLeft=0,deltaRight=0;
double current_time,last_time,dt;
double dist_per_count = 0.0005;
//double v_left=0,v_right=0;
float v_left=0,v_right=0;

ros::NodeHandle  encoder;

//std_msgs::Int64 msg;
//std_msgs::Int64MultiArray msg;
geometry_msgs::Vector3 msg;
ros::Publisher chatter("right",&msg);

ISR(INT2_vect){
  left_chB = PIND & 0x08;
  left_count -= left_chB ? -1 : +1;
}

ISR(INT4_vect){
  right_chB = PINE & 0x20;
  right_count += right_chB ? -1 : +1;
}

void setup() {
  encoder.initNode();
  encoder.advertise(chatter);
  current_time = millis();
  last_time = millis();
  cli();
  motion_pin_config();
  timer4_init();
  velocity(254,254);
  encoder_pin_config();  
  encoder_interrupt_init();
  sei();
  forward();
  aTune.SetOutputStep(67);
  aTune.SetControlType(1);
  aTune.SetNoiseBand(0.01);
  aTune.SetLookbackSec(3);
}

void loop() {
  if(!aTune.Runtime()){
    velocity(int(output),int(output));
    msg.x = float(output);
    msg.y = input;
    chatter.publish(&msg);
  }
  else{
    msg.x = aTune.GetKp();
    msg.y = aTune.GetKi();
    msg.z = aTune.GetKd();
    chatter.publish(&msg);
  }
  encoder.spinOnce();
  compute_velocity();
  
  /*delay(2000);
  velocity(255,255);
  delay(2000);
  //delay(10);
  compute_velocity();
  msg.x = v_left;
  msg.y = v_right;
  chatter.publish(&msg);
  encoder.spinOnce();
  delay(10);*/
}

void compute_velocity(){
      current_time = millis();
      _currentLeftcount = left_count; 
      _currentRightcount = right_count; 

      deltaLeft = _currentLeftcount - _previousLeftcount;
      deltaRight = _currentRightcount - _previousRightcount;
      dt = (current_time - last_time)/1000;
      
      v_left = (deltaLeft*dist_per_count)/(dt);
      v_right = (deltaRight*dist_per_count)/dt; 
      input = v_left;
      
      last_time = current_time;
      _previousLeftcount = _currentLeftcount;
      _previousRightcount = _currentRightcount;
    }
