#include <ros.h>
#include <ros/time.h>
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "motion.c"
#include "interrupt.c"
#include <PID_v1.h>

double input[2] = {0.22,0.22};
double output[2] = {120,120};
double setpoint[2] = {0.29,0.29};
double Kp[2] = {512,2048};
double Ki[2] = {1024,8192};
double Kd[2] = {0,0};

PID PID_Left(&input[0], &output[0], &setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID PID_Right(&input[1], &output[1], &setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);

volatile long left_count = 0;
volatile long right_count = 0;
volatile bool right_chB;
volatile bool left_chB;

long _currentLeftcount=0,_currentRightcount=0;
long _previousLeftcount=0,_previousRightcount=0;
int deltaLeft=0,deltaRight=0;
double current_time,last_time,dt;
double dist_per_count = 0.0005;
float v_left=0,v_right=0;

ros::NodeHandle  encoder;

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

void callback(const geometry_msgs::Twist& msg){
  if(msg.linear.x>0){
    if(msg.angular.z>0){
      setpoint[0] = 0.31;
      setpoint[1] = 0.31;
    }
    else if(msg.angular.z==0){
      //Kp[0] += Kp[0];
      Ki[0] += Ki[0];
      PID_Left.SetTunings(Kp[0],Ki[0],Kd[0]);
      PID_Right.SetTunings(Kp[1],Ki[1],Kd[1]);
    }
    else{
      setpoint[0] = 0.26;
      setpoint[1] = 0.26;
    }
  }
  else if(msg.linear.x==0){
    if(msg.angular.z>0){
      Kp[0]++; 
      PID_Left.SetTunings(Kp[0],Ki[0],Kd[0]);
      }
    else if(msg.angular.z==0)
      ;
    else{
      Kp[0]--; 
      PID_Left.SetTunings(Kp[0],Ki[0],Kd[0]);
    }
  }
  else{
    if(msg.angular.z>0){
      Kp[1]--;
      PID_Right.SetTunings(Kp[1],Ki[1],Kd[1]);  
    }
    else if(msg.angular.z==0){
      //Kp[0] /= 2;
      Ki[0] /= 2;
      PID_Left.SetTunings(Kp[0],Ki[0],Kd[0]);
      PID_Right.SetTunings(Kp[1],Ki[1],Kd[1]);
    }
    else{
      Kp[1]++;
      PID_Right.SetTunings(Kp[1],Ki[1],Kd[1]);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);

void setup() {
  encoder.initNode();
  encoder.advertise(chatter);
  encoder.subscribe(sub);
  delay(10);
  cli();
  motion_pin_config();
  timer4_init();
  velocity(120,0);
  encoder_pin_config();  
  encoder_interrupt_init();
  sei();
  forward();
  PID_Left.SetOutputLimits(0,255);
  PID_Left.SetSampleTime(100);
  PID_Left.SetMode(AUTOMATIC);
  PID_Right.SetOutputLimits(120,255);
  PID_Right.SetSampleTime(100);
  PID_Right.SetMode(AUTOMATIC);
  //Ki = 1;
}

void loop() {
  if(PID_Left.Compute() || PID_Right.Compute()){
    velocity(output[0],0);
    compute_velocity();
    msg.x = input[0];
    msg.y = output[0];
    msg.z = PID_Left.GetKi();
    chatter.publish(&msg);
    encoder.spinOnce();
  }
  delay(10);
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
      
      input[0] = abs(v_left);
      input[1] = abs(v_right);
      
      last_time = current_time;
      _previousLeftcount = _currentLeftcount;
      _previousRightcount = _currentRightcount;
    }
