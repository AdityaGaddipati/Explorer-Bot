#include <ros.h>
//#include <std_msgs/Int64.h>
//#include<std_msgs/Int64MultiArray.h>
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
/*#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>*/
#include "motion.c"
#include "interrupt.c"

volatile long left_count = 0;
volatile long right_count = 0;
volatile bool right_chB;
volatile bool left_chB;
/*
long _currentLeftcount=0,_currentRightcount=0;
long _previousLeftcount=0,_previousRightcount=0;
int deltaLeft=0,deltaRight=0;
double dt=0.023;
double v_left=0,v_right=0,Vx=0,Vth=0;
double x=0,y=0,th=0;
double dist_per_count = 0.000123;
double wheel_axle_length = 0.24;
*/


ros::NodeHandle  encoder;

void callback(const geometry_msgs::Twist& msg){
  if(msg.linear.x>0){
    if(msg.angular.z>0)
      soft_left();
    else if(msg.angular.z==0)
      forward();
    else
      soft_right();
  }
  else if(msg.linear.x==0){
    if(msg.angular.z>0)
      left();
    else if(msg.angular.z==0)
      stp();
    else
      right();
  }
  else{
    if(msg.angular.z>0)
      soft_left_2();
    else if(msg.angular.z==0)
      back();
    else
      soft_right_2();
  }
}

//std_msgs::Int64 msg;
//std_msgs::Int64MultiArray msg;
geometry_msgs::Vector3 msg;
ros::Publisher chatter("right",&msg);


//geometry_msgs::Quaternion odom_quat;

//ros::Time current_time = encoder.now();
//ros::Time last_time = encoder.now();
//geometry_msgs::TransformStamped odom_trans;
//tf::TransformBroadcaster _brdcst;

//nav_msgs::Odometry odom;
//std_msgs::Int64 msg2;
//ros::Publisher chatter2("left",&msg2);


ISR(INT2_vect){
  left_chB = PIND & 0x08;
  left_count -= left_chB ? -1 : +1;
}

ISR(INT4_vect){
  right_chB = PINE & 0x20;
  right_count += right_chB ? -1 : +1;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);


void setup() {
  encoder.initNode();
  encoder.advertise(chatter);
  encoder.subscribe(sub);
  //_brdcst.init(encoder);
  //encoder.advertise(chatter2);
  cli();
  motion_pin_config();
  timer4_init();
  velocity(150,150);
  encoder_pin_config();  
  encoder_interrupt_init();
  sei();
}

void loop() {
  msg.x = right_count;
  msg.y = left_count;
  //msg2.data = left_count;
  chatter.publish(&msg);
  encoder.spinOnce();
  /*
  if(right_count>2026){
    stp();
    while(1);
  }*/
  //chatter2.publish(&msg2);
  //encoder.spinOnce();
 
  delay(10);
}
/*
void compute_wheel_odom(){
      current_time = encoder.now();
      _currentLeftcount = left_count; 
      _currentRightcount = right_count; 

      deltaLeft = _currentLeftcount - _previousLeftcount;
      deltaRight = _currentRightcount - _previousRightcount;
      //dt = current_time.toSec() - last_time.toSec();
      //dt = deltaLeft*(0.6/1680);
      v_left = (deltaLeft*dist_per_count)/(dt);
      v_right = (deltaRight*dist_per_count)/dt; 

      Vx = (v_right + v_left)/2;
      Vth = (v_right - v_left)/wheel_axle_length;

      x += Vx*cos(th)*dt;
      y += Vx*sin(th)*dt;
      th = Vth*dt;
      odom_quat = tf::createQuaternionFromYaw(th);
      
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      _brdcst.sendTransform(odom_trans);
      
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = odom_quat;
      //odom_pub.publish(&odom);
      //encoder.spinOnce();
      //msg2.data = deltaRight;
      last_time = current_time;
      _previousLeftcount = _currentLeftcount;
      _previousRightcount = _currentRightcount;
    }*/
