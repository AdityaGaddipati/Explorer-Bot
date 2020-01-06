#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "motion.c"

ros::NodeHandle  EB;

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

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);

void setup() {
  
  EB.initNode();
  EB.subscribe(sub);
  cli();                      //disable all interrupts
  motion_pin_config();
  timer4_init();
  velocity(150,150);
  sei();                      //re-enable interrupts

}

void loop() {
  
  EB.spinOnce();

}
