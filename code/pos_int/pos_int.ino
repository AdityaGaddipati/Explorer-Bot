#include <ros.h>
#include <std_msgs/Int64.h>
#include "motion.c"
#include "interrupt.c"

volatile long left_count = 0;
volatile long right_count = 0;
volatile bool right_chB;
volatile bool left_chB;

ros::NodeHandle  encoder;

std_msgs::Int64 msg;
ros::Publisher chatter("chatter", &msg);


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
  cli();
  motion_pin_config();
  encoder_pin_config();  
  encoder_interrupt_init();
  sei();
}

void loop() {
  forward();
  if(right_count>420)
  {
      stp();
      while(1);
  }/*
  msg.data = right_count;
  chatter.publish(&msg);
  encoder.spinOnce();*/
  delay(100);
}
