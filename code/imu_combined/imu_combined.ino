#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include "motion.c"
#include "interrupt.c"
#include <PID_v1.h>


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <avr/interrupt.h>


#define OUTPUT_READABLE_QUATERNION

MPU6050 mpu;

// MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int itr = 1;
float yaw_offset = 0;

double input[2] = {0.22,0.22};
double output[2] = {120,120};
double setpoint[2] = {0.29,0.29};
double Kp[2] = {512,512};
double Ki[2] = {1024,1024};
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

ros::NodeHandle  EB;

geometry_msgs::Vector3 msg;
ros::Publisher enc("encoder_ticks",&msg);

geometry_msgs::Quaternion quat;
ros::Publisher rot("rotation",&quat);

geometry_msgs::Vector3 msg2;
ros::Publisher velo("velocity",&msg2);

ISR(INT2_vect){
  left_chB = PIND & 0x08;
  left_count -= left_chB ? -1 : +1;
}

ISR(INT4_vect){
  right_chB = PINE & 0x20;
  right_count += right_chB ? -1 : +1;
}

ISR(INT6_vect){
    mpuInterrupt = true;
}

void callback(const geometry_msgs::Vector3& msg){
  stp();
  setpoint[0] = abs(msg.x);
  setpoint[1] = abs(msg.y);
  if(msg.x>0)
    PORTA |= 0x02;
  else if(msg.x<0)
    PORTA |= 0x01;
  if(msg.y>0)
    PORTA |= 0x04;
  else if(msg.y<0)
    PORTA |= 0x08;
}

ros::Subscriber<geometry_msgs::Vector3> vel("motor_vel", callback);

void setup() {
  EB.initNode();
  EB.advertise(enc);
  EB.advertise(rot);
  EB.advertise(velo);
  EB.subscribe(vel);
  delay(10);
  cli();
  motion_pin_config();
  timer4_init();
  encoder_pin_config();  
  encoder_interrupt_init();
  sei();
  PID_Left.SetOutputLimits(0,255);
  PID_Left.SetSampleTime(100);
  PID_Left.SetMode(AUTOMATIC);
  PID_Right.SetOutputLimits(0,255);
  PID_Right.SetSampleTime(100);
  PID_Right.SetMode(AUTOMATIC);
  mpu_setup();
  //delay(2000);
}

void loop() {
  if(PID_Left.Compute() || PID_Right.Compute()){
    //velocity(output[0],output[1]);
    OCR4BL = output[0];
    OCR4AL = output[1];
    compute_velocity();
    msg.x = right_count;
    msg.y = left_count;
    //msg.z = PID_Right.GetKp();
    //msg.y = deltaLeft;
    //msg.z = dt;
    msg2.x = v_left;
    msg2.y = v_right;
    enc.publish(&msg);
    velo.publish(&msg2);
    EB.spinOnce();
  }
  mpu_data();
  EB.spinOnce();
  delay(30);
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

void mpu6050_interrupt_init(){
    DDRE  = DDRE & 0xBF;  //Set the direction of PE6 pin as input
    PORTE = PORTE | 0x40;
    EICRB = EICRB | 0x30;   // INT6(mpu6050) is set to trigger with rising edge
    EIMSK = EIMSK | 0x40;   // Enable Interrupt INT6
  }

  // ================================================================
  // ===                      INITIAL SETUP                       ===
  // ================================================================

  void mpu_setup() {
      cli();
      mpu6050_interrupt_init();
      sei();
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
      #endif

      // initialize serial communication
      // (115200 chosen because it is required for Teapot Demo output, but it's
      // really up to you depending on your project)
      Serial.begin(57600);
      while (!Serial); // wait for Leonardo enumeration, others continue immediately


      // initialize device
  //    Serial.println(F("Initializing I2C devices..."));
      mpu.initialize();

      // verify connection
  //    Serial.println(F("Testing device connections..."));
  //    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

      // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  //    while (Serial.available() && Serial.read()); // empty buffer again

      // load and configure the DMP
  //    Serial.println(F("Initializing DMP..."));
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(-6323);
      mpu.setYAccelOffset(-275);
      mpu.setZAccelOffset(1882);
      mpu.setXGyroOffset(-44);
      mpu.setYGyroOffset(34);
      mpu.setZGyroOffset(34);

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
    // turn on the DMP, now that it's ready
  //        Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
  //        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
  //        Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  //        Serial.print(F("DMP Initialization failed (code "));
  //        Serial.print(devStatus);
  //        Serial.println(F(")"));
      }
  }



  // ================================================================
  // ===                    MAIN PROGRAM LOOP                     ===
  // ================================================================

  void mpu_data() {
      // if programming failed, don't try to do anything
      if (!dmpReady) return;

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        EB.spinOnce();
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
  //        Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        quat.x = q.x;
        quat.y = q.y;
        quat.z = q.z;
        quat.w = q.w;
        rot.publish(&quat);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
  
    #endif

    #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
    #endif
      
      }
  }
