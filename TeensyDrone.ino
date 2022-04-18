///////////////
//Need Pid Values
//!*! ERROR !*! State Loop with control loop in it crashes the teensy lol
//Need Filter of some kind
//Need aborts
//Needs testing
//////////////
#include "Control.hpp"
#include "Allocator"
#include "strapdown_analyitcs.hpp"
#include <MPU6050.h>
#include "PWM.hpp"
#include <Wire.h>
#include <Servo.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
//IMU
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

// Reciever
PWM ch1(37);
PWM ch2(38);
PWM ch2(36);
PWM ch3(33);
PWM ch4(28);
PWM ch5(29);
// Motors 
Servo m0;
Servo m1;
Servo m2;
Servo m3;
// Misc
#define led 23

//Variables
  int s1;
  float batt_v;
  int stage_inflight =0;


  //Motors
  float motor_outputs[4];
  
  //Receiver 
  float rc_cntrl[5];

  //IMU
  int16_t  ax,ay,az,gx,gy,gz;
  float acc[3];
  float gyro[3];
  float Cb2i[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float gyro_cal[3];
  //ControlLaw
  float Cb2i_target[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float kp[3] = {0,0,0};
  float ki[3] = {0,0,0};
  float kd[3[ = {0,0,0};
  float prev_i[3];
  float prev_error[3];
  float pid_output[3];

    


  //timers
  float prev_time_led,loop_timer,current_t,dt,data_time,start_time;
  float led_timer_test = 1000;
  float led_timer_flight =250;


  //magic numbers
  const float pi = 3.1415926;
  const float analog2voltage = (3.3 / 1023.0);

void setup() {

  Serial.begin(57600);     
  delay(250);
  
  Wire.begin();
  imu_startup();
  imu_calibrate();
  
  pinMode(led,OUTPUT);
  
  //Reciever Setup
  ch1.begin(true); // ch1 on pin 2 reading PWM HIGH duration
  ch2.begin(true); // ch2 on pin 3 reading PWM HIGH duration
  ch3.begin(true); // ch3 on pin 18 reading PWM HIGH duration
  ch4.begin(true); // ch4 on pin 19 reading PWM HIGH duration
  ch5.begin(true); // ch5 on pin 20 reading PWM HIGH duration
  
  // Motor pinouts 
  m1.attach(3);
  m2.attach(4);
  m3.attach(10);
  m4.attach(26);
  m1.writeMicroseconds(1000); //set the motors to lowest power setting
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  loop_timer = micros();                                               //Reset loop timer
  prev_time_led = millis();
  start_time = prev_time_led;

}


////////////////////////////////
//Main Loop
///////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  imu_update();
  receiver_update();
  nav();
  stageinflight();
  m0.writeMicroseconds(motor_output[0]);
  m1.writeMicroseconds(motor_output[1]);
  m2.writeMicroseconds(motor_output[2]);
  m3.writeMicroseconds(motor_output[3]);

//  print_();
  while(micros() - loop_timer < 4000); //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  current_t = micros();
  dt = (current_t - loop_timer)/1000000;
  loop_timer = micros();          //Reset the loop timer
  data_time = (millis()-start_time)/1000;
}

///////
//IMU//
///////
void imu_update(){
  //Read Raw data 
  mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);  //magical i2c library shit that i dont understand
  acc[0] = float(ax)/4096*g2ms2; //Convert int16 raw (LSB) to float m/s2
  acc[1] = float(ay)/4096*g2ms2;
  acc[2] = float(az)/4096*g2ms2;
  mag_acc = sqrt((ax*ax)+(ay*ay)+(az*az));
  mag_acc = sqrt((acc[0]*acc[0])+(acc[1]*acc[1])+(acc[2]*acc[2]));
  gyro[0] = (float(gx)-gx_cal)/65.5*deg2rad; //Convert int16 raw (LSB) to float rad/s
  gyro[1] = (float(gy)-gy_cal)/65.5*deg2rad;
  gyro[2] = (float(gz)-gz_cal)/65.5*deg2rad;    
}


void imu_calibrate(){
  float init_acc[4];
  int n = 2000;
  for (int cal_int = 0; cal_int < n ; cal_int ++){                  //Run this code n times 
    mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);                           
    gyro_cal[0] += gx;
    gyro_cal[1] += gy;
    gyro_cal[2] += gz;        
    init_acc[0] += ax;
    init_acc[1] += ay;
    init_acc[2] += az;
                                          
    delay(4);                                                         //(250hz)
  }
  gyro_cal[0] /= n;                                                  //Divide the gyro_x_cal variable by n to get the avarage offset
  gyro_cal[1] /= n;                                                  
  gyro_cal[2] /= n;
  init_acc[0] /= n;
  init_acc[1] /= n;
  init_acc[2] /= n;
  init_acc[3] = sqrt((init_acc[0]*init_acc[0])+(init_acc[1]*init_acc[1])+(init_acc[2]*init_acc[2]));
  
  imu_init(init_acc,Cb2i_gyro);
  
  
}
void imu_startup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(1);
}


////////////////////////////////
//Receiver
///////////////////////////////

void receiver_update(){
  rc_ctrl[0] = ch1.getValue(); //pitch
  if (rc_ctrl[0] < 1000) rc_ctrl[0] = 1000;
  if (rc_ctrl[0] > 2000) rc_ctrl[0] = 2000;
  rc_ctrl[1] = ch2.getValue(); //roll
  if (rc_ctrl[1] < 1000) rc_ctrl[1] = 1000;
  if (rc_ctrl[1] > 2000) rc_ctrl[1] = 2000;
  rc_ctrl[2] = ch4.getValue(); //yaw
  if (rc_ctrl[2] < 1000) rc_ctrl[2] = 1000;
  if (rc_ctrl[2] > 2000) rc_ctrl[2] = 2000;
  rc_ctrl[3] = ch3.getValue(); //throttle
  if (rc_ctrl[3] < 1000) rc_ctrl[3] = 1000;
  if (rc_ctrl[3] > 2000) rc_ctrl[3] = 2000;
  rc_ctrl[4] = ch5.getValue(); //switch
  if (rc_ctrl[4] < 1000) rc_ctrl[4] = 1000;
  if (rc_ctrl[4] > 2000) rc_ctrl[4] = 2000;
}

void print_(){
  Serial.print(m1_output);
  Serial.print(" ");
  Serial.println(m3_output);
}



void nav(){
  gyro2dcm(gyro,Cb2i_gyro,dt);
  //sketchy comp filter
  acc_init(acc[4],Cb2i_acc[3][3]);
  complimentary_filter(0.996,Cb2i_gyro,Cb2i_acc,Cb2i);
}
//stage of flight commands (should be final part of code)
////////////////////////////////
//STATE LOOP
///////////////////////////////
void state_loop(){
  batt_v = analogRead(7);
  if (r_switch >1500){
    state =1;
  }
  else{
    state = 0;

  }

  if (state ==0){ //idle
    m1_output = 0;
    m2_output = 0;
    m3_output = 0;
    m4_output = 0;
    if (s1 != 1){
      m1.writeMicroseconds(m1_output);
      m2.writeMicroseconds(m2_output);
      m3.writeMicroseconds(m3_output);
      m4.writeMicroseconds(m4_output);
      digitalWrite(led,LOW);
      imu_calibrate();
      s1=1;
    }
  }
  if (state ==1){ //Flight
    s1=0;   
    control(Cb2i_target,kp,ki,kd,prev_i,prev_error,rc_ctrl,pid_output);
    allocation(rc_ctrl,pid_output,motor_outputs);
    //Led blink
    if ((millis()-prev_time_led)>led_timer_flight) digitalWrite(led,HIGH);
    if ((millis()-prev_time_led)>led_timer_flight*2){
      digitalWrite(led,LOW);
      prev_time_led = millis();
      if ((millis()-prev_time_led)>led_timer_flight*2){
        digitalWrite(led,LOW);
        prev_time_led = millis();
      }
    }
  }

  if (state == -1){ //testing
    control_loop();
    m1_output = rc_crl[4];
    m2_output = rc_crl[4];
    m3_output = rc_crl[4];
    m4_output = rc_crl[4];
    if ((millis()-prev_time_led)>led_timer_test) digitalWrite(led,HIGH);
    if ((millis()-prev_time_led)>led_timer_test*5){
      digitalWrite(led,LOW);
      prev_time_led = millis();
    }
  }
}
