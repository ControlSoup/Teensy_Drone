///////////////
//Need Pid Values
//!*! ERROR !*! State Loop with control loop in it crashes the teensy lol
//Need Filter of some kind
//Need aborts
//Needs testing
//////////////
#include "PWM.hpp"
#include <Servo.h>
//IMU
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

// Reciever
PWM ch1(37);
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
//Magic Numbers
  const float rad2deg = 57.2958;
  const float deg2rad = 0.01745328;
  const float standard_gravity = 9.80665; 
  const float g2ms2 = standard_gravity;
  int s1;
  float batt_v;
  int stageinflight =0;
  

  //Motors
  float motor_outputs[4];
  
  //Receiver 
  float rc_ctrl[5];

  //IMU
  int16_t  ax,ay,az,gx,gy,gz;
  float acc[4];
  float gyro[3];
  float Cb2i[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float Cb2i_acc[3][3];
  float gyro_cal[3];
  //ControlLaw
  float Cb2i_target[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float Cb2i_gyro[3][3];
  float kp[3] = {90,90,200};
  float ki[3] = {0.001,0.001,0.001};
  float kd[3] = {10,10,0};
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
