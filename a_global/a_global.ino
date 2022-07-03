///////////////
//Need Kalman
//Need aborts
//Needs Data recording
//////////////
#include "PWM.hpp"
#include <Servo.h>
//IMU
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SD.h>
#include <SPI.h>



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


//Sd Card
bool record_data = true;
const int chipSelect = BUILTIN_SDCARD;
int test_number = 0;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

//Variables
//Magic Numbers
  const float rad2deg = 57.2958;
  const float deg2rad = 0.01745328;
  const float standard_gravity = 9.80665; 
  const float g2ms2 = standard_gravity;
  const float pi = 3.1415926;
  const float analog2voltage = (3.3 / 1023.0);
  //switches
  int s1,s2,s3;
  //Misc
  float batt_v;
  int stageinflight =0;
  

  //Motors
  float motor_output[4];
  
  //Receiver 
  float rc_ctrl[5];

  //IMU
  int16_t  ax,ay,az,gx,gy,gz;
  float acc[4];
  float gyro[3];
  float Cb2i[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float Cb2i_acc[3][3];
  float gyro_cal[3];
  //Control Law
  float Cb2i_target[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float Cb2i_gyro[3][3];
  float kp[3] = {80,80,200};
  float ki[3] = {0.01,0.01,0.01};  
  float kd[3] = {1200,1200,0};
  float p[3];
  float i[3];
  float d[3];
  float prev_i[3];
  float prev_error[3];
  float pid_output[3];
  float euler_error[3];

    


  //timers
  float prev_time_led,loop_timer,current_t,dt,data_time,start_time;
  float led_timer_test = 1000;
  float led_timer_flight =250;
