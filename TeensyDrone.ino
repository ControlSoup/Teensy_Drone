#include <MPU6050.h>

///////////////
//Need receiver implementation
//Need IMU Implementation
//Need PD controller for quat
//Need aborts
//Needs testing
//////////////
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
PWM ch3(33);
PWM ch4(28);
PWM ch5(29);
// Motors 
Servo m1;
Servo m2;
Servo m3;
Servo m4;
// Misc
#define led 23

//Variables




int state =-1;

  
  //Motors
  float full_t = 1500;
  float min_t = 1000;
  float m1_output,m2_output,m3_output,m4_output;

  //Receiver 
  float r_roll,r_pitch,r_yaw,r_throttle,r_switch;
  
  //IMU
  int16_t  ax,ay,az,gx,gy,gz;
  float acc[3];
  float gyro[3];
  float scew_sym[3][3]={{0,0,0},{0,0,0},{0,0,0}};
  float Cb2i[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float Cb2i_dot[3][3];
  float gx_cal, gy_cal, gz_cal;
  float pitch;
  int temperature;

  //PID
  float output_pitch_pid,output_roll_pid,output_yaw_pid;

  //timers
  float prev_time_led,loop_timer,current_t,dt;
  float led_timer_test = 1000;
  float led_timer_flight =250;


  //magic numbers
  const float deg2rad = 0.01745327777;
  const float g2ms2 = 9.8055;


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
  m1.attach(1);
  m2.attach(4);
  m3.attach(10);
  m4.attach(26);
  m1.writeMicroseconds(1000); //set the motors to lowest power setting
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  current_t = micros();                                               //Reset loop timer
  prev_time_led = millis();
  Serial.println("Starting");
}



////////////////////////////////
//Main Loop
///////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  imu_update();
  receiver_update();
  pid_update();
  state_loop();

  m1.writeMicroseconds(m1_output);
  m2.writeMicroseconds(m2_output);
  m3.writeMicroseconds(m3_output);
  m4.writeMicroseconds(m4_output);

  
  while(micros() - loop_timer < 4000); //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  current_t = micros();
  dt = (current_t - loop_timer)/1000000;
  loop_timer = micros();          //Reset the loop timer
  
  
  
}


////////////////////////////////
//STATE LOOP
///////////////////////////////
void state_loop(){
  if (r_switch >1500){
    state =1;
  }
  else{
    state = 0;
  }
  check_abort(); //Abort conditions

  if (state ==0){ //idle
    m1_output = 0;
    m2_output = 0;
    m3_output = 0;
    m4_output = 0;
    digitalWrite(led,LOW);
  }

  else if (state ==1){ //Flight
    if (r_throttle > 1800) r_throttle = 1800;
    if (r_throttle < 1000) r_throttle = 1000;
    pid_update(); //calls the pid values
    m1_output =r_throttle+output_yaw_pid+output_pitch_pid+output_roll_pid;
    m2_output =r_throttle-output_yaw_pid+output_pitch_pid-output_roll_pid;
    m3_output =r_throttle+output_yaw_pid-output_pitch_pid-output_roll_pid;
    m4_output =r_throttle-output_yaw_pid-output_pitch_pid+output_roll_pid;

    if (m1_output < min_t) m1_output = min_t; //Keep the motors running.
    if (m2_output < min_t) m2_output = min_t;                                         
    if (m3_output < min_t) m3_output = min_t;                                         
    if (m4_output < min_t) m4_output = min_t;  
    if ((millis()-prev_time_led)>led_timer_flight) digitalWrite(led,HIGH);
    if ((millis()-prev_time_led)>led_timer_flight*2){
      digitalWrite(led,LOW);
      prev_time_led = millis();
    }
  }
   
  else if (state == -1){ //testing
    m1_output = r_throttle;
    m2_output = r_throttle;
    m3_output = r_throttle;
    m4_output = r_throttle;
    if ((millis()-prev_time_led)>led_timer_test) digitalWrite(led,HIGH);
    if ((millis()-prev_time_led)>led_timer_test*5){
      digitalWrite(led,LOW);
      prev_time_led = millis();
    }
    
    
  }

}




void pid_update(){
}


void check_abort(){
}

////////////////////////////////
//IMU
///////////////////////////////

void imu_update(){
  //Read Raw data 
  mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);  //magical i2c library shit that i dont understand
  acc[0] = float(ax)/4096*g2ms2; //Convert int16 raw (LSB) to float m/s2
  acc[1] = float(ay)/4096*g2ms2;
  acc[2] = float(az)/4096*g2ms2;
  gyro[0] = (float(gx)-gx_cal)/65.5*deg2rad; //Convert int16 raw (LSB) to float rad/s
  gyro[1] = (float(gy)-gy_cal)/65.5*deg2rad;
  gyro[2] = (float(gz)-gz_cal)/65.5*deg2rad;
  //DCM Attitude Estimate
    //scew symetric based on Strap Down Analytics (Paul G. Savage) pg 3-52
    
    //scew_sym[0][0] =0
    scew_sym[0][1] = -gyro[2]
    scew_sym[0][2] = gyro[1]
    
    scew_sym[1][0] = gyro[2]
    //scew_sym[1][1] =0
    scew_sym[1][2] = -gyro[0]
    
    scew_sym[2][0] = -gyro[1]
    scew_sym[2][1] = -gyro[0]
    //scew_sym[2][2] =0
    
    //DCM_rate from body to inertial frame (matrix multiplication) this could be faster if the product of 0's from scew symtric was not computed every time
    Cb2i_dot[0][0] = Cb2i[0][1]*scew_sym[1][0]+Cb2i[0][2]*scew_sym[2][0];
    Cb2i_dot[1][0] = Cb2i[1][1]*scew_sym[1][0]+Cb2i[1][2]*scew_sym[2][0];
    Cb2i_dot[2][0] = Cb2i[2][1]*scew_sym[1][0]+Cb2i[2][2]*scew_sym[2][0];

    Cb2i_dot[0][1] = Cb2i[0][0]*scew_sym[0][1]+Cb2i[0][2]*scew_sym[2][1];
    Cb2i_dot[1][1] = Cb2i[1][0]*scew_sym[0][1]+Cb2i[1][2]*scew_sym[2][1];
    Cb2i_dot[2][1] = Cb2i[2][0]*scew_sym[0][1]+Cb2i[2][2]*scew_sym[2][1];
    
    Cb2i_dot[0][2] = Cb2i[0][0]*scew_sym[0][2]+Cb2i[0][1]*scew_sym[1][2];
    Cb2i_dot[1][2] = Cb2i[1][0]*scew_sym[0][2]+Cb2i[1][2]*scew_sym[1][2];
    Cb2i_dot[2][2] = Cb2i[2][0]*scew_sym[0][2]+Cb2i[2][1]*scew_sym[1][2];
    
    // DCM Attitude Estimates

    Cb2i[0][0] += Cb2i_dot[0][0] *dt; 
    Cb2i[1][0] += Cb2i[1][0] *dt; 
    Cb2i[2][0] += Cb2i[2][0] *dt; 
    Cb2i[0][1] += Cb2i[0][1] *dt; 
    Cb2i[1][1] += Cb2i[1][1] *dt; 
    Cb2i[2][1] += Cb2i[2][1] *dt; 
    Cb2i[0][2] += Cb2i[0][2] *dt; 
    Cb2i[1][2] += Cb2i[1][2] *dt; 
    Cb2i[2][2] += Cb2i[2][2] *dt; 
    
    
}

void imu_calibrate(){
  
  int n = 2000;
  for (int cal_int = 0; cal_int < n ; cal_int ++){                  //Run this code n times 
    mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);                           
    gx_cal += gx;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gy_cal += gy;                                              
    gz_cal += gz;                                              
    delay(4);                                                         //(250hz)
  }
  gx_cal /= n;                                                  //Divide the gyro_x_cal variable by n to get the avarage offset
  gy_cal /= n;                                                  
  gz_cal /= n;
  Serial.print(" cal = ");
  Serial.println(gx_cal);
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
  r_pitch = ch1.getValue();
  if (r_pitch < 1000) r_pitch = 1000;
  if (r_pitch > 2000) r_pitch = 2000;
  r_roll = ch2.getValue();
  if (r_roll < 1000) r_roll = 1000;
  if (r_roll > 2000) r_roll = 2000;
  r_throttle = ch3.getValue();
  if (r_throttle < 1000) r_throttle = 1000;
  if (r_throttle > 2000) r_throttle = 2000;
  r_yaw = ch4.getValue();
  if (r_yaw < 1000) r_yaw = 1000;
  if (r_yaw > 2000) r_yaw = 2000;
  r_switch = ch5.getValue();
  if (r_switch < 1000) r_switch = 1000;
  if (r_switch > 2000) r_switch = 2000;
}
