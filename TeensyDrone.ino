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
  float acc_x, acc_y, acc_z;
  float gyro_x,gyro_y,gyro_z;
  long gyro_x_cal, gyro_y_cal, gyro_z_cal;
  int temperature;

  //PID
  float output_pitch_pid,output_roll_pid,output_yaw_pid;

  //timers
  long prev_time_led,loop_timer;
  float led_timer_test = 1000;
  float led_timer_flight =250;




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
  loop_timer = micros();                                               //Reset loop timer
  prev_time_led = millis();
  Serial.println("test");
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
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
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
  bool needs_code; // placeholder so code will still run
  needs_code = true;
}


void check_abort(){
  bool needs_code; // placeholder so code will still run
  needs_code = true;
}

////////////////////////////////
//IMU
///////////////////////////////

void imu_update(){
  mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);   
  acc_x = float(ax)/4096*9.81;
  acc_y = float(ay)/4096*9.81;
  acc_z = float(az)/4096*9.81;
  gyro_x = float(gx);
  gyro_y = float(gy);
  gyro_z = float(gz);
  Serial.println(gx);
}


void imu_calibrate(){
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times                           
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;
  bool needs_code; // placeholder so code will still run    
  needs_code = true; //need to add offsets for accelerometer
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
