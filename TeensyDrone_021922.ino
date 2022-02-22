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



// I2c
#define scl 19
#define sda 18
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
  int gyro_x, gyro_y, gyro_z;
  long acc_x, acc_y, acc_z;
  long gyro_x_cal, gyro_y_cal, gyro_z_cal;
  int temperature;

  //PID
  float output_pitch_pid,output_roll_pid,output_yaw_pid;

  //timers
  long prev_time_led,loop_timer;
  float led_timer_test = 1000;




void setup() {
  
  Serial.begin(57600);     
  delay(250);
  Wire.begin();
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.setClock(400000);
  imu_startup();
  imu_calibrate();
  pinMode(led,OUTPUT);
  pinMode(r_roll,INPUT);
  pinMode(r_pitch,INPUT);
  pinMode(r_yaw,INPUT);
  pinMode(r_switch,INPUT);
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
  m2.writeMicroseconds(m1_output);
  m3.writeMicroseconds(m1_output);
  m4.writeMicroseconds(m1_output);
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();          //Reset the loop timer
}


////////////////////////////////
//STATE LOOP
///////////////////////////////
void state_loop(){
//  if (r_switch >1500){
//    state =1;
//  }
//  else{
//    state = 0;
//  }
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
  bool needs_code; // placeholder so code will still run
  needs_code = true; //need to decide what teensy library to use and implement quaternions
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
  bool needs_code; // placeholder so code will still run
  needs_code = true; //need i2c startup and selections of settings 
}

////////////////////////////////
//Receiver
///////////////////////////////

void receiver_update(){
  r_pitch = ch1.getValue();
  r_roll = ch2.getValue();
  r_throttle = ch3.getValue();
  r_yaw = ch4.getValue();
  r_switch = ch5.getValue();
  Serial.println(r_throttle);
}
