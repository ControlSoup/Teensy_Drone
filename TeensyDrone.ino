///////////////
//Need Pid Values
//!*! ERROR !*! State Loop with control loop in it crashes the teensy lol
//Need Filter of some kind
//Need aborts
//Needs testing
//////////////
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
PWM ch2(36);
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

//magic numbers
  const float deg2rad = 0.01745327777;
  const float g2ms2 = 9.8055;
  const float rad2deg = 1/deg2rad;
  const float pi = 3.1415926;
  const float analog2voltage = (3.3 / 1023.0);
//switchs

int s1;


float batt_v;
int state =0;

  
  //Motors
  float full_t = 1500;
  float min_t = 1000;
  float m1_output,m2_output,m3_output,m4_output;

  //Receiver 
  float r_roll,r_pitch,r_yaw,r_throttle,r_switch;
  
  //IMU
  float comp_gain = 0.98;
  int16_t  ax,ay,az,gx,gy,gz;
  float acc[3];
  float mag_acc;
  float gyro[3];
  float scew_sym[3][3]={{0,0,0},{0,0,0},{0,0,0}};
  float Cb2i_gyro[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float Cb2i_acc[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float Cb2i[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float Cb2i_dot[3][3];
  float gx_cal, gy_cal, gz_cal;
  int temperature;

  //ControlLaw
  
  float Cb2i_target[3][3] = {{1,0,0},{0,1,0},{0,0,1}} ;
  float Cb2i_error[3][3] = {{0,0,0},{0,0,0},{0,0,0}} ;
  
    //Yaw
    float output_yaw,p_yaw,i_yaw,prev_yaw_error,yaw_error;
      const float kp_yaw = 200;
      const float ki_yaw = 0;
      //Pitch
    float output_pitch,p_pitch,i_pitch,d_pitch,prev_pitch_error,pitch_error;
      const float kp_pitch = 90;
      const float ki_pitch = 0;
      const float kd_pitch = 0;
    //Roll
    float output_roll,p_roll,i_roll,d_roll,prev_roll_error,roll_error;
      const float kp_roll = 90;
      const float ki_roll = 0;
      const float kd_roll = 0;
    
   
 
  //timers
  float prev_time_led,loop_timer,current_t,dt,data_time,start_time;
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
  start_time = prev_time_led;
  
}



////////////////////////////////
//Main Loop
///////////////////////////////

void loop() {
  imu_update();
  receiver_update();
  state_loop();
  m1.writeMicroseconds(m1_output);
  m2.writeMicroseconds(m2_output);
  m3.writeMicroseconds(m3_output);
  m4.writeMicroseconds(m4_output);
//  print_();
  while(micros() - loop_timer < 4000); //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  current_t = micros();
  dt = (current_t - loop_timer)/1000000;
  loop_timer = micros();          //Reset the loop timer
  data_time = (millis()-start_time)/1000;
  Serial.println(batt_v);
  
  
}


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
  check_abort(); //Abort conditions
  
  if (state ==0){ //idle
    m1_output = 0;
    m2_output = 0;
    m3_output = 0;
    m4_output = 0;
    m1.writeMicroseconds(m1_output);
    m2.writeMicroseconds(m2_output);
    m3.writeMicroseconds(m3_output);
    m4.writeMicroseconds(m4_output);

    digitalWrite(led,LOW);
    if (s1 != 1){
      imu_calibrate();
      s1=1;
    }
  }
  

  if (state ==1){ //Flight
    s1=0;
    if (r_throttle > 1800) r_throttle = 1800;
    if (r_throttle < 1000) r_throttle = 1000;
    control_loop();
    m1_output =r_throttle-output_yaw-output_pitch-output_roll;
    m2_output =r_throttle+output_yaw+output_pitch+output_roll;
    m3_output =r_throttle-output_yaw-output_pitch+output_roll;
    m4_output =r_throttle+output_yaw+output_pitch-output_roll;
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
   
  if (state == -1){ //testing
    control_loop();
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


//// Common to all control algs
//typedef struct _control_in{
//  // input stuff
//} control_in;
//
//typedef struct _control_out{
//  // Output stuff
//} control_out;
//
//// Control algorithm: ALPHA
//typedef struct _control_alpha_internal{
//  float k_p[3];
//  float k_d[3];
//  float k_i[3];
//  float i_val[3];
//} control_alpha_internal;
//
//control_alpha_internal ctrl = {.k_p = {1.0, 2.0, 3.0},
//                         .k_d = {0.5, 0.5, 0.5},
//                         .k_i = {0.1, 0.2, 0.3},
//                         .i_val = {0.0, 0.0, 0.0}};
//
//control_alpha_internal ctrl;
////ctrl.k_p[0] = 1.0;
////ctrl.k_p[1] = 2.0;
//
//control_out control_alpha(control_alpha_internal ctrl, control_in ctrl_in){
//  control_out ctrl_out;
//
//  //Do stuff
//  
//  return ctrl_out;
//}
// end control algorithm ALPHA

void control_loop(){//seperate and only use needed argumented
  //DCM error  Cb2i_error = Cb2i_target * Cb2i^T
  Cb2i_error[0][0] = Cb2i_target[0][0]*Cb2i[0][0]+Cb2i_target[0][1]*Cb2i[0][1]+Cb2i_target[0][2]*Cb2i[0][2];
  Cb2i_error[1][0] = Cb2i_target[1][0]*Cb2i[0][0]+Cb2i_target[1][1]*Cb2i[0][1]+Cb2i_target[1][2]*Cb2i[0][2];
  Cb2i_error[2][0] = Cb2i_target[2][0]*Cb2i[0][0]+Cb2i_target[2][1]*Cb2i[0][1]+Cb2i_target[2][2]*Cb2i[0][2];

  Cb2i_error[0][1] = Cb2i_target[0][0]*Cb2i[1][0]+Cb2i_target[0][1]*Cb2i[1][1]+Cb2i_target[0][2]*Cb2i[1][2];
  Cb2i_error[1][1] = Cb2i_target[1][0]*Cb2i[1][0]+Cb2i_target[1][1]*Cb2i[1][1]+Cb2i_target[1][2]*Cb2i[1][2];
  Cb2i_error[2][1] = Cb2i_target[2][0]*Cb2i[1][0]+Cb2i_target[2][1]*Cb2i[1][1]+Cb2i_target[2][2]*Cb2i[1][2];
  
  Cb2i_error[0][2] = Cb2i_target[0][0]*Cb2i[2][0]+Cb2i_target[0][1]*Cb2i[2][1]+Cb2i_target[0][2]*Cb2i[2][2];
  Cb2i_error[1][2] = Cb2i_target[1][0]*Cb2i[2][0]+Cb2i_target[1][2]*Cb2i[2][1]+Cb2i_target[1][2]*Cb2i[2][2];
  Cb2i_error[2][2] = Cb2i_target[2][0]*Cb2i[2][0]+Cb2i_target[2][1]*Cb2i[2][1]+Cb2i_target[2][2]*Cb2i[2][2];
  
  //DCM to Euler
  roll_error = atan(-Cb2i_error[2][0]/sqrt(1-(Cb2i_error[2][0]*Cb2i_error[2][0])));
  if (abs(Cb2i_error[2][0]) <0.999){
    pitch_error = atan(Cb2i_error[2][1]/Cb2i_error[2][2]);
    yaw_error = atan(Cb2i_error[1][0]/Cb2i_error[0][0]);
  }
  if (Cb2i_error[2][0] <= - 0.999){
    pitch_error = yaw_error - atan((Cb2i_error[1][2]-Cb2i_error[0][1])/(Cb2i_error[0][2]+Cb2i_error[1][1]));
  }
  if (Cb2i_error[2][0] >= 0.999){
    pitch_error =  pi + atan((Cb2i_error[1][2]+Cb2i_error[0][1])/(Cb2i_error[0][2]-Cb2i_error[1][1]))-yaw_error ;
  }
  if (r_pitch >1800){
    pitch_error +=2*deg2rad;
  }
  if (r_pitch<1300){
    pitch_error -=2*deg2rad;
  }
  if (r_roll >1800){
    roll_error += 2*deg2rad;
  }
  if (r_roll<1300){
    roll_error -= 2*deg2rad;
  }
  //Attitude Control Law
      //yaw
        p_yaw= kp_yaw*yaw_error;
        if (yaw_error >-3 or yaw_error <-3){
            i_yaw += ki_yaw*yaw_error;
        }
        else i_yaw =0;
        output_yaw = p_yaw + i_yaw ;
        
      
      //Pitch
        p_pitch= kp_pitch*pitch_error;
        if (pitch_error >-3 or pitch_error <3){
          i_pitch += ki_pitch*pitch_error;
        }
        else i_pitch =0;
        d_pitch = kd_pitch*(prev_pitch_error - pitch_error);
        prev_pitch_error = pitch_error;
        output_pitch = p_pitch + i_pitch + d_pitch ;

      //Roll (dont do a barrel roll)
        p_roll= kp_roll*roll_error;
        if (roll_error >-3 or roll_error <3){
          i_roll += ki_roll*roll_error;
        }
        else i_roll =0;
        d_roll = kd_roll*(prev_roll_error - roll_error);
        prev_roll_error = roll_error;
        output_roll = p_roll + i_roll + d_roll ;

  
        
}



void check_abort(){
}

///////
//IMU//
///////
float test;
void imu_update(){
  //Read Raw data 
 
  mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);  //magical i2c library shit that i dont understand
  acc[0] = float(ax)/4096*g2ms2; //Convert int16 raw (LSB) to float m/s2
  acc[1] = float(ay)/4096*g2ms2;
  acc[2] = float(az)/4096*g2ms2;
  mag_acc = sqrt((acc[0]*acc[0])+(acc[1]*acc[1])+(acc[2]*acc[2]));
  gyro[0] = (float(gx)-gx_cal)/65.5*deg2rad; //Convert int16 raw (LSB) to float rad/s
  gyro[1] = (float(gy)-gy_cal)/65.5*deg2rad;
  gyro[2] = (float(gz)-gz_cal)/65.5*deg2rad;

  //DCM Attitude Estimate
    //scew symetric based on Strap Down Analytics (Paul G. Savage) pg 3-52
    
    //scew_sym[0][0] =0
    scew_sym[0][1] = -gyro[2];
    scew_sym[0][2] = gyro[1];
    
    scew_sym[1][0] = gyro[2];
    //scew_sym[1][1] =0;
    scew_sym[1][2] = -gyro[0];
    
    scew_sym[2][0] = -gyro[1];
    scew_sym[2][1] = -gyro[0];
    //scew_sym[2][2] =0;
    
    //DCM_rate from body to inertial frame (matrix multiplication) 
    Cb2i_dot[0][0] = Cb2i_gyro[0][1]*scew_sym[1][0]+Cb2i_gyro[0][2]*scew_sym[2][0];
    Cb2i_dot[1][0] = Cb2i_gyro[1][1]*scew_sym[1][0]+Cb2i_gyro[1][2]*scew_sym[2][0];
    Cb2i_dot[2][0] = Cb2i_gyro[2][1]*scew_sym[1][0]+Cb2i_gyro[2][2]*scew_sym[2][0];

    Cb2i_dot[0][1] = Cb2i_gyro[0][0]*scew_sym[0][1]+Cb2i_gyro[0][2]*scew_sym[2][1];
    Cb2i_dot[1][1] = Cb2i_gyro[1][0]*scew_sym[0][1]+Cb2i_gyro[1][2]*scew_sym[2][1];
    Cb2i_dot[2][1] = Cb2i_gyro[2][0]*scew_sym[0][1]+Cb2i_gyro[2][2]*scew_sym[2][1];
    
    Cb2i_dot[0][2] = Cb2i_gyro[0][0]*scew_sym[0][2]+Cb2i_gyro[0][1]*scew_sym[1][2];
    Cb2i_dot[1][2] = Cb2i_gyro[1][0]*scew_sym[0][2]+Cb2i_gyro[1][1]*scew_sym[1][2];
    Cb2i_dot[2][2] = Cb2i_gyro[2][0]*scew_sym[0][2]+Cb2i_gyro[2][1]*scew_sym[1][2];
    
    // DCM Attitude Estimates

    
    //DCM rate
    Cb2i_gyro[0][0] += Cb2i_dot[0][0] *dt; 
    Cb2i_gyro[1][0] += Cb2i_dot[1][0] *dt; 
    Cb2i_gyro[2][0] += Cb2i_dot[2][0] *dt; 
    
    Cb2i_gyro[0][1] += Cb2i_dot[0][1] *dt; 
    Cb2i_gyro[1][1] += Cb2i_dot[1][1] *dt; 
    Cb2i_gyro[2][1] += Cb2i_dot[2][1] *dt; 
    
    Cb2i_gyro[0][2] += Cb2i_dot[0][2] *dt; 
    Cb2i_gyro[1][2] += Cb2i_dot[1][2] *dt; 
    Cb2i_gyro[2][2] += Cb2i_dot[2][2] *dt; 
    
    //Shitty attiude estimate from acc
    Cb2i_acc[2][0] = acc[0]/mag_acc;
    Cb2i_acc[2][1] = acc[1]/mag_acc;
    Cb2i_acc[2][2] = acc[2]/mag_acc;
    
    Cb2i_acc[1][0] = 0;
    Cb2i_acc[1][1] = Cb2i_acc[2][2]/sqrt((Cb2i_acc[2][1]*Cb2i_acc[2][1])+(Cb2i_acc[2][2]*Cb2i_acc[2][2]));
    Cb2i_acc[1][2] = -Cb2i_acc[2][1] /sqrt((Cb2i_acc[2][1]*Cb2i_acc[2][1])+(Cb2i_acc[2][2]*Cb2i_acc[2][2]));
    
    Cb2i_acc[0][0] = (Cb2i_acc[1][1]*Cb2i_acc[2][2])-(Cb2i_acc[1][2]*Cb2i_acc[2][1]);
    Cb2i_acc[0][1] = (Cb2i_acc[1][2]*Cb2i_acc[2][0])-(Cb2i_acc[1][0]*Cb2i_acc[2][2]);
    Cb2i_acc[0][2] = (Cb2i_acc[1][0]*Cb2i_acc[2][1])-(Cb2i_acc[1][1]*Cb2i_acc[2][0]);

    //Complimentary Filter
    Cb2i[0][0] = Cb2i_gyro[0][0]*comp_gain+(Cb2i_acc[0][0]*(1-comp_gain));
    Cb2i[1][0] = Cb2i_gyro[1][0]*comp_gain+(Cb2i_acc[1][0]*(1-comp_gain));
    Cb2i[2][0] = Cb2i_gyro[2][0]*comp_gain+(Cb2i_acc[2][0]*(1-comp_gain));
    
    Cb2i[0][1] = Cb2i_gyro[0][1]*comp_gain+(Cb2i_acc[0][1]*(1-comp_gain));
    Cb2i[1][1] = Cb2i_gyro[1][1]*comp_gain+(Cb2i_acc[1][1]*(1-comp_gain));
    Cb2i[2][1] = Cb2i_gyro[2][1]*comp_gain+(Cb2i_acc[2][1]*(1-comp_gain));
    
    Cb2i[0][2] = Cb2i_gyro[0][2]*comp_gain+(Cb2i_acc[0][2]*(1-comp_gain));
    Cb2i[1][2] = Cb2i_gyro[1][2]*comp_gain+(Cb2i_acc[1][2]*(1-comp_gain));
    Cb2i[2][2] = Cb2i_gyro[2][2]*comp_gain+(Cb2i_acc[2][2]*(1-comp_gain));
    
   
        
    
    
}

void imu_calibrate(){
  float ax_avg;
  float ay_avg;
  float az_avg;
  int n = 2000;
  for (int cal_int = 0; cal_int < n ; cal_int ++){                  //Run this code n times 
    mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);                           
    gx_cal += gx;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gy_cal += gy;                                              
    gz_cal += gz;        
    ax_avg += ax;
    ay_avg += ay;
    az_avg += az;
                                          
    delay(4);                                                         //(250hz)
  }
  gx_cal /= n;                                                  //Divide the gyro_x_cal variable by n to get the avarage offset
  gy_cal /= n;                                                  
  gz_cal /= n;
  ax_avg /= n;
  ay_avg /= n;
  az_avg /= n;
  int mag_ax_avg = sqrt((ax_avg*ax_avg)+(ay_avg*ay_avg)+(az_avg*az_avg));
  
  //Initalize Gyro (Strapdown) 6-4 
  
  Cb2i_gyro[2][0] = ax_avg/mag_ax_avg;
  Cb2i_gyro[2][1] = ay_avg/mag_ax_avg;
  Cb2i_gyro[2][2] = az_avg/mag_ax_avg;
  
  Cb2i_gyro[1][0] = 0;
  Cb2i_gyro[1][1] = Cb2i_gyro[2][2]/sqrt((Cb2i_gyro[2][1]*Cb2i_gyro[2][1])+(Cb2i_gyro[2][2]*Cb2i_gyro[2][2]));
  Cb2i_gyro[1][2] = -Cb2i_gyro[2][1] /sqrt((Cb2i_gyro[2][1]*Cb2i_gyro[2][1])+(Cb2i_gyro[2][2]*Cb2i_gyro[2][2]));
  
  Cb2i_gyro[0][0] = (Cb2i_gyro[1][1]*Cb2i_gyro[2][2])-(Cb2i_gyro[1][2]*Cb2i_gyro[2][1]);
  Cb2i_gyro[0][1] = (Cb2i_gyro[1][2]*Cb2i_gyro[2][0])-(Cb2i_gyro[1][0]*Cb2i_gyro[2][2]);
  Cb2i_gyro[0][2] = (Cb2i_gyro[1][0]*Cb2i_gyro[2][1])-(Cb2i_gyro[1][1]*Cb2i_gyro[2][0]);
  
  
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
  r_pitch = ch2.getValue();
  if (r_pitch < 1000) r_pitch = 1000;
  if (r_pitch > 2000) r_pitch = 2000;
  r_roll = ch1.getValue();
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

void print_(){
  Serial.print(m1_output);
  Serial.print(" ");
  Serial.println(m3_output);
}
