///////////////
//Need receiver implementation
//Need IMU Implementation
//Need PD controller for quat
//Need aborts
//Needs testing
//////////////

#include <Wire.h>
#include <Servo.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>



// I2c
#define scl 19;
#define sda 18;
// Reciever
#define ch1 37;
#define ch2 38;
#define ch3 33;
#define ch4 28;
#define ch5 29;
// Motors 
Servo m1;
Servo m2;
Servo m3;
Servo m4;
// Misc
#define led 23;

//Variables




int state;
long loop_timer;

  //Motors
  float full_t = 1500;
  float min_t = 1000;
  float m1_output,m2_output,m3_output,m4_output;
  
  
  //IMU
  int gyro_x, gyro_y, gyro_z;
  long acc_x, acc_y, acc_z, acc_total_vector;
  long gyro_x_cal, gyro_y_cal, gyro_z_cal;
  long loop_timer;
  int temperature;

  //PID
  float output_pitch,output_roll_pid,output_yaw_pid;

void setup() {
  
  Serial.begin(57600);     
  delay(250);
  while(!Serial);
  Serial.begin(115200);
  Serial1.begin(57600);
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

  // Motor pinouts 
  m1.attach(1);
  m2.attach(4);
  m3.attach(12);
  m4.attach(26);
  m1.writeMicroseconds(1000); //set the motors to lowest power setting
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  loop_timer = micros();                                               //Reset loop timer
}

void loop() {
  // put your main code here, to run repeatedly:
  imu_update();
  pid_update();
  

  m1.writeMicroseconds(m1_output);
  m2.writeMicroseconds(m1_output);
  m3.writeMicroseconds(m1_output);
  m4.writeMicroseconds(m1_output);
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();          //Reset the loop timer
}

void state(){
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
    if (throttle > 1800) throttle = 1800;
    if (throttle < 1000) throttle = 1000;
    call_pid(); //calls the pid values
    m1_output =throttle+output_y_pid+output_pitch_pid+output_roll_pid;
    m2_output =throttle-output_y_pid+output_pitch_pid-output_roll_pid;
    m3_output =throttle+output_y_pid-output_pitch_pid-output_roll_pid;
    m4_output =throttle-output_y_pid-output_pitch_pid+output_roll_pid;

    if (m1_output < mint) m1_output = min_t; //Keep the motors running.
    if (m2_output < mint) m2_output = min_t;                                         
    if (m3_output < mint) m3_output = min_t;                                         
    if (m4_output < mint) m4_output = min_t;  

   
  else if (state == -1){ //testing
    m1_output = throttle;
    m2_output = throttle;
    m3_output = throttle;
    m4_output = throttle;
  }


}



pid_update(){
  
  bool need_code // placeholder so code will still run
  needs_code = true;
}

imu_calibrate(){
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times                           
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;
  bool needs_code // placeholder so code will still run    
  need_code = true; //need to add offsets for accelerometer
}

imu_update(){
  bool needs_code // placeholder so code will still run
}
  needs_code = true; //need to decide what teensy library to use and implement quaternions
}
check_abort(){
  bool needs_code // placeholder so code will still run
  needs_code = true;
}
