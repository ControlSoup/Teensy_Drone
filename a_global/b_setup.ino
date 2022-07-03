void setup() {

  Serial.begin(57600);     
  delay(250);
  pinMode(led,OUTPUT);

  
  Wire.begin();
  digitalWrite(led,HIGH);  
  imu_startup();
  imu_calibrate();
  digitalWrite(led,LOW);
  delay(300);
  digitalWrite(led,HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(300);
  
  //Reciever Setup
  ch1.begin(true); // ch1 on pin 2 reading PWM HIGH duration
  ch2.begin(true); // ch2 on pin 3 reading PWM HIGH duration
  ch3.begin(true); // ch3 on pin 18 reading PWM HIGH duration
  ch4.begin(true); // ch4 on pin 19 reading PWM HIGH duration
  ch5.begin(true); // ch5 on pin 20 reading PWM HIGH duration

  //data_logger
  data_logger_setup();
  
  // Motor pinouts 
  m0.attach(2);
  m1.attach(4);
  m2.attach(10);
  m3.attach(26);
  m0.writeMicroseconds(1000); //set the motors to lowest power setting
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  loop_timer = micros();                                               //Reset loop timer
  prev_time_led = millis();
  start_time = prev_time_led;

}
