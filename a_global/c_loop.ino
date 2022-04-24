////////////////////////////////
//Main Loop
///////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  imu_update();
  receiver_update();
  nav();  
  flight_stage();
  m0.writeMicroseconds(motor_outputs[0]);
  m1.writeMicroseconds(motor_outputs[1]);
  m2.writeMicroseconds(motor_outputs[2]);
  m3.writeMicroseconds(motor_outputs[3]);
  

  while(micros() - loop_timer < 4000); //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  current_t = micros();
  dt = (current_t - loop_timer)/1000000;
  loop_timer = micros();          //Reset the loop timer
  data_time = (millis()-start_time)/1000;
}
