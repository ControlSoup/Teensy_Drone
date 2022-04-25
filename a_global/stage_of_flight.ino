////////////////////////////////
//STATE LOOP
///////////////////////////////
void flight_stage(){
  batt_v = analogRead(7);
  if (rc_ctrl[4] >1500){
    stageinflight =1;
  }
  else{
    stageinflight = 0;

  }

  if (stageinflight ==0){ //idle
    motor_output[0] = 0;
    motor_output[1] = 0;
    motor_output[2] = 0;
    motor_output[3] = 0;
    if (s1 != 1){
      m0.writeMicroseconds(motor_output[0]);
      m1.writeMicroseconds(motor_output[1]);
      m2.writeMicroseconds(motor_output[2]);
      m3.writeMicroseconds(motor_output[3]);
      digitalWrite(led,LOW);
      imu_calibrate();
      s1=1;
    }
  }
  if (stageinflight ==1){ //Flight
    s1=0;   
    control(Cb2i_target,Cb2i,kp,ki,kd,prev_i,prev_error,rc_ctrl,pid_output);
    allocation(rc_ctrl,pid_output,motor_output);
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

  if (stageinflight == -1){ //testing
    motor_output[0] = rc_ctrl[4];
    motor_output[1] = rc_ctrl[4];
    motor_output[2] = rc_ctrl[4];
    motor_output[3] = rc_ctrl[4];
    if ((millis()-prev_time_led)>led_timer_test) digitalWrite(led,HIGH);
    if ((millis()-prev_time_led)>led_timer_test*5){
      digitalWrite(led,LOW);
      prev_time_led = millis();
    }
  }
}
