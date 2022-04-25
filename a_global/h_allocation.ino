//////////////
//Allocation//
/////////////

void allocation(float rc_ctrl[5],float pid_output[3],float motor_ouput[4]){
  float min_throttle = 1000;
  float throttle = rc_ctrl[3];
  
  if (throttle > 1800) throttle = 1800;
  if (throttle < 1000) throttle = 1000;
  
  motor_ouput[0] =throttle+pid_output[0]+pid_output[1]+pid_output[2]; //throttle - yaw error - pitch error - roll error 
  motor_ouput[1] =throttle+pid_output[0]-pid_output[1]-pid_output[2];
  motor_ouput[2] =throttle-pid_output[0]+pid_output[1]-pid_output[2];
  motor_ouput[3] =throttle-pid_output[0]-pid_output[1]+pid_output[2];
  
  if (motor_ouput[0] < min_throttle) motor_ouput[0] = min_throttle; //Keep the motors running.
  if (motor_ouput[1] < min_throttle) motor_ouput[1] = min_throttle;                                         
  if (motor_ouput[2] < min_throttle) motor_ouput[2] = min_throttle;                                         
  if (motor_ouput[3] < min_throttle) motor_ouput[3] = min_throttle;  

  Serial.print(motor_ouput[0]);
  Serial.print(" ");
  Serial.println(motor_ouput[1]);
}
