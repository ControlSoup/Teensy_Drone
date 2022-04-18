//Allocator code for the Teensy Drone


void allocation(float rc_ctrl[5],float pid_output[3],float motor_ouputs[4]){
  float min_throttle = 1000;
  float throttle = rc_ctrl[4];
  
  if (throttle > 1800) throttle = 1800;
  if (throttle < 1000) throttle = 1000;
  
  motor_ouputs[0] =throttle-pid_output[2]-pid_output[0]-pid_output[1]; //throttle - yaw error - pitch error - roll error 
  motor_ouputs[1] =throttle+pid_output[2]+pid_output[0]+pid_output[1];
  motor_ouputs[2] =throttle-pid_output[2]-pid_output[0]+pid_output[1];
  motor_ouputs[3] =throttle+pid_output[2]+pid_output[0]-pid_output[1];
  
  if (motor_ouputs[0] < min_throttle) motor_ouputs[0] = min_throttle; //Keep the motors running.
  if (motor_ouputs[1] < min_throttle) motor_ouputs[1] = min_throttle;                                         
  if (motor_ouputs[2] < min_throttle) motor_ouputs[2] = min_throttle;                                         
  if (motor_ouputs[3] < min_throttle) motor_ouputs[3] = min_throttle;  
}
