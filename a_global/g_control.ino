///////////////
//Control Law//
///////////////

void control(float Cb2i_target[3][3],float Cb2i[3][3],float kp[3], float ki[3], float kd[3],float prev_i[3],float prev_error[3],float rc_ctrl[5],float pid_output[3],float euler_error[3]){
  int deg_limit = 6;
  
  float Cb2i_error[3][3];
  float imu2frame[3] = {1.854753,6.88589}; //Set the drone on a level table and measure the pitch and roll error (this is the tilt of the imu relative to the drone frame)
  
  dcm_error(Cb2i_target,Cb2i,Cb2i_error);
  dcm2euler(Cb2i_error,euler_error);
 
  //rc controller commands
  euler_error[0] +=((rc_ctrl[0]-1500)*0.0002-(imu2frame[0]*deg2rad));
  euler_error[1]-= (rc_ctrl[1]-1500)*0.002+(imu2frame[1]*deg2rad);
  //Attitude Control Law

  i[0] = prev_i[0];
  i[1] = prev_i[1];
  i[2] = prev_i[2];
  
  //pitch
    p[0]= kp[0]*euler_error[0];
    if (euler_error[0] >-(deg_limit*deg2rad) and euler_error[0] <(deg_limit*deg2rad)){
      i[0] += ki[0]*euler_error[0];
    }
    else i[0] =0; 
    d[0] = kd[0]*-gyro[0];
    prev_error[0] = euler_error[0];
    pid_output[0] = p[0] + i[0] +d[0];
    prev_i[0] = i[0];
    if (pid_output[0] >=100)pid_output[0] = 100;
    if (pid_output[0] <=-100)pid_output[0] = -100;
  //roll
    p[1]= kp[1]*euler_error[1];
    if (euler_error[1] >-(deg_limit*deg2rad) and euler_error[1] <(deg_limit*deg2rad)){
      i[1] += ki[1]*euler_error[1];
    }
    else i[1] =0;
    d[1] = kd[1]*-gyro[1];
    prev_error[1] = euler_error[1];
    pid_output[1] = p[1] + i[1] +d[1];
    prev_i[1] = i[1];
    if (pid_output[1] >=100)pid_output[1] = 100;
    if (pid_output[1] <=-100)pid_output[1] = -100;

 //roll
    p[2]= kp[2]*euler_error[2];
    if (euler_error[2] >-(deg_limit*deg2rad) and euler_error[2] <(deg_limit*deg2rad)){
      i[2] += ki[2]*euler_error[2];
    }
    else i[2] =0;
    pid_output[2] = p[2] + i[2] +d[2];
    prev_i[2] = i[2];
    if (pid_output[2] >=450)pid_output[2] = 450;
    if (pid_output[2] <=-450)pid_output[2] = -450;


}
