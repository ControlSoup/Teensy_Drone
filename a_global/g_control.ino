void control(float Cb2i_target[3][3],float kp[3], float ki[3], float kd[3],float prev_i[3],float prev_error[3],float rc_ctrl[5],float pid_ouput[3]){

  float Cb2i_error[3][3];

  dcm_error(Cb2i_target,Cb2i,Cb2i_error);
  
  float euler_error[3];
  
  dcm2euler(Cb2i_error,euler_error);
  

  //rc controller commands
  if (rc_ctrl[0] >1800){
    euler_error[0] +=3*deg2rad;
  }
  if (rc_ctrl[0]<1300){
    euler_error[0] -=3*deg2rad;
  }
  if (rc_ctrl[1] >1800){
    euler_error[1] += 3*deg2rad;
  }
  if (rc_ctrl[1]<1300){
    euler_error[1] -= 3*deg2rad;
  }

 
  
  //Attitude Control Law
  float p[3];
  float i[3] = {prev_i[0],prev_i[1],prev_i[2]};
  float d[3];
  
  //pitch
    p[0]= kp[0]*euler_error[0];
    i[0] += ki[0]*euler_error[0];   
    if (euler_error[0] >-3 or euler_error[0] <3){
      i[0] += ki[0]*euler_error[0];
    }
    else i[0] =0; 
    d[0] = kd[0]*(prev_error[0] - euler_error[0]);
    prev_error[0] = euler_error[0];
    pid_output[0] = p[0] + i[0] +d[0];
    prev_i[0] = i[0];
  
  //roll
    p[1]= kp[1]*euler_error[1];
    i[1] += ki[1]*euler_error[1];
    if (euler_error[1] >-3 or euler_error[1] <3){
      i[1] += ki[1]*euler_error[1];
    }
    else i[1] =0;
    d[1] = kd[1]*(prev_error[1] - euler_error[1]);
    prev_error[1] = euler_error[1];
    pid_output[1] = p[1] + i[1] +d[1];
    prev_i[1] = i[1];

 //roll
    p[2]= kp[2]*euler_error[2];
    i[2] += ki[2]*euler_error[2];
    if (euler_error[2] >-3 or euler_error[2] <3){
      i[2] += ki[2]*euler_error[2];
    }
    else i[2] =0;
    pid_output[2] = p[2] + i[2] +d[2];
    prev_i[2] = i[2];



}
