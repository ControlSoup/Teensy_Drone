// The following is a a set of functions based on Strapdown Analytics by Paul G. Savage

//Notes:
// All [3] arrays should really be 3x1 not 1x3 however i have chosen to use 3x1


//Useful Magic Numbers:
float rad2deg = 57.2958;
float deg2rad = 0.01745328;
float standard_gravity = 9.80665; 
float g2ms2 = standard_gravity;


//the result matrix is replaced with the cross product of a and b
void cross3x3(float a[3][3],float b[3][3], float result[3][3]){
  result[0][0] = (a[0][0] * b[0][0]) + (a[0][1] * b[1][0]) + (a[0][2] * b[2][0]);
  result[1][0] = (a[1][0] * b[0][0]) + (a[1][1] * b[1][0]) + (a[1][2] * b[2][0]);
  result[2][0] = (a[2][0] * b[0][0]) + (a[2][1] * b[1][0]) + (a[2][2] * b[2][0]);
  
  result[0][1] = (a[0][0] * b[0][1]) + (a[0][1] * b[1][1]) + (a[0][2] * b[2][1]);
  result[1][1] = (a[1][0] * b[0][1]) + (a[1][1] * b[1][1]) + (a[1][2] * b[2][1]);
  result[2][1] = (a[2][0] * b[0][1]) + (a[2][1] * b[1][1]) + (a[2][2] * b[2][1]);
  
  result[0][2] = (a[0][0] * b[0][2]) + (a[0][1] * b[1][2]) + (a[0][2] * b[2][2]);
  result[1][2] = (a[1][0] * b[0][2]) + (a[1][1] * b[1][2]) + (a[1][2] * b[2][2]);
  result[2][2] = (a[2][0] * b[0][2]) + (a[2][1] * b[1][2]) + (a[2][2] * b[2][2]);
}


//the result matrix is replaced witht the transpose of a
void transpose3x3(float a[3][3],float result[3][3]){
  result[0][0] = a[0][0];
  result[1][0] = a[0][1];
  result[2][0] = a[0][2];
  
  result[0][1] = a[1][0];
  result[1][1] = a[1][1];
  result[2][1] = a[1][2];
  
  result[0][2] = a[2][0];
  result[1][2] = a[2][1];
  result[2][2] = a[2][2];
}




//the result matrix is replaced with euler angle equivalent of  a dcm (resulting angles in radians)
void dcm2euler(float dcm[3][3],float result[3]){
  float roll,pitch,yaw;
  roll = atan(-dcm[2][0]/sqrt(1-(dcm[2][0]*dcm[2][0])));
  if (abs(dcm[2][0]) <0.999){
    pitch = atan(dcm[2][1]/dcm[2][2]);
    yaw = atan(dcm[1][0]/dcm[0][0]);
  }
  if (dcm[2][0] <= - 0.999){
    pitch = yaw - atan((dcm[1][2]-dcm[0][1])/(dcm[0][2]+dcm[1][1]));
  }
  if (dcm[2][0] >= 0.999){
    pitch =  3.1415926 + atan((dcm[1][2]+dcm[0][1])/(dcm[0][2]-dcm[1][1]))-yaw ;
  }

  result[0] = pitch;
  result[1] = roll;
  result[2] = yaw;
}

//takes a vector of gyro measurments (radians/s) and updates a dcm (Cb2i_gyro) with the current gyro integration
void gyro2dcm(float gyro[3],float Cb2i_gyro[3][3], float dt){
    //scew symetric pg 3-52
    float scew_sym[3][3];
    scew_sym[0][0] = 0;
    scew_sym[0][1] = -gyro[2];
    scew_sym[0][2] = gyro[1];
    
    scew_sym[1][0] = gyro[2];
    scew_sym[1][1] = 0;
    scew_sym[1][2] = -gyro[0];
    
    scew_sym[2][0] = -gyro[1];
    scew_sym[2][1] = -gyro[0];
    scew_sym[2][2] = 0;
    
    //DCM_rate from body to inertial frame pg 3-53 Cb2i_dot = Cb2i X scew symetric
    float Cb2i_dot[3][3];
    cross3x3(Cb2i_gyro,scew_sym,Cb2i_dot);

    //gyro dcm estimate integration
    Cb2i_gyro[0][0] += Cb2i_dot[0][0] *dt; 
    Cb2i_gyro[1][0] += Cb2i_dot[1][0] *dt; 
    Cb2i_gyro[2][0] += Cb2i_dot[2][0] *dt; 
    
    Cb2i_gyro[0][1] += Cb2i_dot[0][1] *dt; 
    Cb2i_gyro[1][1] += Cb2i_dot[1][1] *dt; 
    Cb2i_gyro[2][1] += Cb2i_dot[2][1] *dt; 
    
    Cb2i_gyro[0][2] += Cb2i_dot[0][2] *dt; 
    Cb2i_gyro[1][2] += Cb2i_dot[1][2] *dt; 
    Cb2i_gyro[2][2] += Cb2i_dot[2][2] *dt; 
}




void imu_init(float init_acc[4],float Cb2i_gyro[3][3]){
  //Initalize a Gyro (disregarding rotation of the earth) pg6-4 

  
  //init_acc = [ax_average,ay_average,az_average,acc_average_magnitude]
  Cb2i_gyro[2][0] = init_acc[0]/init_acc[3];
  Cb2i_gyro[2][1] = init_acc[1]/init_acc[3];
  Cb2i_gyro[2][2] = init_acc[2]/init_acc[3];
  
  Cb2i_gyro[1][0] = 0;
  Cb2i_gyro[1][1] = Cb2i_gyro[2][2]/sqrt((Cb2i_gyro[2][1]*Cb2i_gyro[2][1])+(Cb2i_gyro[2][2]*Cb2i_gyro[2][2]));
  Cb2i_gyro[1][2] = -Cb2i_gyro[2][1] /sqrt((Cb2i_gyro[2][1]*Cb2i_gyro[2][1])+(Cb2i_gyro[2][2]*Cb2i_gyro[2][2]));
  
  Cb2i_gyro[0][0] = (Cb2i_gyro[1][1]*Cb2i_gyro[2][2])-(Cb2i_gyro[1][2]*Cb2i_gyro[2][1]);
  Cb2i_gyro[0][1] = (Cb2i_gyro[1][2]*Cb2i_gyro[2][0])-(Cb2i_gyro[1][0]*Cb2i_gyro[2][2]);
  Cb2i_gyro[0][2] = (Cb2i_gyro[1][0]*Cb2i_gyro[2][1])-(Cb2i_gyro[1][1]*Cb2i_gyro[2][0]);
}


//Calculates the DCM error based on a target dcm (Cb2i_target) and current dcm estimate (Cb2i), replacing an error matrix (Cb2i_error)
void dcm_error(float Cb2i_target[3][3],float Cb2i[3][3],float Cb2i_error[3][3]){
  //DCM error  Cb2i_error = Cb2i_target x Cb2i^T
  float Cb2i_T[3][3];
  transpose3x3(Cb2i,Cb2i_T);
  cross3x3(Cb2i_target,Cb2i_T,Cb2i_error);
}


//NOT TESTED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


//the result matrix is replaced with the cross product of by3 andby1
void cross3x3x3x1 (float by3[3][3], float by1[3] , float result[3]){
  result[0] = (by3[0][0] * by1[0]) + (by3[0][1]*by1[1]) + (by3[0][2]*by1[2]);
  result[1] = (by3[1][0] * by1[0]) + (by3[1][1]*by1[1]) + (by3[1][2]*by1[2]);
  result[2] = (by3[2][0] * by1[0]) + (by3[2][1]*by1[1]) + (by3[2][2]*by1[2]);
  
}

void acc2intertial(float acc[4],float Cb2i[3][3],float acc_i[4]){
  //pg 4-10 Eq (4.2-1) 
  cross3x3x3x1(Cb2i,acc,acc_i);
}
