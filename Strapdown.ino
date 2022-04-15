

void dcm2euler(float dcm[3][3]){
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
  float euler[3];
  euler[0] = pitch;
  euler[1] = roll;
  euler[2] = yaw;
  return euler;
}
