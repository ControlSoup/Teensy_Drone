//////////////
//Navigation//
//////////////

void nav(){
  gyro2dcm(gyro,Cb2i_gyro,dt);
  //sketchy comp filter  
  imu_init(acc,Cb2i_acc);
  complimentary_filter(0.996,Cb2i_gyro,Cb2i_acc,Cb2i);
}
//stage of flight commands (should be final part of code)
