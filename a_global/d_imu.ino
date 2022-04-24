///////
//IMU//
///////
void imu_update(){
  //Read Raw data 
  mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);  //magical i2c library shit that i dont understand
  acc[0] = float(ax)/4096*g2ms2; //Convert int16 raw (LSB) to float m/s2
  acc[1] = float(ay)/4096*g2ms2;
  acc[2] = float(az)/4096*g2ms2;
  acc[3] = sqrt((acc[0]*acc[0])+(acc[1]*acc[1])+(acc[2]*acc[2]));
  gyro[0] = (float(gx)-gyro_cal[0])/65.5*deg2rad; //Convert int16 raw (LSB) to float rad/s
  gyro[1] = (float(gy)-gyro_cal[1])/65.5*deg2rad;
  gyro[2] = (float(gz)-gyro_cal[2])/65.5*deg2rad;    
}

void imu_calibrate(){
  float init_acc[4];
  int n = 2000;
  for (int cal_int = 0; cal_int < n ; cal_int ++){                  //Run this code n times 
    mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);                           
    gyro_cal[0] += gx;
    gyro_cal[1] += gy;
    gyro_cal[2] += gz;        
    init_acc[0] += ax;
    init_acc[1] += ay;
    init_acc[2] += az;
                                          
    delay(4);                                                         //(250hz)
  }
  gyro_cal[0] /= n;                                                  //Divide the gyro_x_cal variable by n to get the avarage offset
  gyro_cal[1] /= n;                                                  
  gyro_cal[2] /= n;
  init_acc[0] /= n;
  init_acc[1] /= n;
  init_acc[2] /= n;
  init_acc[3] = sqrt((init_acc[0]*init_acc[0])+(init_acc[1]*init_acc[1])+(init_acc[2]*init_acc[2]));
  
  imu_init(init_acc,Cb2i_gyro);
  
  
}
void imu_startup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(1);
}
