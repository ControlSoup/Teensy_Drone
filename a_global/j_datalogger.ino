/////////////////
//Data Logging//
///////////////
void data_logger_setup(){
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");
  digitalWrite(led,HIGH);
  delay(500);
  digitalWrite(led,LOW);
  delay(500);
  digitalWrite(led,HIGH);
  delay(500);
  digitalWrite(led,LOW);
  delay(500);
  digitalWrite(led,HIGH);
  delay(500);
  digitalWrite(led,LOW);
  delay(500);
}
void write_sd(){
  // make a string for assembling the data to log:
  String dataString = "";
 
  if (s3 !=1){
    dataString = "test_number,stageinflight,time(s),ax(ms2),ay(ms2),az(ms2),gx(deg/s),gy(deg/s),gz(deg/s),pitch_error(deg),roll_error(deg),yaw_error(deg),pitch_output,p_pitch_output,i_pitch_output,d_pitch_output,roll_output,p_roll_output,i_roll_output,d_roll_output,yaw_output,m0_output(pwm),m1_output(pwm),m2_output(pwm),m3_output(pwm)";
    File dataFile = SD.open("flight_test.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    } else {
      // if the file isn't open, pop up an error:
    }
    s3=1;
  }
  else{
    dataString += String(test_number)+", "+String(stageinflight)+", " + String(data_time) + ", " + String(acc[0])+", "+String(acc[1])+", "+String(acc[2])+", "+String(gyro[0]*rad2deg)+", "+String(gyro[1]*rad2deg)+", "+String(gyro[2]*rad2deg)+", "+String(euler_error[0]*rad2deg)+", "+String(euler_error[1]*rad2deg)+", "+String(euler_error[2]*rad2deg)+", "+String(pid_output[0])+", "+String(p[0])+","+String(i[0])+", "+String(d[0])+", "+String(pid_output[1])+", "+String(p[1])+","+String(i[1])+", "+String(d[1])+", "+String(pid_output[2])+", "+String(motor_output[0])+", "+String(motor_output[1])+", "+String(motor_output[2])+", "+String(motor_output[3]);
    File dataFile = SD.open("flight_test.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    } else {
      // if the file isn't open, pop up an error:
    }
  }
  
  
  
}
