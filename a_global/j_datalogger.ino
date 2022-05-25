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
      dataFile.println("Start");
      dataFile.println("["+String(kp[0])+","+String(kp[1])+","+String(kp[2])+"]"+"["+String(ki[0])+","+String(ki[1])+","+String(ki[2])+"]"+"["+String(kd[0])+","+String(kd[1])+","+String(kd[2])+"]");
      dataFile.println(dataString);
      dataFile.close();
    } else {
      // if the file isn't open, pop up an error:
    }
    s3=1;
  }
  else{
    dataString += String(test_number)+", "+String(stageinflight)+", " + String(data_time,3) + ", " + String(acc[0],6)+", "+String(acc[1],6)+", "+String(acc[2],6)+", "+String(gyro[0]*rad2deg,6)+", "+String(gyro[1]*rad2deg,6)+", "+String(gyro[2]*rad2deg,6)+", "+String(euler_error[0]*rad2deg,6)+", "+String(euler_error[1]*rad2deg,6)+", "+String(euler_error[2]*rad2deg,6)+", "+String(pid_output[0],6)+", "+String(p[0],6)+","+String(i[0],6)+", "+String(d[0],6)+", "+String(pid_output[1],6)+", "+String(p[1],6)+","+String(i[1],6)+", "+String(d[1],6)+", "+String(pid_output[2],6)+", "+String(motor_output[0],6)+", "+String(motor_output[1],6)+", "+String(motor_output[2],6)+", "+String(motor_output[3],6);
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
