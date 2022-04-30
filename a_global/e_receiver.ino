/////////////
//Receiver///
/////////////

void receiver_update(){
  rc_ctrl[0] = ch1.getValue(); //pitch
  if (rc_ctrl[0] < 1000) rc_ctrl[0] = 1000;
  if (rc_ctrl[0] > 2000) rc_ctrl[0] = 2000;
  rc_ctrl[1] = ch2.getValue(); //roll
  if (rc_ctrl[1] < 1000) rc_ctrl[1] = 1000;
  if (rc_ctrl[1] > 2000) rc_ctrl[1] = 2000;
  rc_ctrl[2] = ch4.getValue(); //yaw
  if (rc_ctrl[2] < 1000) rc_ctrl[2] = 1000;
  if (rc_ctrl[2] > 2000) rc_ctrl[2] = 2000;
  rc_ctrl[3] = ch3.getValue(); //throttle
  if (rc_ctrl[3] < 1000) rc_ctrl[3] = 1000;
  if (rc_ctrl[3] > 2000) rc_ctrl[3] = 2000;
  rc_ctrl[4] = ch5.getValue(); //switch
  if (rc_ctrl[4] < 1000) rc_ctrl[4] = 1000;
  if (rc_ctrl[4] > 2000) rc_ctrl[4] = 2000;
 
}
