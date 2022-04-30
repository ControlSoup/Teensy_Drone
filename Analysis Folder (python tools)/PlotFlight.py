from pandas import read_csv
import matplotlib.pyplot as plt


        #Open Data
data = read_csv('test.txt',sep=',',skiprows=0)
time = data['time(s)']
ax = data['ax(ms2)']
ay = data['ay(ms2)']
az = data['az(ms2)']
gx = data['gx(deg/s)']
gy = data['gy(deg/s)']
gz = data['gz(deg/s)']
pitch_error = data['pitch_error(deg)']
roll_error = data['roll_error(deg)']
yaw_error = data['yaw_error(deg)']
pitch_output = data['pitch_output']
p_pitch = data['p_pitch_output']
i_pitch = data['i_pitch_output']
d_pitch = data['d_pitch_output']
roll_output = data['roll_output']
p_roll = data['p_roll_output']
i_roll = data['i_roll_output']
d_roll = data['d_roll_output']
yaw_output = data['yaw_output']
m0_output = data['m0_output(pwm)']
m1_output = data['m1_output(pwm)']
m2_output = data['m2_output(pwm)']
m3_output = data['m3_output(pwm)']


plt.title("Pitch,Roll,Yaw Error (deg)")
plt.plot(time,pitch_error, label='pitch')
plt.plot(time,roll_error, label='roll')
plt.plot(time,yaw_error, label='yaw')
plt.legend()
plt.xlabel('time(s)')
plt.ylabel('Error (deg)')
plt.show()

plt.title("Pitch,Roll,Yaw Output")
plt.plot(time,pitch_output, label='pitch')
plt.plot(time,roll_output, label='roll')
plt.plot(time,yaw_output, label='yaw')
plt.legend()
plt.xlabel('time(s)')
plt.ylabel('Output(deg)')
plt.show()

plt.title("Pitch PID")
plt.plot(time,p_pitch, label='p')
plt.plot(time,i_pitch, label='i')
plt.plot(time,d_pitch, label='d')
plt.legend()
plt.xlabel('time(s)')
plt.ylabel('Output(')
plt.show()

plt.title("Roll PID")
plt.plot(time,p_roll, label='p')
plt.plot(time,i_roll, label='i')
plt.plot(time,d_roll, label='d')
plt.legend()
plt.xlabel('time(s)')
plt.ylabel('Output')
plt.show()

plt.title("Motor Outputs")
plt.plot(time,m0_output, label='m0')
plt.plot(time,m1_output, label='m1')
plt.plot(time,m2_output, label='m2')
plt.plot(time,m3_output, label='m3')
plt.legend()
plt.xlabel('time(s)')
plt.ylabel('Output(pwm)')
plt.show()
