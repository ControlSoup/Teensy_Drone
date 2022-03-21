# Change Log
#Added Pitch Controller, implemented noise to the control loop and tuned some of the pid loops
#Added a imu model start
#Any amount of noise or setpoint error that isint perfect fucks everthing up

from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt



#Rough cut of an imu model
def imu_model(state_vector,integration_values,dt):
    [x, y, x_dot, y_dot, theta, theta_dot] = state_vector
    [prev_x_dot,prev_y_dot,total_drift] = integration_values
    #Barometer + Sonar + GPS eventually?
    baro_noise = 0
    baro_read = y + baro_noise

    #Accelerometer
    acc_read = [(prev_x_dot-x_dot)/dt,(prev_y_dot-y_dot)/dt]

    #Gyro
    drift_slope = 0.001
    total_drift = total_drift + drift_slope
    gyro_read = theta_dot + gyro_noise + (total_drift)

    return [[baro_read,acc_read,gyro_read],[prev_x_dot,prev_y_dot,total_drift]]


def dragModel(x, Cd=0.1):
    # Input: state vector x
    # Returns: drag force given current velocity vector and attitude
    #
    # Probably want to add the previous thrust command in here as well
    # Honestly not sure what the prop dynamics look like.
    # Drag is low when cross section is low, drag is high when cross section is high??

    x_dot = x[2]
    y_dot = x[3]
    theta = x[4]
    airspeed = np.sqrt(x_dot ** 2 + y_dot ** 2)
    airspeed_angle = np.arctan2(y_dot, x_dot)
    return -1 * Cd * np.abs(np.sin(theta - airspeed_angle)) * airspeed * np.array([x_dot, y_dot])  #newtons?

def stateDerivative(state_vector,input_vector,g=9.8055):
    [x,y,x_dot,y_dot,theta,theta_dot] = state_vector
    [F,M] = input_vector #Force in the body and moment about cg
    theta_ddot = M / 0.00939 #M/I
    [Fdx, Fdy] = dragModel(state_vector)
    x_ddot = F*np.sin(theta)/0.82 + Fdx #Acceleration = Force/Mass
    y_ddot = F*np.cos(theta)/0.82 + Fdy - g
    return np.array([x_dot,y_dot,x_ddot,y_ddot,theta_dot,theta_ddot])

def thrust(pwm):
    pwm_percent = (pwm-1000)/1000
    max_motor_thrust = (600*2)/1000*kg2n #kilo/grams
    return pwm_percent*max_motor_thrust

# 4th Order Runge Kutta Calculation
def RK4(x,u,dt):
    # Inputs: x[k], u[k], dt (time step, seconds)
    # Returns: x[k+1]
    # Calculate slope estimates
    K1 = stateDerivative(x, u)
    K2 = stateDerivative(x + K1 * dt / 2, u)
    K3 = stateDerivative(x + K2 * dt / 2, u)
    K4 = stateDerivative(x + K3 * dt, u)
    # Calculate x[k+1] estimate using combination of slope estimates
    x_next = x + (1/6 * (K1+ 2*K2 + 2*K3 + K4) * dt)
    return np.array(x_next)



#######
# Magic Numbers
in2m = 0.0254
kg2n= 9.8055

# Vehicle Properties
arm_length = 5 *in2m

# Integration options
dt=1/500
tf = 120
sim_t = np.arange(0,tf,dt)

#Inital State
state_itt = np.zeros((len(sim_t),13))
state = [0,0,0,0,0,0]

#Control
alt_desired = 10
kp_alt = 20
ki_alt = 0.1
kd_alt = 2
alt_i = 0
prev_alt_i = 0
prev_alt_error =0

pitch_desired = 0
kp_pitch = 0
ki_pitch = 0
kd_pitch = 0
pitch_i = 0
prev_pitch_i = 0
prev_pitch_error =0



control_timer =0
control_dt = 1/250
m1_output = 1000
m2_output = 1000
pitch_output=0
alt_output =0
alt_p =0
pitch_p=0
pitch_i =0
alt_i =0
alt_d=0
pitch_d=0
# Actuators
pwm_input = 0

#Noise
alt_noise_pre = np.random.normal(0,0.001,len(sim_t))
pitch_noise_pre =  np.random.normal(0,0.00,len(sim_t))



for i in range (0,len(sim_t)):
    #to the center of the eye we go

    if sim_t[i]-control_timer >= control_dt:
        control_timer = sim_t[i]
        #alltitude pid
        alt_noise = alt_noise_pre[i]



        alt_error = alt_desired -(state[1] + alt_noise)
        alt_p = kp_alt*alt_error
        alt_i = prev_alt_i + (ki_alt*alt_error*control_dt)
        prev_alt_i = alt_i
        alt_d = kd_alt*(prev_alt_error - alt_error)/control_dt
        prev_alt_error = alt_error
        alt_output = alt_p+alt_i+alt_d+1436.31415926
        if alt_output > 1800:
            alt_output = 1800
        if alt_output < 1000:
            alt_output = 1000

        #pitch pid
        pitch_noise = pitch_noise_pre[i]

        pitch_error = pitch_desired - (state[4] + pitch_noise)
        pitch_p = kp_pitch * pitch_error
        pitch_i = prev_pitch_i + (ki_pitch * pitch_error * control_dt)
        prev_pitch_i = pitch_i
        pitch_d = kd_pitch * (prev_pitch_error - pitch_error) / control_dt
        prev_pitch_error = pitch_error
        pitch_output = pitch_p + pitch_i + pitch_d


        if sim_t[i] > 5:
            pitch_desired = 3
        if sim_t[i] > 10:
            pitch_desired = -3
        if sim_t[i] > 15:
            pitch_desired = 0

        m1_output = alt_output-pitch_output
        if m1_output > 2000:
            m1_output = 2000
        if m1_output < 1000:
            m1_output = 1000
        m2_output = alt_output + pitch_output
        if m2_output > 2000:
            m2_output = 2000
        if m2_output < 1000:
            m2_output = 1000






    m1_force = thrust(m1_output)
    m2_force = thrust(m2_output)

    M = m1_force*arm_length - m2_force*arm_length

    if i>0:
        state = state_itt[i-1][:]
    else:
        alt_error =0
        pitch_error=0



    state_itt[i][0:6] = RK4(np.array(state[0:6]),np.array([m1_force+m2_force,M]),dt)
    state_itt[i][6] = m1_force
    state_itt[i][7] = m2_force
    state_itt[i][8] = pitch_p
    state_itt[i][9] = pitch_i
    state_itt[i][10] = pitch_d
    state_itt[i][11] = pitch_error
    state_itt[i][12] = pitch_desired


plt.subplots(5, 1)

plt.subplot(5, 1, 1)
plt.plot(sim_t, state_itt[:,11], 'r--', label="Error")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Pitch Error')

plt.subplot(5, 1, 2)
plt.plot(sim_t, state_itt[:,8], 'r--', label="pitch_p")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('P Term')

plt.subplot(5, 1, 3)
plt.plot(sim_t, state_itt[:,9], 'r--', label="pitch_i")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('I Term')

plt.subplot(5, 1, 4)
plt.plot(sim_t, state_itt[:,10], 'r--', label="pitch_d")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('D Term')

plt.subplot(5, 1, 5)
plt.plot(sim_t, state_itt[:,12], 'r--', label="pitch_d")
plt.xlabel('Time(s)')
plt.ylabel('Pitch Desired')

plt.show()

plt.subplots(4, 1)

plt.subplot(4, 1, 1)
plt.plot(sim_t, state_itt[:,6], 'r--', label="m1_force")
plt.plot(sim_t, state_itt[:,7], 'k--', label="m2_force")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Thrust(N)')

plt.subplot(4, 1, 2)
plt.plot(sim_t, state_itt[:,0], 'r--', label="x_pos")
plt.plot(sim_t, state_itt[:,1], 'k--', label="y_pos")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Pos(m)')

plt.subplot(4, 1, 3)
plt.plot(sim_t, state_itt[:,2], 'r--', label="x_dot")
plt.plot(sim_t, state_itt[:,3], 'k--', label="y_dot")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Velocity(m/s)')

plt.subplot(4, 1, 4)
plt.plot(sim_t, state_itt[:,4], 'r--', label="theta")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Pitch(degrees)')

plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.4)

plt.show()
