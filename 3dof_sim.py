# Change Log
#Added pid loop for altitude needs tuning and variable gains

from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt



def stateDerivative(state_vector,input_vector,g=9.8055):
    [x,y,x_dot,y_dot,theta,theta_dot] = state_vector
    [F,M] = input_vector #Force in the body and moment about cg
    theta_ddot = M / 0.00939 #M/I
    x_ddot = F*np.sin(theta)/0.82 #Acceleration = Force/Mass
    y_ddot = F*np.cos(theta)/0.82-g
    return np.array([x_dot,y_dot,x_ddot,y_ddot,theta_dot,theta_ddot])

def thrust(pwm):
    pwm_percent = (pwm-1000)/1000
    max_motor_thrust = (470*2)/1000*kg2n #kilo/grams
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
dt=0.002

sim_t = np.arange(0,100,dt)
pwm_input = 1000

#Inital State
state_itt = np.zeros((len(sim_t),7))
state = [0,0,0,0,0,0]

#Control
kp_alt = 1
ki_alt = 0.001
kd_alt = 0
alt_i = 0
prev_i = 0
prev_alt_error =0
alt_output = 1000
control_timer =0
control_dt = 1/250
#Data Log
thrust_log = np.zeros(len(sim_t))
alt_output_log = []



for i in range (0,len(sim_t)):
    #do a bunch of random voodo
    #get the hoodo
    #sim results

    if i-control_timer >= control_dt:
        #alltitude pid
        alt_error = 0-state[1]
        alt_p = kp_alt*alt_error
        alt_i = prev_i + ki_alt*alt_error*dt
        prev_i = alt_i
        alt_d = kd_alt*(prev_alt_error - alt_error)/dt
        prev_alt_error = alt_error
        alt_output = alt_p+alt_i+alt_d+1436.31415926

        alt_output_log.append(alt_output)
        control_timer = i




    m1_force = thrust(alt_output)
    m2_force = thrust(alt_output)
    M = m1_force*arm_length - m2_force*arm_length

    if i>0:
        state = state_itt[i-1][:]

    state_itt[i][0:6] = RK4(np.array(state[0:6]),np.array([m1_force+m2_force,M]),dt)
    state_itt[i][6] = alt_output




plt.plot(state_itt[:,1], state_itt[:,6], 'r--', label="pid_output")
plt.legend()
plt.xlabel('Y Position (m)')
plt.ylabel('Pid Output')
plt.show()

plt.plot(sim_t, state_itt[:,0], 'r--', label="x_pos")
plt.plot(sim_t, state_itt[:,1], 'k--', label="y_pos")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Pos(m) ')
plt.show()


plt.plot(sim_t, state_itt[:,2], 'r--', label="x_dot")
plt.plot(sim_t, state_itt[:,3], 'k--', label="y_dot")
plt.legend()
plt.xlabel('Time(s)')
plt.ylabel('Velocity(m/s)')
plt.show()
