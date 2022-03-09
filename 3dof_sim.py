from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt



def stateDerivative(state_vector,input_vector,g=9.8055):
    [x,y,x_dot,y_dot,theta,theta_dot] = state_vector
    [F,M] = input_vector # force in the body and moment about cg
    theta_ddot = M / 0.00939 #M/I
    x_ddot = F*np.sin(theta)/0.82 #Acceleration = Force/Mass
    y_ddot = F*np.cos(theta)/0.82 -g
    return np.array([x_dot,y_dot,x_ddot,y_ddot,theta_dot,theta_ddot])

def thrust(pwm):
    max_motor_thrust = 470*2 #grams
    return max_motor_thrust/2000*(pwm-1000)

# 4th Order Runge Kutta Calculation
def RK4(x,u,dt):
    # Inputs: x[k], u[k], dt (time step, seconds)
    # Returns: x[k+1]
    # Calculate slope estimates
    K1 = stateDerivative(x, u)
    # K2 = stateDerivative(x + K1 * dt / 2, u)
    # K3 = stateDerivative(x + K2 * dt / 2, u)
    # K4 = stateDerivative(x + K3 * dt, u)
    # Calculate x[k+1] estimate using combination of slope estimates
    x_next = x + (K1*dt)#1/6 * (K1+ 2*K2 + 2*K3 + K4) * dt
    return x_next

#######
# Magic Numbers
in2m = 0.0254
# Vehicle Properties

arm_length = 5 *in2m
dt=0.002
sim_t = np.arange(0,100,dt)

pwm_input =1010.431383

#Inital State
state_itt = np.zeros((len(sim_t),6))
state = [0,0,0,0,0,0]

for i in range (0,len(sim_t)):
    #do a bunch of random voodo
    #get the hoodo
    #sim results
    m1_force = thrust(pwm_input)
    m2_force = thrust(pwm_input)
    M = m1_force*arm_length - m2_force*arm_length

    if i>0:
        state = state_itt[i-1][:]
    state_itt[i][:] = np.array(RK4(np.array(state),np.array([m1_force+m2_force,M]),dt))



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

