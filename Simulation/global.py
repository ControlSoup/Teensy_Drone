#######
# Magic Numbers
rad2deg = 57.2958
deg2rad = 0.01745328
standard_gravity = 9.80665
g2ms2 = standard_gravity
pi = 3.1415926
analog2voltage = (3.3 / 1023.0)
in2m = 0.0254

# Vehicle Properties
arm_length = 5 *in2m

# Integration options
dt=1/500
tf = 120
sim_t = np.arange(0,tf,dt)

#Inital State
state_itt = np.zeros((len(sim_t),13))
state = [0,0,0,0,0,0]

#Navigiation Suggestions
alt_desired = 10
pitch_desired = 0

#altitude, theta
kp = [0,0]
ki = [0,0]
kd = [0,0]
prev_i = [0,0]


#Timers
control_timer =0
control_dt = 1/250

#Motors

motor_output=[0,0]

# Actuators
pwm_input = 0

#Noise
alt_noise_pre = np.random.normal(0,0.001,len(sim_t))
pitch_noise_pre =  np.random.normal(0,0.00,len(sim_t))