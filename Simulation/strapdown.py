import numpy as np
null_3x3 = [[0,0,0],[0,0,0],[0,0,0]]
null_3x1 = [0,0,0]
#the result matrix is replaced with the cross product of a and b
def cross3x3(a,b,result):
    result[0][0] = (a[0][0] * b[0][0]) + (a[0][1] * b[1][0]) + (a[0][2] * b[2][0])
    result[1][0] = (a[1][0] * b[0][0]) + (a[1][1] * b[1][0]) + (a[1][2] * b[2][0])
    result[2][0] = (a[2][0] * b[0][0]) + (a[2][1] * b[1][0]) + (a[2][2] * b[2][0])

    result[0][1] = (a[0][0] * b[0][1]) + (a[0][1] * b[1][1]) + (a[0][2] * b[2][1])
    result[1][1] = (a[1][0] * b[0][1]) + (a[1][1] * b[1][1]) + (a[1][2] * b[2][1])
    result[2][1] = (a[2][0] * b[0][1]) + (a[2][1] * b[1][1]) + (a[2][2] * b[2][1])

    result[0][2] = (a[0][0] * b[0][2]) + (a[0][1] * b[1][2]) + (a[0][2] * b[2][2])
    result[1][2] = (a[1][0] * b[0][2]) + (a[1][1] * b[1][2]) + (a[1][2] * b[2][2])
    result[2][2] = (a[2][0] * b[0][2]) + (a[2][1] * b[1][2]) + (a[2][2] * b[2][2])

#the result matrix is replaced witht the transpose of a
def transpose3x3(a,result):
    result[0][0] = a[0][0]
    result[1][0] = a[0][1]
    result[2][0] = a[0][2]

    result[0][1] = a[1][0]
    result[1][1] = a[1][1]
    result[2][1] = a[1][2]

    result[0][2] = a[2][0]
    result[1][2] = a[2][1]
    result[2][2] = a[2][2]

#the result matrix is replaced with euler angle equivalent of  a dcm (resulting angles in radians)
def dcm2euler(dcm,result):
    roll = np.atan(-dcm[2][0] / np.sqrt(1 - (dcm[2][0] * dcm[2][0])))
    if (abs(dcm[2][0]) < 0.999):
        pitch = np.atan(dcm[2][1] / dcm[2][2])
        yaw = np.atan(dcm[1][0] / dcm[0][0])

    if (dcm[2][0] <= - 0.999):
        pitch = yaw - np.atan((dcm[1][2]-dcm[0][1]) / (dcm[0][2]+dcm[1][1]))

    if (dcm[2][0] >= 0.999):
        pitch =  3.1415926 + np.atan((dcm[1][2]+dcm[0][1]) / (dcm[0][2]-dcm[1][1]))-yaw
    result[0] = pitch
    result[1] = roll
    result[2] = yaw

#takes a vector of gyro measurments (radians/s) and updates a dcm (Cb2i_gyro) with the current gyro integration
def gyro2dcm(gyro,Cb2i_gyro,dt):
    scew_sym = [[0,0,0],[0,0,0],[0,0,0]]
    scew_sym[3][3]
    scew_sym[0][0] = 0
    scew_sym[0][1] = -gyro[2]
    scew_sym[0][2] = gyro[1]

    scew_sym[1][0] = gyro[2]
    scew_sym[1][1] = 0
    scew_sym[1][2] = -gyro[0]

    scew_sym[2][0] = -gyro[1]
    scew_sym[2][1] = -gyro[0]
    scew_sym[2][2] = 0

    #DCM_rate from body to inertial frame pg 3 - 53 Cb2i_dot = Cb2i X scew symetric

    Cb2i_dot= [[0,0,0],[0,0,0],[0,0,0]]
    cross3x3(Cb2i_gyro, scew_sym, Cb2i_dot)

    #gyro dcm estimate integration
    Cb2i_gyro[0][0] += Cb2i_dot[0][0] * dt
    Cb2i_gyro[1][0] += Cb2i_dot[1][0] * dt
    Cb2i_gyro[2][0] += Cb2i_dot[2][0] * dt

    Cb2i_gyro[0][1] += Cb2i_dot[0][1] * dt
    Cb2i_gyro[1][1] += Cb2i_dot[1][1] * dt
    Cb2i_gyro[2][1] += Cb2i_dot[2][1] * dt

    Cb2i_gyro[0][2] += Cb2i_dot[0][2] * dt
    Cb2i_gyro[1][2] += Cb2i_dot[1][2] * dt
    Cb2i_gyro[2][2] += Cb2i_dot[2][2] * dt


def imu_init(init_acc,Cb2i_gyro):
    #Initalize a Gyro(disregarding rotation of the earth) pg6 - 4

    #init_acc = [ax_average, ay_average, az_average, acc_average_magnitude]
    Cb2i_gyro[2][0] = init_acc[0] / init_acc[3]
    Cb2i_gyro[2][1] = init_acc[1] / init_acc[3]
    Cb2i_gyro[2][2] = init_acc[2] / init_acc[3]

    Cb2i_gyro[1][0] = 0
    Cb2i_gyro[1][1] = Cb2i_gyro[2][2] / np.sqrt((Cb2i_gyro[2][1] * Cb2i_gyro[2][1]) + (Cb2i_gyro[2][2] * Cb2i_gyro[2][2]))
    Cb2i_gyro[1][2] = -Cb2i_gyro[2][1] / np.sqrt((Cb2i_gyro[2][1] * Cb2i_gyro[2][1]) + (Cb2i_gyro[2][2] * Cb2i_gyro[2][2]))

    Cb2i_gyro[0][0] = (Cb2i_gyro[1][1] * Cb2i_gyro[2][2]) - (Cb2i_gyro[1][2] * Cb2i_gyro[2][1])
    Cb2i_gyro[0][1] = (Cb2i_gyro[1][2] * Cb2i_gyro[2][0]) - (Cb2i_gyro[1][0] * Cb2i_gyro[2][2])
    Cb2i_gyro[0][2] = (Cb2i_gyro[1][0] * Cb2i_gyro[2][1]) - (Cb2i_gyro[1][1] * Cb2i_gyro[2][0])


#Calculates the DCM error based on a target dcm(Cb2i_target) and current dcm estimate(Cb2i), replacing an error matrix(Cb2i_error)
def dcm_error(Cb2i_target,Cb2i,Cb2i_error):

    #DCM error Cb2i_error = Cb2i_target x Cb2i ^ T

    Cb2i_T = [[0,0,0],[0,0,0],[0,0,0]]
    transpose3x3(Cb2i, Cb2i_T)
    cross3x3(Cb2i_target, Cb2i_T, Cb2i_error)

#Complimentary filter(seriously not reccomended by paul......but i am not paul so go for it dude)
def complimentary_filter(comp_gain,gyro_dcm,acc_dcm,Cb2i):
    #get an attiude estimate by smashing a traingle into a square hole, not tottaly stupid but your only half way there

    Cb2i[0][0] = (gyro_dcm[0][0] * comp_gain) + (acc_dcm[0][0] * (1 - comp_gain))
    Cb2i[1][0] = (gyro_dcm[1][0] * comp_gain) + (acc_dcm[1][0] * (1 - comp_gain))
    Cb2i[2][0] = (gyro_dcm[2][0] * comp_gain) + (acc_dcm[2][0] * (1 - comp_gain))

    Cb2i[0][1] = (gyro_dcm[0][1] * comp_gain) + (acc_dcm[0][1] * (1 - comp_gain))
    Cb2i[1][1] = (gyro_dcm[1][1] * comp_gain) + (acc_dcm[1][1] * (1 - comp_gain))
    Cb2i[2][1] = (gyro_dcm[2][1] * comp_gain) + (acc_dcm[2][1] * (1 - comp_gain))

    Cb2i[0][2] = (gyro_dcm[0][2] * comp_gain) + (acc_dcm[0][2] * (1 - comp_gain))
    Cb2i[1][2] = (gyro_dcm[1][2] * comp_gain) + (acc_dcm[1][2] * (1 - comp_gain))
    Cb2i[2][2] = (gyro_dcm[2][2] * comp_gain) + (acc_dcm[2][2] * (1 - comp_gain))


# NOT TESTED

#the result matrix is replaced with the cross product of by3 andby1

def cross3x3x3x1(by3, by1, result):
    result[0] = (by3[0][0] * by1[0]) + (by3[0][1] * by1[1]) + (by3[0][2] * by1[2])
    result[1] = (by3[1][0] * by1[0]) + (by3[1][1] * by1[1]) + (by3[1][2] * by1[2])
    result[2] = (by3[2][0] * by1[0]) + (by3[2][1] * by1[1]) + (by3[2][2] * by1[2])



def acc2intertial(acc,Cb2i,acc_i):
    #pg 4 - 10 Eq(4.2 - 1)
    cross3x3x3x1(Cb2i, acc, acc_i)


