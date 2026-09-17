####################################################################################################################
# Libraries
####################################################################################################################
import time
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime
import serial



####################################################################################################################
# DC motor parameters (provisionals)
####################################################################################################################
L = 0.0006      # inductance
R = 2.5         # resistance
Ke = 0.015      # electrical constant
J = 0.000015    # rotor inertia
B = 0.000035    # friction coeficient
TL = 0          # load torque (0 = no load)
Kt = Ke         # motor torque constant



####################################################################################################################
# Initialization of variables
####################################################################################################################
Wt_prev = 0     # initialization of angular speed
It_prev = 0     # initialization of current
TM = 0          # initialization of mechanical torque generated on the motor due to the current
eb = 0          # electromotive inducted force
Th_final = 100  # reference position in rad
Th = 0          # initialization of motor position
u = 0



####################################################################################################################
# Sampling period
####################################################################################################################
Ts = 0.0001 # 0.1 ms



####################################################################################################################
# DC motor equations
####################################################################################################################
#V = L*dIt + R*It + eb
#eb = Ke*Wt
#TM = J*dWt + B*Wt + TL
#TM = Kt*It



####################################################################################################################
# PID parameters
####################################################################################################################
##Kp = 1
##Ki = 0
##Kd = 0
Kp = 0.04
Ki = 2
Kd = 0.000018
alpha = 0.1
err_acc = 0
err_prev = 0
Ed_prev = 0



####################################################################################################################
# Code control variables
####################################################################################################################
counter1 = 0    # useful to print the results
counter2 = 0    # useful to finish the program
serial_init = 0
final_time = 30000



####################################################################################################################
# Plot definition and variables
####################################################################################################################
#plt.ion() # activating the interactive plot mode to see the motor evolution in real-time
fig1, ax1 = plt.subplots()
line1, = ax1.plot([], [], 'r-')
ax1.set_xlim(0, 2.5)
ax1.set_ylim(0, 110)
x_values = []
t = 0
y1_values = []

fig2, ax2 = plt.subplots()
line2, = ax2.plot([], [], 'g-')
ax2.set_xlim(0, final_time/10000)
ax2.set_ylim(-0.1, 12.1)
y2_values = []

fig3, ax3 = plt.subplots()
line3, = ax3.plot([], [], 'b-')
ax3.set_xlim(0, final_time/10000)
ax3.set_ylim(-0.1, 4)
y3_values = []


####################################################################################################################
# Functions
####################################################################################################################
def plot_data(count, tim, It):
    # Plotting and debugging code
    #if count == 0: # plotting and debugging with a period of 0.01s
        #print(Th, tim)
        #count = 0
    x_values.append(tim)
    tim += 0.0001
    y1_values.append(Th)
    y2_values.append(u)
    line1.set_data(x_values, y1_values)
    line2.set_data(x_values, y2_values)
    return count, tim

def motor_control(err_acc, err_prev, Ed_prev, tim):
    err = Th_final - Th
    err_acc = err_acc + err*Ts
    Ep = Kp*err
    Ei = Ki*err_acc
    Ed_no_filt = Kd*(err-err_prev)/Ts
    Ed = (1-alpha)*Ed_prev + alpha*Ed_no_filt
    err_prev = err
    Ed_prev = Ed
    u = Ep + Ei + Ed

    # Limiting the control action for a L298N driver, as its maximum current is 2A
    if u > R*2:
        u = R*2
    elif u < -2*R:
        u = -2*R
    return u

def next_step_values(TM, Wt_prev, It_prev, eb_prev, Th):
    # Computing the derivative of the angular speed (dWt) and the current (dIt)
    dWt = (TM - B*Wt_prev - TL)/J
    dIt = (u - R*It_prev - eb_prev)/L

    # Computing all the variables for the next step
    Wt = Wt_prev + dWt*Ts   # angular speed in rad/s
    Th = Th + Wt*Ts         # angular position in rad
    It = It_prev + dIt*Ts   # current in A
    TM = Kt*It         # mechanical torque of the motor in Nm
    eb = Ke*Wt         # electromotive inducted force in V

    # preparing the next step calculations
    Wt_prev = Wt
    It_prev = It
    y3_values.append(It)
    line3.set_data(x_values, y3_values)

    return TM, Wt_prev, It_prev, eb, Th, Wt, It

def serial_comm(serial_init):
    if serial_init == 0:
        ser = serial.Serial('COM1', 9600, timeout=1)
        print(f"Connected to {ser.name}")
        serial_init = 1
    ser.write(b'Hello\n')


####################################################################################################################
# Loop
####################################################################################################################
while counter2 < final_time:
    # Plotting and debugging the motor status
    counter1, t = plot_data(counter1, t, It_prev)
    
    # Comparing the reference value with the output
    u = motor_control(err_acc, err_prev, Ed_prev, t)

    # Calculating the next step values of all state variables
    TM, Wt_prev, It_prev, eb, Th, Wt, It = next_step_values(TM, Wt_prev, It_prev, eb, Th)
    
    # plus one step
    counter1 += 1

    # waiting Ts
    #time.sleep(Ts)

    counter2 += 1

##    if counter2 == 2500:
##        Th_final = 150
##        serial_comm(serial_init)

##    if counter2 == 5000:
##        serial_comm(serial_init)

    if counter2 == (final_time-1):
##        plt.plot(x_values, y1_values)
##        plt.plot(x_values, y2_values)
        plt.show()
