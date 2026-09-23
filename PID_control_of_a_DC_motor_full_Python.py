####################################################################################################################
# Libraries
####################################################################################################################
import time
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime
import serial
import struct



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
It = 0



####################################################################################################################
# Sampling period
####################################################################################################################
Ts = 0.0001 # 1 ms



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
final_time = 20000



####################################################################################################################
# Serial communication (USART) variables
####################################################################################################################
port = 'COM3'
baudrate = 115200
serial_initialization = 0



####################################################################################################################
# Plot definition and variables
####################################################################################################################
#plt.ion() # activating the interactive plot mode to see the motor evolution in real-time
fig1, ax1 = plt.subplots(figsize=(10,6))
line1, = ax1.plot([], [], 'r-')
ax1.set_xlim(0, final_time*Ts)
ax1.set_ylim(-13, 20)
x_values = []
tim = 0
y1_values = []

##fig2, ax2 = plt.subplots()
##line2, = ax2.plot([], [], 'g-')
##ax2.set_xlim(0, final_time/1000)
##ax2.set_ylim(-5.5, 5.5)
y2_values = []

##fig3, ax3 = plt.subplots()
##line3, = ax3.plot([], [], 'b-')
##ax3.set_xlim(0, final_time/1000)
##ax3.set_ylim(-2, 2)
y3_values = []


####################################################################################################################
# Functions
####################################################################################################################
def serial_comm_init(serial_initialization):
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Successfully connected to port {port} at {baudrate} baud")
    time.sleep(2)
    return ser

def send_and_receive(uint16):
    msg = int(uint16/(6*3.1416)*65535)
    if msg > 65535:
        msg = 65535
    elif msg < 0:
        msg = 0
    msg = msg.to_bytes(2, byteorder='little',signed=False)
    ser.write(msg)
    ser.flush()

    u_raw = ser.read(2)
    if len(u_raw) == 2:
        u_unpack = struct.unpack('H', u_raw)[0]
        u_norm = (u_unpack/2730)-12
    else:
        TimeoutError("STM32 didn't respond at time")
    ref_raw = ser.read(2)
    if len(ref_raw) == 2:
            ref_unpack = struct.unpack('H', ref_raw)[0]
            ref_norm = ref_unpack/217.2465
    else:
        TimeoutError("STM32 didn't respond at time")
    return u_norm, ref_norm

def save_data(ref_val, Th, u, tim):
    x_values.append(tim)
    y1_values.append(ref_val)
    y2_values.append(Th)
    y3_values.append(u)
    tim += Ts
    return tim

def plot_data():
    # Plotting and debugging code
    ax1.plot(x_values, y1_values, 'r-', label='Ref (rad)')
    ax1.plot(x_values, y2_values, 'b-', label='Output (rad)')
    ax1.plot(x_values, y3_values, 'g-', label='Current (A)')
    plt.show()

def reference_update(ref_num):
    Th_final = (ref_num/4095*24)-12
    return Th_final

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

    # Limiting the control action
    if u > 12:
        u = 12
    elif u < -12:
        u = -12
    return u

def next_step_values(TM, Wt_prev, It_prev, eb_prev, Th, u):
    # Computing the derivative of the angular speed (dWt) and the current (dIt)
    dWt = (TM - B*Wt_prev - TL)/J
    dIt = (u - R*It_prev - eb_prev)/L

    # Computing all the variables for the next step
    Wt = Wt_prev + dWt*Ts   # angular speed in rad/s
    Th = Th + Wt*Ts         # angular position in rad
    if Th < 0:
        Th = 0
    It = It_prev + dIt*Ts   # current in A
    TM = Kt*It         # mechanical torque of the motor in Nm
    eb = Ke*Wt         # electromotive inducted force in V

    # preparing the next step calculations
    Wt_prev = Wt
    It_prev = It

    return TM, Wt_prev, It_prev, eb, Th, Wt, It

def serial_comm_finish(serial_initialization):
    ser.close()
    print(f"Port {port} has been closed")


####################################################################################################################
# Loop
####################################################################################################################
# Initializing the USART communication
ser = serial_comm_init(serial_initialization)

while counter2 < final_time:
    
##    if ser.in_waiting > 0: # Reading USART port if there is an incoming message
        
    # Reading the control action that the STM32 Nucleo-64 has computed
    u_val, ref_val = send_and_receive(Th)
        
    # Calculating the next step values of all state variables
    TM, Wt_prev, It_prev, eb, Th, Wt, It = next_step_values(TM, Wt_prev, It_prev, eb, Th, u_val)
    tim = save_data(ref_val, Th, u_val, tim)

    if counter2 == (final_time/2):
        TL = 0.003
    if counter2 == (final_time/5*3):
        TL = 0
        
    counter2 += 1
    counter1 += 1
    if counter1 == 1000:
        print(ref_val)
        counter1 = 0
    if counter2 == final_time-1:
        plot_data()
    # Plotting and debugging the motor status
#    tim = save_data(Th_final, Th, u, tim)

    # Computing the position reference
#    Th_final = reference_update(ref_num)
    
    # Comparing the reference value with the output
#    u = motor_control(err_acc, err_prev, Ed_prev, tim)

    # Calculating the next step values of all state variables
#    TM, Wt_prev, It_prev, eb, Th, Wt, It = next_step_values(TM, Wt_prev, It_prev, eb, Th)
    
    # plus one step
#    counter1 += 1
#    counter2 += 1

#    if counter2 == (final_time-1):
#        plot_data()
