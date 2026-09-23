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
# DC motor equations
####################################################################################################################
#V = L*dIt + R*It + eb
#eb = Ke*Wt
#TM = J*dWt + B*Wt + TL
#TM = Kt*It


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
Wt_prev = 0     # initial angular speed
It_prev = 0     # initial current
TM = 0          # initial mechanical torque generated on the motor due to the current
eb = 0          # initial electromotive inducted force
Th = 0          # initial motor position
It = 0          # initial motor current


####################################################################################################################
# Sampling period
####################################################################################################################
Ts = 0.0001 # 0.1 ms


####################################################################################################################
# Code control variables
####################################################################################################################
counter1 = 0    # useful to print the results
counter2 = 0    # useful to finish the program
final_time = 50000 # seconds multiplied by 10.000


####################################################################################################################
# Serial communication (USART) variables
####################################################################################################################
port = 'COM3'       # STM32 port definition
baudrate = 115200   # baud-rate


####################################################################################################################
# Plot definition and plot variables
####################################################################################################################
fig1, ax1 = plt.subplots(figsize=(10,6))
line1, = ax1.plot([], [], 'r-')
ax1.set_xlim(0, final_time*Ts)
ax1.set_ylim(-13, 20)
x_values = []
tim = 0
y1_values = []
y2_values = []
y3_values = []


####################################################################################################################
# Functions
####################################################################################################################
def serial_comm_init():
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


####################################################################################################################
# Loop
####################################################################################################################
# Initializing the USART communication
ser = serial_comm_init()

while counter2 < final_time:
    
    # Reading the control action that the STM32 Nucleo-64 has computed
    u_val, ref_val = send_and_receive(Th)
        
    # Calculating the next step values of all state variables
    TM, Wt_prev, It_prev, eb, Th, Wt, It = next_step_values(TM, Wt_prev, It_prev, eb, Th, u_val)
    tim = save_data(ref_val, Th, u_val, tim)

    if counter2 == (final_time/2):
        TL = 0.005
        
    counter2 += 1
    counter1 += 1
    if counter1 == 1000:
        print(Th)
        counter1 = 0
    if counter2 == final_time-1:
        plot_data()
