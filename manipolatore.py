#!/usr/bin/env python3
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from utilities import *

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

# Definisco i parametri di DH

a1=7.5
a2=6.5
a3=2
a4=0

d1=6.5
d2=4.2
d3=3.7
d4=0.5

alpha1=np.pi/2
alpha2=-np.pi/2
alpha3=np.pi/2
alpha4=0

a=[a1,a2,a3,a4]
d=[d1,d2,d3,d4]
alpha=[alpha1, alpha2, alpha3, alpha4]
lmax = sum(a)+sum(d); 

# Configurazione iniziale
q1 = np.pi/2
q2 = 0
q3 = -np.pi/4
q4 = np.pi
qe_i = np.array([q1,q2,q3,q4])
qe = []
qd = []
qe[0] = qe_i
qd[0] = qe_i #Chiedere al prof

T01 = DH_computation(d1,a1,alpha1,q1)
T12 = DH_computation(d2,a2,alpha2,q2)
T23 = DH_computation(d3,a3,alpha3,q3)
T34 = DH_computation(d4,a4,alpha4,q4)
T02 = np.matmul(T01,T12)
T03 = np.matmul(T02,T23)
T04 = np.matmul(T03,T34)
p_i = DirectKinematics(T04) #è la mia pA

# FAI LA STESSA COSA PER LA CONFIGURAZIONE FINALE

# Definisco alcuni valori utili
xd = p_i
xddot = np.array([0,0,0])
xe = p_i #l LEGGO DAI SENSORI
pA = [-0.7,1.1,-0.1]

tA = 0
tB = 5
tC=10
delta_t = 1
time = np.arange(tA,tC,delta_t)
K = 10
pB = np.array([1, 1, 1])
pC=np.array([1.97,1.45,-1.20])
ro=2
Cen=np.array([0, 0, 0])
deg=np.pi/4
qddot = []
error = []

for i in range(1,len(time)):
    [xd[i,:],xddot[i,:]] = cartesianPlanner(pA,pB,pC,ro,deg,Cen,tA,tB,tC,time[i])
        
        
    # qe1 = lm1.position
    # qe2 = qe1 + lm2.position
    # qe3 = qe2 + lm3.position
    # qe4 = qe3 + lm3.position
    # qe[i] = np.array([qe1,qe2,qe3,qe4])
    qddot[i,:],error[i,:] = InverseKinematicsProgetto(qd[i-1,:],a,xd[i,:],xddot[i,:],5,"i",K)
    qd[i,:] = qd[i-1,:] + np.matmul(qddot[i,:],delta_t)
    # PAUSA
    lm1.on_to_position(10,qd[i,0])
    lm2.on_to_position(10,qd[i,1])
    lm3.on_to_position(10,qd[i,2])
    mm1.on_to_position(10,qd[i,3])


