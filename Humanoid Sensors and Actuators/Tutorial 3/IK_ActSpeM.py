# -*- coding: utf-8 -*-
"""
Created on Mon May 27 22:11:57 2019

@author: john
"""
import sympy as sym
from numpy import *
import math 
from dh import * 
from IK_ActIndM import *

# the input is the arc's parameters: 
# Tested values:
# 1. L = 2, T = pi, P = pi/4
# 2. L = 5, T = pi/2, P = pi/2
# 3. L = 5, T = pi/4, P = 0

#l_val = 2.0
#t_val = pi
#p_val = pi/4

###############
# inverse kinematics for actuator specific mapping 

L = sym.Symbol('L')
L1 = sym.Symbol('L1')
L2 = sym.Symbol('L2')
L3 = sym.Symbol('L3')
L4 = sym.Symbol('L4')
d = sym.Symbol('d')
t = sym.Symbol('t')
P1 = sym.Symbol('P1')
P2 = sym.Symbol('P2')
P3 = sym.Symbol('P3')
P4 = sym.Symbol('P4')
P = sym.Symbol('P')

d_val = 0.1

L1 = L-t*d*sym.cos(P1)
L1 = L1.subs('P1',pi/4.0-P)

L2 = L-t*d*sym.cos(P2)
L2 = L2.subs('P2',3*pi/4.0-P)

L3 = L-t*d*sym.cos(P3)
L3 = L3.subs('P3',5*pi/4.0-P)

L4 = L-t*d*sym.cos(P4)
L4 = L4.subs('P4',7*pi/4.0-P)
######

# the output is the strings' lengths: 

T = L1
T = T.subs('L',l_val)
T = T.subs('d',d_val)
T = T.subs('P',p_val)
T = T.subs('t',t_val)
print ("L1:")
print (T) 

T = L2
T = T.subs('L',l_val)
T = T.subs('d',d_val)
T = T.subs('P',p_val)
T = T.subs('t',t_val)
print ("L2:")
print (T) 

T = L3
T = T.subs('L',l_val)
T = T.subs('d',d_val)
T = T.subs('P',p_val)
T = T.subs('t',t_val)
print ("L3:")
print (T) 

T = L4
T = T.subs('L',l_val)
T = T.subs('d',d_val)
T = T.subs('P',p_val)
T = T.subs('t',t_val)
print ("L4:")
print (T)

