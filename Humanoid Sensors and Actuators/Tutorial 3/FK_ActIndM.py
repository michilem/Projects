# -*- coding: utf-8 -*-
"""
Created on Mon May 27 22:09:33 2019

@author: john
"""

import sympy as sym
from numpy import *
import math 
from dh import * 
from FK_ActSpeM import *

#######################################################
# forward kinematics for actuator independant mapping 

# the input is arc's parameters: 

# arc parameters 
P = sym.Symbol('P')
L = sym.Symbol('L')
K = sym.Symbol('K')


# Finding D-H matrix for eachtransofrmation in D-H table:
# DenHarMat2(cos_theta, sin_theta, cos_alpha, sin_alpha, a, d)
A0 = DenHarMat2(sym.cos(P), sym.sin(P), 0 ,-1.0, 0.0, 0.0)
A1 = DenHarMat2(sym.cos(K*L/2.0),sym.sin(K*L/2.0), 0.0,1.0, 0.0, 0.0)
A2 = DenHarMat2(1.0, 0.0 , 0.0, -1.0 , 0.0, (2.0*sym.sin(K*L/2.0))/K)
A3 = DenHarMat2(sym.cos(K*L/2.0),sym.sin(K*L/2.0), 0.0,1.0, 0.0, 0.0)
A4 = DenHarMat2(sym.cos(-P), sym.sin(-P),1.0,0.0, 0.0, 0.0)


# Finding the final transformation 
A01 = dot(A0,A1)
A02 = dot(A01,A2)
A03 = dot(A02,A3)
A04 = dot(A03,A4)

# Symplify the symbolic equation several times: 
A04 = sym.simplify(A04)
A04 = sym.simplify(A04)
A04 = sym.simplify(A04)
A04 = sym.simplify(A04)
print ("A01234= ")
print (A04)

# K= theta/L


#theta = 2*pi/4.0
#l_val = 1.0
#k_val = theta/l_val 
#p_val = 4.0*pi/4.0

print (" ")
T = A04
T = T.subs('K',k_val)
T = T.subs('L',l_val)
T = T.subs('P',p_val)

print(T)

# the output is a transformation matrix: 
Tr = zeros((4, 4))

i=0
j=0
for e in T:
    for f in e: 
        Tr[i][j]= f
        j = j+1 
        if j==4: 
            j=0
            i=i+1
            