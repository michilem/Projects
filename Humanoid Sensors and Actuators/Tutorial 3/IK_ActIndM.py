# -*- coding: utf-8 -*-
"""
Created on Mon May 27 23:07:10 2019

@author: john
"""
import sympy as sym
from numpy import *
import math 
from FK_ActIndM import *

X = sym.Symbol('X')
Y = sym.Symbol('Y')
Z = sym.Symbol('Z')

# the input is a transformation matrix: 


x_val = Tr[0][3]
y_val = Tr[1][3]
z_val = Tr[2][3]


P = sym.atan2(Y,X)

K = (2.0*(sym.sqrt((X**2)+(Y**2))))/((X**2)+(Y**2)+(Z**2))

theta1 = sym.acos(1.0-(K*(sym.sqrt((X**2)+(Y**2))))) 
theta2 = (2.0*pi)-(sym.acos(1.0-(K*(sym.sqrt((X**2)+(Y**2))))))

# the output is the arc's parameters: 

T = P 
T = T.subs('X',x_val)
T = T.subs('Y',y_val)

print ("P:")
print (T)
p_val = T

T = K 
T = T.subs('X',x_val)
T = T.subs('Y',y_val)
T = T.subs('Z',z_val)

print ("K:")
print (T)
k_val = T


if (z_val > 0):
    T = theta1
else:
    T = theta2

T = T.subs('X',x_val)
T = T.subs('Y',y_val)
T = T.subs('Z',z_val)

print ("theta:")
print (T)
t_val = T

l_val = t_val/k_val
print ("L:")
print (l_val)
