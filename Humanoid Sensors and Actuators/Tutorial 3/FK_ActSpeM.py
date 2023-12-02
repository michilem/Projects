# -*- coding: utf-8 -*-
"""
Created on Mon May 27 22:13:54 2019

@author: john
"""

import sympy as sym 
from numpy import *
import math 
from dh import * 

###################
# forward kinematics for actuator specific mapping 

L1 = sym.Symbol('L1')
L2 = sym.Symbol('L2')
L3 = sym.Symbol('L3')
L4 = sym.Symbol('L4')
P = sym.Symbol('P')
K = sym.Symbol('K')
d = sym.Symbol('d')

d_val = 0.1

# the input is the strings' lengths: 
# proposed new lengths:
# 1. L1 = 1.68584073464102 L2 = 2.00000000000000 L3 = 2.31415926535898 L4 = 2.00000000000000
# 2. L1 = 4.88892792654604 L2 = 4.88892792654604 L3 = 5.11107207345396 L4 = 5.11107207345396
# 3. L1 = 4.94446396327302 L2 = 5.05553603672698 L3 = 5.05553603672698 L4 = 4.94446396327302

l1_val = 1.68584073464102
l2_val = 2.00000000000000
l3_val = 2.31415926535898
l4_val = 2.00000000000000



L = (L1+L2+L3+L4)/4.0

P = pi/4.0 - sym.atan2((L2-L4),(L3-L1))

K = ((-3*L1+L2+L3+L4)*sym.sqrt((L3-L1)**2+(L2-L4)**2))/(d*(L1+L2+L3+L4)*(L3-L1))

# the output is arc's parameters: 

T = L
T = T.subs('L1',l1_val)
T = T.subs('L2',l2_val)
T = T.subs('L3',l3_val)
T = T.subs('L4',l4_val)
print ("L:")
print (T)
l_val = T

T = P
T = T.subs('L1',l1_val)
T = T.subs('L2',l2_val)
T = T.subs('L3',l3_val)
T = T.subs('L4',l4_val)
print ("P:")
print (T)
p_val = T 

T = K
T = T.subs('L1',l1_val)
T = T.subs('L2',l2_val)
T = T.subs('L3',l3_val)
T = T.subs('L4',l4_val)
T = T.subs('d',d_val)

print ("K:")
print (T)
k_val = T
