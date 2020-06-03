# -*- coding: utf-8 -*-
"""
Created on Tue May 19 16:23:04 2020

@author: Lenovo Admin
"""

import math

def polar(r,theta):
    x=r*math.cos(theta)          # For getting first polar co-ordinates
    y=r*math.sin(theta)          # For getting second polar co-ordinates
    return (x,y)
def cart(x,y):
    r=math.sqrt(x**2+y**2)      # For getting first cartesian co-ordinates
    theta=1/math.tan(y/x)       # For getting second cartesian co-ordinates
    return (r,theta)
print('Enter 1 for converting cartesian to polar co-ordinates and 2 for converting polar to cartesian')
z=input('Enter 1 or 2')        # To make choice

def first(a):
    r=int(input('Enter first polar co-ordinate'))
    theta=int(input('Enter second polar co-ordinate'))
    print('cartesian co-ordinate are',polar(r,theta))

def second(b):
    x=int(input('Enter first cartesian co-ordinate'))
    y=int(input('Enter second cartesian co-ordinate'))     
    print('polar co-ordinate are',cart(x, y))
    
if z==1:
    print(first(a))
if z==2:
    print(second(b))


    





 


    
    
        