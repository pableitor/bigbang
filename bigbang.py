# -*- coding: utf-8 -*-
"""
Created on Sat Jun 20 09:32:43 2020

@author: ptorr
"""


import numpy as np
from vpython import *
n = 200 #num. bolitas
rad = np.random.random((n,1))
rad = 0.25 + 0.5 * rad

org = np.zeros((n,3))
#org = -4 + 8 * org 
vel = np.random.random((n,3))
vel = -5 + 10 * vel
col = np.random.random((n,3))
col = 0.5 + 0.5 * col
balls = []
walls = [vector(6,0,0),vector(-6,0,0),vector(0,6,0),vector(0,-6,0),vector(0,0,6),vector(0,0,-6)]
for i in range(n):
    ball = sphere(make_trail=False, retain=50, pos=vector(org[i,0],org[i,1],org[i,2]), radius=rad[i,0] , color=vector(col[i,0], col[i,1], col[i,2]) )
    ball.velocity = vector( vel[i,0], vel[i,1], vel[i,2])
    balls.append(ball)
wallR = box(pos=walls[0], size=vector(0.2,12,12), color=color.green, opacity=0.5)
wallL = box(pos=walls[1], size=vector(0.2,12,12), color=color.red, opacity=0.5)
wallT = box(pos=walls[2], size=vector(12,0.2,12), color=color.blue, opacity=0.5)
wallD = box(pos=walls[3], size=vector(12,0.2,12), color=color.blue, opacity=0.5)
wallF = box(pos=walls[4], size=vector(12,12,0.2), color=color.yellow, opacity=0)
wallB = box(pos=walls[5], size=vector(12,12,0.2), color=color.yellow, opacity=0.5)


deltat = 0.005
t = 0
vscale = 0.1

while True:
    #rate(400)
    for i in range(n):
        ball = balls[i]
        if (ball.pos.x + ball.radius) >= wallR.pos.x:
            ball.velocity.x = -ball.velocity.x
        if (ball.pos.x - ball.radius ) <= wallL.pos.x:
            ball.velocity.x = -ball.velocity.x
        if (ball.pos.y + ball.radius ) >= wallT.pos.y:
            ball.velocity.y = -ball.velocity.y
        if (ball.pos.y - ball.radius ) <= wallD.pos.y:
            ball.velocity.y = -ball.velocity.y
        if (ball.pos.z + ball.radius ) >= wallF.pos.z:
            ball.velocity.z = -ball.velocity.z
        if (ball.pos.z - ball.radius ) <= wallB.pos.z:
            ball.velocity.z = -ball.velocity.z
        ball.pos = ball.pos + ball.velocity*deltat
        balls[i] = ball

        org = org + vel * deltat


    t = t + deltat
