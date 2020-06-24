# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 19:01:08 2020
modelo big bang 3D.
Universo ESFERICO
En la colision la particula de mayor radio absorbe a la menor , los radios se suman
y la menor desaparece.
El tamaño de las particulas es aleatorio, teniendo en cuenta que al final solo queda una
particula, con un radio que sería la suma de todos los radios de las demas particulas
y con un diametro limitado aproximadamente a la mitad del espacio
Created on Sat Jun 20 09:32:43 2020

@author: ptorr
"""


import numpy as np
from vpython import *
N =10 #num. bolitas
n = N #contador bolitas
wsize = 20 #medida del espacio
rad = np.random.rand(n,1) #array radios de las bolas
#rad = wsize/(4 * n ) * (1 + rad )#calculo de radio: en las colisiones sumamos los radios y el total es aprox. 1/4 del tamaño del espacio
rad = wsize/8 * (1 + rad )/(n ** (1.0 / 3))
dst = rad.reshape(1,n) + rad #calcula matriz distancias para detectar colisiones
dst = dst**2# distancia al cuadrado de la suma de radios dos a dos

org = np.zeros((n,3)) #array coordenadas origen de las bolas
#org = -4 + 8 * org 
vel = np.random.rand(n,3) #array velocidades bolas
vel = -5 + 10 * vel
col = np.random.rand(n,3) #array de colores de las bolas
col = 0.5 + 0.5 * col
balls = [] #lista bolas



for i in range(n):
    ball = sphere(pos=vector(org[i,0],org[i,1],org[i,2]), radius=rad[i,0] , color=vector(col[i,0], col[i,1], col[i,2]) )
    ball.velocity = vector( vel[i,0], vel[i,1], vel[i,2])
    balls.append(ball)
#crea limite universo observable
limit = sphere(pos=vector(0,0,0), radius=wsize , color=color.blue, opacity = 0.3 )

deltat = 0.005
t = 0
vscale = 0.1

while (True):

    rate(400)
   
    org = org + vel * deltat #actualiza coordenadas

    #chequea colisiones con el limite
    for i in range(n): 
        balls[i].pos = vector(org[i,0],org[i,1],org[i,2])
        balls[i].velocity = vector( vel[i,0], vel[i,1], vel[i,2])
        modulo = mag(balls[i].pos)
        chklim = (modulo + rad[i,0]) >= wsize
        if chklim :
            k = 1 - (modulo +rad[i,0] - wsize)/modulo
            org[i,:] =  k * org[i,:]
            vel[i,:] = -vel[i,:] 



















