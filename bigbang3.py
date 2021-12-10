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
N =200 #num. bolitas
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
detcol = False #flag desactiva deteccion de colisiones hasta que no se separen todas las bolas


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

    #rate(400)
   
    org = org + vel * deltat #actualiza coordenadas

    #chequea colisiones con esfera limite
    for i in range(n): 
        balls[i].pos = vector(org[i,0],org[i,1],org[i,2])
        balls[i].velocity = vector( vel[i,0], vel[i,1], vel[i,2])
        modulo = mag(balls[i].pos)
        chklim = (modulo + rad[i,0]) >= wsize
        if chklim :
            k = 1 - (modulo +rad[i,0] - wsize)/modulo
            org[i,:] =  k * org[i,:]
            vel[i,:] -= 2 * org[i,:] * np.dot(org[i,:],vel[i,:])/modulo**2


    diff = org.reshape(n,1,3) - org #distancia entre bolas
    d = (diff ** 2).sum(2) #array nxn distancia entre bolas 2 a 2
    i = np.arange(n)
    d[i,i] = np.inf
    chkdst = d < dst #array que indica true si dos bolas han chocado
    coords = np.argwhere(chkdst) #coords lista de filas y cols (numero de bola) que han colisionado
    if len(coords) <= N / 20:
        detcol = True #en el momento en que ya se han separado todas las bolas comienza deteccion colisiones
        
    
    if detcol: #no empieza a detectar colisiones hasta pasados xxx frames, sino se autodestruiria

        #print(coords)
        coll =[]

        #generamos list de los pares de bolas que colisionan
        for a in range(len(coords)):
            if coords[a,0] < coords[a,1]: 
                coll.append(coords[a]) #lista de pares de bolas que han colisionado
                
        delb = [] #lista de bolas a borrar
        #collision action: Bigger ball increases radius, smaller gets deleted
        for i in range(len(coll)):  #coll lista de pares de bolas en colision

            b1 = coll[i][0] #indice primera bola
            b2 = coll[i][1] #indice segunda bola
            r1 = balls[b1].radius
            r2 = balls[b2].radius
            
            n = n-1
    
            if r1 > r2 :
                balls[b1].radius = (r1**3 + r2**3)**(1.0/3)
                rad[b1,0] = balls[b1].radius
                #remove smaller ball
                delb.append(b2)

            else:
                balls[b2].radius = (r1**3 + r2**3)**(1.0/3)
                rad[b2,0] = balls[b2].radius
                delb.append(b1)

        delb.sort(reverse=True)
        for i in range(len(delb)):
            idx = delb[i]
            rad= np.delete(rad,idx,0) #elimina radio bola eliminada
            balls[idx].visible = False #elimina de la escena
            balls.pop(idx) #elimina de la lista
            org = np.delete(org,idx,0) #elimina datos coordenadas bola
            vel = np.delete(vel,idx,0) #elimina datos velocidad bola
            col = np.delete(col,idx,0) #elimina datos color bola2
        dst = rad.reshape(1,n) + rad #recalcula nueva matriz distancias
        dst = dst**2# distancia al cuadrado de la suma de radios dos a dos

            
    t = t + deltat                
    if n == 1: # gravitational collapse
        rad = rad * 0.999
        balls[0].radius = rad[0,0]
        if rad[0,0] < (wsize / (2 * N)): # la bola ha colapsado ?
            balls[0].visible = False #elimina ultima bola de la escena
            
            # restart big- bang
            n = N #contador bolitas
            t=0
            rad = np.random.rand(n,1) #array radios de las bolas
            #rad = wsize/(4 * n ) * (1 + rad )
            rad = wsize/8 * (1 + rad )/(n ** (1.0 / 3))
            dst = rad.reshape(1,n) + rad #calcula matriz distancias para detectar colisiones
            dst = dst**2# distancia al cuadrado de la suma de radios dos a dos
            pos = org[0]
            org = np.zeros((n,3)) #array coordenadas origen de las bolas
            org[:] = pos
 
            vel = np.random.rand(n,3) #array velocidades bolas
            vel = -5 + 10 * vel
            col = np.random.rand(n,3) #array de colores de las bolas
            col = 0.5 + 0.5 * col
            balls = [] #lista bolas
            detcol = False
            for i in range(n):
                ball = sphere(pos=vector(org[i,0],org[i,1],org[i,2]), radius=rad[i,0] , color=vector(col[i,0], col[i,1], col[i,2]) )
                ball.velocity = vector( vel[i,0], vel[i,1], vel[i,2])
                balls.append(ball)














