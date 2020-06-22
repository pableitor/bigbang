# -*- coding: utf-8 -*-
"""
modelo big bang 3D.
Universo cubico 
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
N = 20 #num. bolitas
n = N #contador bolitas
wsize = 10 #medida del espacio
rad = np.random.rand(n,1) #array radios de las bolas
rad = wsize/(4 * n ) * (1 + rad )
dst = rad.reshape(1,n) + rad #calcula matriz distancias para detectar colisiones
dst = dst**2# distancia al cuadrado de la suma de radios dos a dos

org = np.zeros((n,3)) #array coordenadas origen de las bolas
#org = -4 + 8 * org 
vel = np.random.rand(n,3) #array velocidades bolas
vel = -5 + 10 * vel
col = np.random.rand(n,3) #array de colores de las bolas
col = 0.5 + 0.5 * col
balls = [] #lista bolas

walls = [vector(wsize,0,0),vector(-wsize,0,0),vector(0,wsize,0),vector(0,-wsize,0),vector(0,0,wsize),vector(0,0,-wsize)]
for i in range(n):
    ball = sphere(pos=vector(org[i,0],org[i,1],org[i,2]), radius=rad[i,0] , color=vector(col[i,0], col[i,1], col[i,2]) )
    ball.velocity = vector( vel[i,0], vel[i,1], vel[i,2])
    balls.append(ball)
wallR = box(pos=walls[0], size=vector(0.2,2*wsize,2*wsize), color=color.green, opacity=0.5)
wallL = box(pos=walls[1], size=vector(0.2,2*wsize,2*wsize), color=color.red, opacity=0.5)
wallT = box(pos=walls[2], size=vector(2*wsize,0.2,2*wsize), color=color.blue, opacity=0.5)
wallD = box(pos=walls[3], size=vector(2*wsize,0.2,2*wsize), color=color.blue, opacity=0.5)
wallF = box(pos=walls[4], size=vector(2*wsize,2*wsize,0.2), color=color.yellow, opacity=0)
wallB = box(pos=walls[5], size=vector(2*wsize,2*wsize,0.2), color=color.yellow, opacity=0.5)


deltat = 0.005
t = 0
vscale = 0.1

while (True):

    #rate(400)
   
    org = org + vel * deltat #actualiza coordenadas
    #check rebote con las paredes derecha-izquierda (L - R)
    checkR = (org[:,0] + rad[:,0] >= wallR.pos.x) #testea limite derecho
    org[:,0] =  org[:,0] + checkR * (walls[0].x - (org[:,0] + rad[:,0]))
    vel[:,0] = vel[:,0] * (1 - 2 * checkR)
    checkL = (org[:,0] - rad[:,0] <= wallL.pos.x)  #testea limite izquierdo
    org[:,0] =  org[:,0] + checkL * (walls[1].x - (org[:,0] - rad[:,0]))
    vel[:,0] = vel[:,0] * (1 - 2 * checkL)
    #check rebote con las paredes arriba-abajo (T - D)
    checkT = (org[:,1] + rad[:,0] >= wallT.pos.y) #testea limite superior
    org[:,1] =  org[:,1] + checkT * (walls[2].y - (org[:,1] + rad[:,0]))
    vel[:,1] = vel[:,1] * (1 - 2 * checkT)
    checkD = (org[:,1] -rad[:,0] <= wallD.pos.y) #testea limite inferior
    org[:,1] =  org[:,1] + checkD * (walls[3].y - (org[:,1] - rad[:,0]))
    vel[:,1] = vel[:,1] * (1 - 2 * checkD)

    #check rebote con las paredes delante-detras (F - B)
    checkF = (org[:,2] + rad[:,0] >= wallF.pos.z) #testea limite frontal
    org[:,2] =  org[:,2] + checkF * (walls[4].z - (org[:,2] + rad[:,0]))
    vel[:,2] = vel[:,2] * (1 - 2 * checkF)
    checkB = (org[:,2] -rad[:,0] <= wallB.pos.z) #testea limite trasero
    org[:,2] =  org[:,2] + checkB * (walls[5].z - (org[:,2] - rad[:,0]))
    vel[:,2] = vel[:,2] * (1 - 2 * checkB) 
        
    for i in range(n):
    #i =np.range(n)
        
        #actualiza posiciones y velocidades de las bolas
        balls[i].pos = vector(org[i,0],org[i,1],org[i,2])
        balls[i].velocity = vector( vel[i,0], vel[i,1], vel[i,2])
         
    diff = org.reshape(n,1,3) - org #distancia entre bolas
    d = (diff ** 2).sum(2) #array nxn distancia entre bolas 2 a 2
    i = np.arange(n)
    d[i,i] = np.inf
    chkdst = d < dst #array que indica true si dos bolas han chocado
    coords = np.argwhere(chkdst)
    #print(coords)
    coll =[]

    #generamos list de los pares de bolas que colisionan
    for a in range(len(coords)):
        if coords[a,0] < coords[a,1]: 
            coll.append(coords[a]) #lista de pares de bolas que han colisionado
    #print(coll)
    if t > 100*deltat: #no empieza a detectar colisiones hasta pasados 50 frames, sino se autodestruiria
        delb = [] #lista de bolas a borrar
        #collision action: Bigger ball increases radius, smaller gets deleted
        for i in range(len(coll)):  #coll lista de pares de bolas en colision

            b1 = coll[i][0] #indice primera bola
            b2 = coll[i][1] #indice segunda bola
            r1 = balls[b1].radius
            r2 = balls[b2].radius
            
            n = n-1
    
            if r1 > r2 :
                balls[b1].radius = r1 + r2
                rad[b1,0] = r1 + r2
                #remove smaller ball
                delb.append(b2)

            else:
                balls[b2].radius = r1 + r2
                rad[b2,0] = r1 + r2
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
            rad = wsize/(4 * n ) * (1 + rad )
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
            for i in range(n):
                ball = sphere(pos=vector(org[i,0],org[i,1],org[i,2]), radius=rad[i,0] , color=vector(col[i,0], col[i,1], col[i,2]) )
                ball.velocity = vector( vel[i,0], vel[i,1], vel[i,2])
                balls.append(ball)
            
  
            
