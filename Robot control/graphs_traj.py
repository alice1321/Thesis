#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from mpl_toolkits import mplot3d

coordinates = np.load('coord_demo_2.npy')
live_filtering = np.load('coord_demo_filtered_.npy')
#time = np.load('time.npy')
live_x = live_filtering[:,0]
live_y = live_filtering[:,2]
x = coordinates[:,0]
y = coordinates[:,1]
filtered_coordinates = []
xy_coordinates = coordinates[:,0:2]

delta_t = 0.35
filter = KalmanFilter(dim_x = 4, dim_z= 2)
filter.X = np.array([0., 0., 0., 0.]) #initial state
filter.F = np.array([[1., delta_t, 0., 0.], [0., 1., 0., 0.], [0., 0., 1., delta_t], [0., 0., 0., 1.]])
filter.H = np.array([[1., 0., 0., 0.],[0., 0., 1., 0.]])
filter.P *= 40#variance squared -- 20 cm of estimation error
filter.R = np.array([[4.,0.],[0.,4.]])  #3cm of measurement error
filter.Q = Q_discrete_white_noise(dim=2, dt= 0.35, var = 0.25, block_size = 2)  # dim=2 constant velocity
#--------------------------------------------------------------------
i = 0
time_i = []

for position in xy_coordinates:
    i +=1
    filter.predict()
    filter.update(position)
    xy_filtered = filter.x   #diviso 100 per simulazione 
    z = 0.5
    filtered_coordinates.append(xy_filtered)
    time_i.append(i)

coordinates = np.array(filtered_coordinates)
x_filtered = coordinates[:,0,0]
y_filtered = coordinates[:,2,0] 

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x, y, time_i) #real
ax.plot3D(x_filtered, y_filtered, time_i) #offline filtering
ax.plot3D(live_x[:,0]*100, live_y[:,0]*100, time_i) #live filtering
plt.show()