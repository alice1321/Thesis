#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
#from geometry_msgs import Point, Quaternion
from sensor_msgs import msg
import rospy
import time
import math
from mpl_toolkits import mplot3d



#PROBLEMS
#setting the orientation of the EE doesn't make it stay in the desired orientation 
# compute camera position wrt the base of the robot 
# initialise the position of the robot in an appropriate pose and try to maintain the orientation of the EE
#first read the orientation on the real robot that is suitable for the positioning of the camera
#orientation of link 7 is not a good reference as is changes with rotation, batter use the orientation of the tool tip if available 
#fix only the rotation around one axis and let the others free



#_____________________________________________________

coordinates = np.load('coord_square_test.npy')
live_filtering = np.load('coord_square_test_filtered.npy')
live_x = live_filtering[:,0]
live_y = live_filtering[:,2]
x = coordinates[:,0]
y = coordinates[:,1]
filtered_coordinates = []
xy_coordinates = coordinates[:,0:2]
#-----------------------Filter sec order-------------------------------
delta_t = 0.35
filter = KalmanFilter(dim_x = 4, dim_z= 2)
filter.X = np.array([0., 0., 0., 0.]) #initial state
filter.F = np.array([[1., delta_t, 0., 0.], [0., 1., 0., 0.], [0., 0., 1., delta_t], [0., 0., 0., 1.]])
filter.H = np.array([[1., 0., 0., 0.],[0., 0., 1., 0.]])
filter.P *= 400#variance squared -- 20 cm of estimation error
filter.R = np.array([[9.,0.],[0.,9.]])  #3cm of measurement error
filter.Q = Q_discrete_white_noise(dim=2, dt= 0.35, var = 0.25, block_size = 2)  # dim=2 constant velocity
#--------------------------------------------------------------------

#-----------------------Filter third order-------------------------------
delta_t = 0.5
filter2 = KalmanFilter(dim_x = 6, dim_z= 2)
filter2.X = np.array([0., 0., 0., 0., 0., 0.]) #initial state
filter2.F = np.array([[1., delta_t, 0.5*delta_t*delta_t, 0., 0., 0.], [0., 1., delta_t, 0., 0., 0.], [0., 0., 1., 0., 0., 0.], [0., 0., 0., 1., delta_t, 0.5*delta_t*delta_t] ,[0., 0., 0., 0., 1., delta_t], [0., 0., 0., 0., 0., 1.]])
filter2.H = np.array([[1., 0., 0., 0., 0., 0.],[0., 0., 0., 1., 0., 0.]])
filter2.P *= 4000#variance squared -- 20 cm of estimation error
filter2.R = np.array([[9.,0.],[0.,9.]])  #3cm of measurement error
filter2.Q = Q_discrete_white_noise(dim=3, dt= 0.35, var = 0.25, block_size=2) #dim=3 constant acceleration
#--------------------------------------------------------------------
#coordinates_filtered = np.load('coord_square_test_filtered.npy')
#print(coordinates_filtered)
roll = 3.14152875778
pitch = 0.0401180364113
yaw = -2.1921722673

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main():

    rospy.init_node('graph', anonymous = True)
    publisher = rospy.Publisher('tool_pose', Vector3, queue_size=10)
    rate = rospy.Rate(1)
    global pose
    roll, pitch, yaw = euler_from_quaternion(-0.001, 0.702, 0.0, 0.712)
    i = 0
    print('main')
    while not rospy.is_shutdown():

        for position in xy_coordinates:
            i+=1
            filter.predict()
            filter.update(position)
            xy_filtered = filter.x/100   #diviso 100 per simulazione 
            z = 0.5
            position = Vector3(xy_filtered[0,0],xy_filtered[2,0],z)  #must calibrate the xy position with the position of the camera, use tf tree
            orientation = Quaternion(0.001, 0.702, 0.0, 0.712)
            pose = Pose(position,orientation)
            #print(xy_filtered)
            if xy_filtered[0]<1 and xy_filtered[2] < 1:
                print(pose)
                publisher.publish(position)
                time.sleep(0.1)
            filtered_coordinates.append(xy_filtered)
            
#print(np.shape(filtered_coordinates))
    coordinates = np.array(filtered_coordinates)
    x_filtered = coordinates[:,0,0]
    y_filtered = coordinates[:,2,0]  #x, x_dot, y, y_dot
    z_filtered = 0.5

    '''x_filtered = coordinates_filtered[:,0,0]/10
    y_filtered = coordinates_filtered[:,2,0]/10
    print(y_filtered)'''

    '''
    plt.plot(x, y)
    plt.plot(x_filtered, y_filtered)
    plt.plot(live_x*100,live_y*100)
    plt.xlabel("X")
    plt.ylabel('Y')
    plt.title('Trajectory')
    plt.show()
    #print(x)
    plt.plot(x_filtered, y_filtered)'''

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(x, y, i)

if __name__ == '__main__':
	main()
