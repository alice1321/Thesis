#! /usr/bin/env python3
from lwr_controllers.msg import PoseRPY
import rospy 
from geometry_msgs.msg import Pose, Vector3, Point, PointStamped
from lwr_controllers.msg import RPY
import math
import time
from tf import TransformListener
import numpy as np
from tf import transformations
from scipy.spatial.transform import Rotation as R
#camera rotation wrt the tip

T = np.array(([-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]))  #transform from camera to EE 180 degrees rotz with the specific initial position

scale_factor_x =1
scale_factor_y = 1

'''tollerance = 0.01
roll = 3.11061639843
pitch = -0.0281753618245
yaw = -3.1411926646
initial_position = Vector3(-0.388466364786, 0.425485396391 ,0.314065904683) #right one
initial_orientation = RPY(roll, pitch, yaw)
#initial_position = Vector3(-0.528641162061, 0.27358276482 ,0.337287984555)
'''
roll = -3.13796658242
pitch = -0.0380158347695
yaw = -3.11634577067

initial_position = Vector3(-0.414228491773, 0.390018906215 ,0.405499848437)  #to be set
initial_orientation = RPY(roll, pitch, yaw)

cartesian_transform = transformations.euler_matrix(roll, pitch, yaw, axes ='sxyz')
cartesian_transform[0,3] = initial_position.x
cartesian_transform[1,3] = initial_position.y
cartesian_transform[2,3] = initial_position.z




def callback(pos):
    global position
    position = pos

def get_actual_position(pose):
    global actual_position 
    global actual_orientation
    actual_position = pose.position
    actual_orientation = pose.orientation



def main():
    rospy.init_node('send_pose', anonymous=True)
    rospy.Subscriber('/tool_pose', PointStamped, callback)
    rospy.Subscriber('/cartesian_position/pose/tooltip', PoseRPY, get_actual_position)
    publisher = rospy.Publisher('/lwr/one_task_inverse_kinematics/command', PoseRPY, queue_size=10)
    publisher1 = rospy.Publisher('position', PointStamped, queue_size =10)
    rate = rospy.Rate(10)
    rospy.sleep(1)
    command = PoseRPY(0, initial_position, initial_orientation)
    publisher.publish(command)
    time.sleep(1)
    now = rospy.Time.now() + rospy.Duration(0.9)
    listener = TransformListener()
    listener.waitForTransform('lwr_base_link', 'lwr_7_link', now, rospy.Duration(1.0))
    final_position = Vector3()
    final_position.z = 0.337287984555  #to be set
    #(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))
    #transform = listener.fromTranslationRotation(trans, rot)

    while not rospy.is_shutdown():
	
	point = transformations.translation_matrix((position.point.x, position.point.y, position.point.z)) #real point
#--------------use listener transform point----------
	wait = position.header.stamp + rospy.Duration(0.3)	
	listener.waitForTransform('lwr_base_link', 'lwr_7_link', wait , rospy.Duration(2.0))
	position_base = listener.transformPoint('lwr_base_link', position)
	position_base.header.stamp = rospy.Time.now()
	publisher1.publish(position_base)
	final_position.x = position_base.point.x
	final_position.y = position_base.point.y
	final_position.z = position_base.point.z + 0.1
	#print('listener',final_position)

#---------------------tf transform---------------------	
        #(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))
	#transform = listener.fromTranslationRotation(trans, rot)
	#position_new = np.dot(T, point[:,-1])

	#position_new = np.dot(transform, point[:,-1])
#-----------------------------------------------------------	
#-------------------tooltip transform-------------------
	#position_new = np.dot(T, point[:,-1])
	#position_new = np.dot(cartesian_transform, position_new)
	#print('tooltip_new', position_new)
#------------------------------------------------------------------

	#final_position.x = position_new[0]
        #final_position.y = position_new[1]
	command = PoseRPY(0, final_position, initial_orientation)
	
	#print('f',final_position)
	#print('i',initial_position)
	#print(position.x)
	#print(position.y)
	#print('point',position_new)
	if abs(actual_position.x - final_position.x) < 0.11 and abs(actual_position.y - final_position.y) < 0.07:
	    publisher.publish(command)

	#print('transform',transform)
	#print(final_position)


if __name__ == '__main__':
    main()
