#! /usr/bin/env python3
from lwr_controllers.msg import PoseRPY
import rospy 
from geometry_msgs.msg import Pose, Vector3, Point, PointStamped
from lwr_controllers.msg import RPY
import time
from tf import TransformListener
import numpy as np
from tf import transformations


#set initial orientation
roll = -3.13796658242
pitch = -0.0380158347695
yaw = -3.11634577067

initial_position = Vector3(-0.414228491773, 0.390018906215 ,0.18561169506)
initial_orientation = RPY(roll, pitch, yaw)


'''cartesian_transform = transformations.euler_matrix(roll, pitch, yaw, axes ='sxyz')
cartesian_transform[0,3] = initial_position.x
cartesian_transform[1,3] = initial_position.y
cartesian_transform[2,3] = initial_position.z'''


def get_desired_position(desired):
    global position
    position = desired

def get_actual_position(pose):
    global actual_position 
    global actual_orientation
    actual_position = pose.position
    actual_orientation = pose.orientation


def main():
    rospy.init_node('send_pose', anonymous=True)
    rospy.Subscriber('/tool_pose', PointStamped, get_desired_position)
    rospy.Subscriber('/cartesian_position/pose/tooltip', PoseRPY, get_actual_position)
    publisher = rospy.Publisher('/lwr/one_task_inverse_kinematics/command', PoseRPY, queue_size=10)
    position_publish = rospy.Publisher('position', PointStamped, queue_size=10)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    command = PoseRPY(0, initial_position, initial_orientation) #set initial position
    publisher.publish(command)
    time.sleep(2)

    now = rospy.Time.now() + rospy.Duration(0.9)
    listener = TransformListener()
    listener.waitForTransform('lwr_base_link', 'lwr_7_link', now, rospy.Duration(1.0))
    final_position = Vector3()
    final_position.z = 0.18561169506  #to be set
    #(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))
    #transform = listener.fromTranslationRotation(trans, rot)

    while not rospy.is_shutdown():

	
	#point = transformations.translation_matrix((position.x, position.y, position.z)) #real point
#--------------use listener transform point----------
	wait = position.header.stamp + rospy.Duration(0.3)
	listener.waitForTransform('lwr_base_link', 'lwr_7_link', wait, rospy.Duration(2.0))
	position_base = listener.transformPoint('lwr_base_link', position)
	position_base.header.stamp = rospy.Time.now()
	position_publish.publish(position_base)
	final_position.x = position_base.point.x
	final_position.y = position_base.point.y
	final_position.z = position_base.point.z + 0.1
	
#----------------use transform------
        '''(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))
	transform = listener.fromTranslationRotation(trans, rot)  #update transform
	position_new = np.dot(transform, np.array([position.point.x, position.point.y, position.point.z, 1]))
	final_position.x = position_new[0]
        final_position.y = position_new[1]
        final_position.z = position_new[2] + 0.1
	print('new position transform',position_new)'''
#-----------------------------------

#----------------use tooltip---------
	
	'''roll = actual_orientation.roll
	pitch = actual_orientation.pitch
	yaw = actual_orientation.yaw
	cartesian_transform = transformations.euler_matrix(roll, pitch, yaw, axes ='sxyz')
	cartesian_transform[0,3] = actual_position.x
	cartesian_transform[1,3] = actual_position.y
	cartesian_transform[2,3] = actual_position.z   #update transform

	position_new = np.dot(T, point[:,-1])
	position_new = np.dot(cartesian_transform, position_new)
	print('new position tooltip', position_new)'''

#--------------------------------------------------------
	
	command_old = command
	command = PoseRPY(0, final_position, initial_orientation)
	'''if command != command_old  #too many commands sent
		if abs(actual_position.x - final_position.x) < 0.15 and abs(actual_position.y - final_position.y) < 0.1:
	    		publisher.publish(command)'''
	print('actual position',actual_position)
	rate.sleep() #added to respect the 10 hz rate
        

if __name__ == '__main__':
    main()
