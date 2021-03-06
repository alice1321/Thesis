#! /usr/bin/env python3

# This node takes in input the position of the instrument in the local RF of the camera, expresses it in the global RF
# and than send it to the one task inverse kinematics controller.

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

#Callbacks
def get_desired_position(ros_data):
    global position
    position = ros_data

def get_actual_position(ros_data):
    global actual_position 
    global actual_orientation
    actual_position = ros_data.position
    actual_orientation = ros_data.orientation

	
#Main
def main():
    rospy.init_node('send_pose', anonymous=True)
    rospy.Subscriber('/tool_pose', PointStamped, get_desired_position)
    rospy.Subscriber('/cartesian_position/pose/tooltip', PoseRPY, get_actual_position)
    publisher = rospy.Publisher('/lwr/one_task_inverse_kinematics/command', PoseRPY, queue_size=10)
    position_publish = rospy.Publisher('position', PointStamped, queue_size=10)
    rate = rospy.Rate(10)
    #rospy.sleep(1)

    command = PoseRPY(0, initial_position, initial_orientation) #set initial position
    publisher.publish(command)
    time.sleep(2)

    now = rospy.Time.now() + rospy.Duration(0.9)
    listener = TransformListener()
    listener.waitForTransform('lwr_base_link', 'lwr_7_link', now, rospy.Duration(1.0))
    final_position = Vector3()
    #final_position.z = 0.18561169506  #keep a fixed z position
    #(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))
    #transform = listener.fromTranslationRotation(trans, rot)

    while not rospy.is_shutdown():

#--------------use listener transform point----------
	wait = position.header.stamp  #wait for the transform at the detection time to become available
	listener.waitForTransform('lwr_base_link', 'lwr_7_link', wait, rospy.Duration(2.0))
	position_base = listener.transformPoint('lwr_base_link', position)
	position_base.header.stamp = rospy.Time.now()  #used to record global position in time
	position_publish.publish(position_base)
	final_position.x = position_base.point.x
	final_position.y = position_base.point.y
	final_position.z = position_base.point.z + 0.2  #keep the camera 20 cm far from the instrument
	
#----------------use transform------------------------
        '''(trans, rot) = listener.lookupTransform('lwr_base_link', 'lwr_7_link', rospy.Time(0))  #gives the last available transform (not synchronised with the detection)
	transform = listener.fromTranslationRotation(trans, rot)  #update transform
	position_new = np.dot(transform, np.array([position.point.x, position.point.y, position.point.z, 1]))
	final_position.x = position_new[0]
        final_position.y = position_new[1]
        final_position.z = position_new[2] + 0.1
	print('new position transform',position_new)'''
#------------------------------------------------------

	if abs(actual_position.x - final_position.x) < 0.2 and abs(actual_position.y - final_position.y) < 0.2:
	    	publisher.publish(command)
	#print('actual position',actual_position)
	rate.sleep() #added to respect the 10 hz rate
        

if __name__ == '__main__':
    main()
