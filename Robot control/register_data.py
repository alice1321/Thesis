#! /usr/bin/env python3

#registeres all variables: time_sec represents the time of the command;

import rospy
import csv
from geometry_msgs.msg import Vector3, PointStamped
from rospy.names import global_name
from lwr_controllers.msg import PoseRPY

def get_position_unfiltered(ros_data):
    global unfiltered_position
    unfiltered_position = ros_data

def get_tool_pose(tool):
    global tool_pose
    tool_pose = tool

def get_actual_position(position):
    global actual_position
    actual_position = position

def get_global_position(glob):
    global global_position 
    global_position = glob

def get_time_command(comm):
    global position
    position = comm

filename = rospy.get_param("/file_name")  #use rosparam set in terminal: rosparam set file_name 'file.csv'

def main():
    global tool_pose
    global actual_position
    global global_position
    rospy.init_node('data_recording', anonymous= True)
    rospy.Subscriber('/tool_pose', PointStamped, get_tool_pose)
    rospy.Subscriber('/cartesian_position/pose/tooltip', PoseRPY, get_actual_position)
    #rospy.Subscriber('/lwr/one_task_inverse_kinematics/command', PoseRPY, get_global_position) #use it during real aquisitions when the command is going to be published
    rospy.Subscriber('/position', PointStamped, get_time_command)  #use it for tests before publishing the command
    rospy.Subscriber('/unfiltered_local', Vector3, get_position_unfiltered)
    rate = rospy.Rate(10)
    file = open(filename, 'w')
    fieldnames = ['time_sec','time_nsec','detection_time_sec','detection_time_nsec', 'local_position_x','local_position_y', 'local_position_z','global_position_x','global_position_y', 'global_position_z', 'actual_position_x','actual_position_y', 'actual_position_z','local_position_unfiltered_x', 'local_position_unfiltered_y', 'local_position_unfiltered_z']
    writer = csv.DictWriter(file, fieldnames=fieldnames, dialect='excel')
    writer.writeheader()
    rospy.sleep(1)


    while not rospy.is_shutdown():
        global tool_pose
        global actual_position
        global global_position
        time = rospy.Time.now()
        writer.writerow({'time_sec': position.header.stamp.secs,'time_nsec': position.header.stamp.nsecs,'detection_time_sec': tool_pose.header.stamp.secs, 'detection_time_nsec': tool_pose.header.stamp.nsecs, 'local_position_x': tool_pose.point.x, 'local_position_y': tool_pose.point.y, 'local_position_z': tool_pose.point.z, 'global_position_x': position.point.x ,'global_position_y': position.point.y, 'global_position_z':position.point.z, 'actual_position_x': actual_position.position.x, 'actual_position_y': actual_position.position.y, 'actual_position_z': actual_position.position.z, 'local_position_unfiltered_x': unfiltered_position.x, 'local_position_unfiltered_y': unfiltered_position.y, 'local_position_unfiltered_z': unfiltered_position.z})
        rate.sleep()

    file.close()

if __name__ == '__main__':
	main()



