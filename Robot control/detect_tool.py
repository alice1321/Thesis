#! /usr/bin/env python3

# This node takes images from topics coming from the stereo camera mounted on the EE of the Kuka arm,
# identifies a laparoscopic tool by means of YOLOv3 object detection NN (trained on a custom dataset),
# and computes the position of the instrument wrt the camera using the disparity between the two images. 
# The computed position id than sent to the 'send_position' node.

#---TO DO---
# use rospy.time to set the update of the Kalman at the correct time  
# plot coordinates and filtered coordinates in time to see the delay x-time, y-time
# add the camera offset wrt the link7 in the transform T
# write the code in classes

import rospy
from rospy.names import initialize_mappings
import tf2_ros
import tf2_msgs.msg
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import random
import cv2
import numpy as np
import argparse
import time
import torch
import matplotlib.pyplot as plt
import torchvision.transforms as transforms
from pytorchyolo.models import load_model
from pytorchyolo.utils.utils import load_classes, rescale_boxes, non_max_suppression, to_cpu, print_environment_info
from pytorchyolo.utils.transforms import DEFAULT_TRANSFORMS, Resize
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from dataclasses import dataclass
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, PoseStamped, PointStamped
from tf import TransformListener, listener, transformations



global frameL
global frameR
bridge = CvBridge()
global coordinates 
global filtered_coordinates
global time_stamp
time_stamp = []
filtered_coordinates = []
coordinates = [] #save the coordinates of the tip 
T = np.array(([-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1])) #camera rotation wrt the tooltip

#----------------------------------------Input arguments------------------------------

parser = argparse.ArgumentParser(description = "Object detection")
parser.add_argument("-m", "--model", type=str, default="config/yolov3-custom.cfg", help="Path to model definition file (.cfg)")
parser.add_argument("-w", "--weights", type=str, default="weights/yolov3_ckpt_360.pth", help="Path to weights or checkpoint file")  #20 best
parser.add_argument("-c", "--classes", type=str, default="data/custom/classes.names", help="Path to classes label file (.names)")
parser.add_argument("--img_size", type=int, default=416, help="Size of each image dimension for yolo")   
parser.add_argument("--conf_thres", type=float, default=0.1, help="Object confidence threshold")
parser.add_argument("--nms_thres", type=float, default=0.4, help="IOU threshold for non-maximum suppression")
args = parser.parse_args()

#Load the model and classes

torch.backends.cudnn.benchmark = True
classes = load_classes(args.classes)  
model = load_model(args.model, args.weights)
model.eval()

img_size = args.img_size
conf_thresh = args.conf_thres
nms_thresh = args.nms_thres

#Load parameters

parameters_file = cv2.FileStorage()
#parameters_file.open("parameters4pp.xml", cv2.FileStorage_READ)
parameters_file.open("CameraCalib1.xml", cv2.FileStorage_READ)
intrinsic_paramL = parameters_file.getNode("IntrinsciParamL").mat()
intrinsic_paramR = parameters_file.getNode("IntrinsicParamR").mat()
rotL = parameters_file.getNode("Rotation").mat()
translL = parameters_file.getNode("Translation").mat() 
distL = parameters_file.getNode("distL").mat()
distR = parameters_file.getNode("distR").mat()
parameters_file.release()
translL = -translL

#projection matrices A

RTR = cv2.hconcat([np.eye(3), np.zeros((3,1))])
PR = np.dot(intrinsic_paramR,RTR)

RTL = cv2.hconcat([rotL, translL])
PL = np.dot(intrinsic_paramL, RTL)


focal_lenght = [intrinsic_paramL[0,0], intrinsic_paramL[1,1]]
principal_point = [intrinsic_paramL[0,2], intrinsic_paramL[1,2]]
lenses_spacing = 0.035 #35mm
sensor_size = 0.00619512195 #6,19mm i.e. 1/4.1 inch
max_f = 0.00376 #low magnification 
min_f = 0.0188 #high magnification
#focal_lenght_mm = focal_lenght*sensor_size/img_width

'''proj_matrix = cv2.FileStorage()
proj_matrix.open('projective_mat_416x416.xml', cv2.FileStorage_READ)
proj_matL = proj_matrix.getNode('ProjectiveMatL').mat()
proj_matR = proj_matrix.getNode('ProjectiveMatR').mat()
proj_matrix.release()'''

#-----------------------Filter-------------------------------
delta_t = 0.35
filter = KalmanFilter(dim_x = 4, dim_z= 2)
filter.X = np.array([0., 0., 0., 0.]) #initial state
filter.F = np.array([[1., delta_t, 0., 0.], [0., 1., 0., 0.], [0., 0., 1., delta_t], [0., 0., 0., 1.]])
filter.H = np.array([[1., 0., 0., 0.],[0., 0., 1., 0.]])
filter.P *= 400 #variance squared -- 20 cm of estimation error
filter.R = np.array([[9.,0.],[0.,9.]])  #3cm of measurement error
filter.Q = Q_discrete_white_noise(dim=2, dt= 0.35, var = 0.25,  block_size = 2) #dim=2 constant velocity
#--------------------------------------------------------------------


#------------------------callbacks---------------------------------------

def callback_single_image(ros_data):
	global frame
	np_arr = np.frombuffer(ros_data.data, np.uint8)
	frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	#cv2.imshow('frame',frame)
	#cv2.waitKey(50)


#--------------------------------------------------------------------------------------
#---------------------------functions----------------------------------------------

def detect_image(model, image, img_size=416, conf_thres=0.1, nms_thres=0.4):
	
	input_img = transforms.Compose([DEFAULT_TRANSFORMS, Resize(img_size)])((image,np.zeros((1,5))))[0].unsqueeze(0)
	#unsqueeze transforms the new image in a column i.e. dimension 0 becomes unitary
	if torch.cuda.is_available():
		 input_img = input_img.to("cuda")

	#Detection: use torch.no_grad to deactivate the autograd engine. This is done 
	#to perform inference without Gradient Calculation.
	with torch.no_grad():
		global start
		start = time.time()
		detections = model(input_img)
		detections = non_max_suppression(detections, conf_thres, nms_thres)
		#rescale the boxes at the original image size
		detections = rescale_boxes(detections[0], img_size, image.shape[:2])
		print(detections)
		detectionsL = detections[detections[:,0] < 960]
		detectionsR = detections[detections[:,0] > 930]
		global stop
		print('detectionL',detectionsL)
		print('detectionR',detectionsR)
		stop = time.time()

	return to_cpu(detectionsL).numpy(), to_cpu(detectionsR).numpy()

def show_detection(image, detectionsR, detectionsL, name):

	n_cls_preds = 1
	colors = np.random.uniform(0, 255, size=(1, 3))
	font = cv2.FONT_HERSHEY_DUPLEX
	color = (0, 0, 0) 

	for x1, y1, x2, y2, conf, class_pred in detectionsL:
		if conf == max(detectionsL[:,4]) and conf>0.3:   #avoid miss detections 
			x_center_L = (x1 + x2)/2   #positions to be sent to topic
			y_center_L = (y1 + y2)/2
			cv2.rectangle(image, (int(x1),int(y1)), (int(x2),int(y2)), 170, 2)
			cv2.putText(image, 'Confidence Left: ' + str(conf),(x1, y1),font, 0.5, color, 1)
			break
		#cv2.putText(image, 'Confidence Left: ' + str(conf),(0, 354),font, 0.5, color, 1)

	for x1, y1, x2, y2, conf, class_pred in detectionsR:
		if conf == max(detectionsR[:,4]) and conf>0.3: 
			x_center_R = (x1 + x2)/2 - 960
			y_center_R = (y1 + y2)/2
			cv2.rectangle(image, (int(x1),int(y1)), (int(x2),int(y2)), 170, 2)
			cv2.putText(image, 'Confidence Right: ' + str(conf),(x1, y1),font, 0.5, color, 1)
			break
			#cv2.putText(image, 'Confidence Right: ' + str(conf),(0, 368),font, 0.5, color, 1)


	#inference = float("{0:.4f}".format(stop-start))
	#fps = int(1/inference)
	#cv2.putText(image, 'fps: ' + str(fps), (0,410), font, 0.5, color, 1)
	#cv2.putText(image,'Inference time: ' + str(inference) + 's' , (0,396), font, 0.5, color,1)
	cv2.imshow(name, image)
	key = cv2.waitKey(1)
	return x_center_R, y_center_R, x_center_L, y_center_L, image
#-------------------------------------------------------------------------

#--------------------------------offline prediction on a video-------------------

'''	#Take images from video 
	video = cv2.VideoCapture("00013.MTS")
	video_length = int(video.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
	print ("Number of frames: ", video_length)
	count = 0

	while video.isOpened():
		ret, frame = video.read()
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		if not ret:
			continue
		count = count + 1
		#cv2.imshow('image',frame)
		#cv2.waitKey(50)
		detections = detect_image(model, frame)
		if detections.any():
			show_detection(frame, detections)
		if (count > (video_length-1)):
			video.release()'''

#------------------------------------------------------------------------------------------

def main():
	global frameL
	global frameR
	global frame
	global coordinates
	global filtered_coordinates
	global time_stamp
	global pose
	point_stamp = PointStamped()
#--------------------------------------------------------------------

	rospy.init_node('video_subscriber', anonymous = True)
	publisher = rospy.Publisher('tool_pose', PointStamped, queue_size=10)
	image_publisher = rospy.Publisher('processed_images', Image, queue_size=1)
	unfiltered_publisher = rospy.Publisher('unfiltered_local', Vector3, queue_size=1)
	tf_broadcaster = rospy.Publisher('/tf',tf2_msgs.msg.TFMessage, queue_size=1)
	rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, callback_single_image)
	rate = rospy.Rate(10)
	time.sleep(1)

#-----------publish initial pose of the robot as first command-----------

	initial_position = Vector3(0.253434185055, 0.2045420407383, 0.408027999353)  #simulation
	roll_sim = 3.10409946183
	pitch_sim = 0.0220493542626
	yaw_sim = 2.70671490846
	#add for real robot too
	#initial_orientation = transformations.quaternion_from_euler(roll_sim, pitch_sim, yaw_sim)
	#pose = Pose(initial_position, initial_orientation)
	#publisher.publish(initial_position)



#------------publish camera tf-------------------------------
	'''camera_transform = geometry_msgs.msg.TransformStamped()
	camera_transform.header.frame_id = 'lwr_7_link' #to be completed 
	camera_transform.header.stamp = rospy.Time.now()
	camera_transform.child_frame_id = 'camera'
	camera_transform.transform.translation.x = 0
	camera_transform.transform.translation.y = 0
	camera_transform.transform.translation.z = 0
	camera_transform.transform.rotation.x = initial_orientation[0]
	camera_transform.transform.rotation.y = initial_orientation[1]
	camera_transform.transform.rotation.z = initial_orientation[2]
	camera_transform.transform.rotation.w = initial_orientation[3]'''
#-------------------------------------------------------------------	

	while not rospy.is_shutdown():
		start = time.time()
		detection_time = rospy.Time.now()
		detectionL, detectionR = detect_image(model, frame)
		stop = time.time()
		print(stop-start)
		#time_stamp.append(rospy.Time.now())
		#camera_transform.header.stamp = rospy.Time.now()
		#tf_cam = tf2_msgs.msg.TFMessage([camera_transform])
		#tf_broadcaster.publish(tf_cam)

		#Cartesian coordinates:  (computed wrt the right camera)
		if detectionL.any() and detectionR.any():

			point_stamp.header.stamp = detection_time
			xR, yR, xL, yL, img_mod = show_detection(frame, detectionR, detectionL,'Image')
			centerL = np.array([xL,yL], dtype=np.float64)
			centerR = np.array([xR,yR], dtype=np.float64)
			
			image_publisher.publish(bridge.cv2_to_imgmsg(img_mod, 'bgr8'))

			global coordinates  #are necessary?
			global filtered_coordinates
			undistort_L = cv2.undistortPoints(centerL.transpose(), intrinsic_paramL, distL)
			undistort_R = cv2.undistortPoints(centerR.transpose(), intrinsic_paramR, distR)

			#position = cv2.triangulatePoints(PL, PR, centerL,	centerR)  #alternativ:use intrinsics in triangulate
			position = cv2.triangulatePoints(RTR, RTL, undistort_R, undistort_L) #PR,PL only identity and extrinsic
			homog_points = position.transpose()
			euclid_points = cv2.convertPointsFromHomogeneous(homog_points)
			xy_position = euclid_points[0,0,0:2]/10
			print(euclid_points)
			z = 0.5

			position_unfiltered = transformations.translation_matrix((xy_position[0]/100, xy_position[1]/100, z))  #m
			rotate_position_unfiltered = np.dot(T, position_unfiltered[:,-1])
			rotate_position_unfiltered = Vector3(rotate_position_unfiltered[0], rotate_position_unfiltered[1], rotate_position_unfiltered[2])
			

			filter.predict()
			filter.update(xy_position)
			xy_filtered = filter.x/100 #from cm to m
			
			position_to_topic = Vector3(xy_filtered[0,0],xy_filtered[2,0],z)
			point = transformations.translation_matrix((position_to_topic.x, position_to_topic.y, position_to_topic.z))
			rotate_point = np.dot(T, point[:,-1])
			
			point_stamp.header.frame_id = 'lwr_7_link'
			point_stamp.point.x = rotate_point[0]
			point_stamp.point.y = rotate_point[1]
			point_stamp.point.z = rotate_point[2]
			#stop = time.time()
			#print(stop-start)
			
			if abs(position_to_topic.x) < 0.2 and abs(position_to_topic.y) < 0.2:
				publisher.publish(point_stamp)
				unfiltered_publisher.publish(rotate_position_unfiltered)
			
			print(rotate_point)
			filtered_coordinates.append(xy_filtered)
			coordinates.append(euclid_points[0,0,:]/10)
			#print(filter.K)
			#print('Position:', euclid_points/10)  #expressed in cm
			#print('Filtered position:', filtered_coordinates)

		else:
			cv2.imshow('Image', frame)
			#cv2.imshow('ImageR', frameR)
			key = cv2.waitKey(1)
			image_publisher.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
	
		#rate.sleep()
	#np.save('test', coordinates)
	#np.save('filtered_test', filtered_coordinates)
	#np.save('time_test', time_stamp)


if __name__ == '__main__':
	main()




