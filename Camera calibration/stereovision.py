import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image


#Unidstorts and rectify the images coming from the 2 cameras
# Camera parameters to undistort and rectify images

cv_file = cv2.FileStorage()
cv_file.open('stereoMapProva4.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()


def callbackL(ros_data):

    global imgL
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    imgL = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #cv2.imshow('Img 1',imgL)
    #cv2.waitKey(1)

def callbackR(ros_data):

    global imgR
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    imgR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #cv2.imshow('Img 1',imgL)
    #cv2.waitKey(1)

rospy.Subscriber("/camera/rightImage/compressed", CompressedImage, callbackR)
rospy.Subscriber("/camera/leftImage/compressed", CompressedImage, callbackL)

def main():

    global imgL
    global imgR
    rospy.init_node('video_rectifier',anonymous=True)
    rate = rospy.Rate(10)  #10hz
    rospy.sleep(1)   #sleeeps for 1 second in order to have img published before the while

    while not rospy.is_shutdown():

        frame_right = cv2.remap(imgL, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        frame_left = cv2.remap(imgR, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        cv2.imshow("frame right", frame_right) 
        cv2.imshow("frame left", frame_left)

            # Hit "q" to close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()

