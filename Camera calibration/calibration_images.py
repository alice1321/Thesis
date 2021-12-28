import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
global num 
num = 0
global imgL
global imgR

##camera/compressed reduces to half the width

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

'''
    k = cv2.waitKey(5)

    #if k == 27:
    #    break
    if k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', imgL)
        #cv2.imwrite('images/stereoright/imageR' + str(num) + '.png', imgR)
        print("images saved!")
        num += 1
    cv2.imshow('Img 1',imgL)
    cv2.waitKey(1)
    #cv2.imshow('img', imgL)
    #cv2.waitKey(1)'''
   



def main():
    global num
    global imgL
    global imgR
    rospy.init_node('video_subscriber', anonymous = True)
    rate = rospy.Rate(10)  #10hz
    rospy.sleep(1)   #sleeeps for 1 second in order to have img published before the while
    while not rospy.is_shutdown():
        k = cv2.waitKey(5)
        #if k == 27:
            #break
        if k == ord('s'): # wait for 's' key to save and exit
            cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', imgL)
            cv2.imwrite('images/stereoRight/imageR' + str(num) + '.png', imgR)
            print("images saved!")
            num += 1
        cv2.imshow('Img 1',imgL)
        cv2.waitKey(1)
        cv2.imshow('Img 2', imgR)
        cv2.waitKey(1)
        rate.sleep()
        
    #rospy.spin()  #used when the node doesn't do anything outside it's callback
if __name__ == '__main__':
    main()


