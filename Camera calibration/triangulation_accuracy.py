#compute the calibration accuracy on the chessboard pattern

import rospy
import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2 as cv
import triangulation as tri
import calibration


max_f = 3.76 #low magnification in mm
lenses_spacing = 3.5 # cm
alpha = 117 #FOV in degrees 

parameters_file = cv.FileStorage()
parameters_file.open("parameters4pp.xml", cv.FileStorage_READ)
intrinsic_paramL = parameters_file.getNode("IntrinsciParamL").mat()
intrinsic_paramR = parameters_file.getNode("IntrinsicParamR").mat()
rotL = parameters_file.getNode("Rotation").mat()
translL = parameters_file.getNode("Translation").mat() 
distL = parameters_file.getNode("distL").mat()
distR = parameters_file.getNode("distR").mat()
error_calib = parameters_file.getNode("Error").mat()
parameters_file.release()
translL = -translL   #t in the projectiv matrix represents the position of L wrt R in L reference frame

calib_error_x = sum(error_calib[:,0])/np.shape(error_calib)[0]
calib_error_y = sum(error_calib[:,1])/np.shape(error_calib)[0]

proj_matrix = cv.FileStorage()
proj_matrix.open('projective_mat_416x416.xml', cv.FileStorage_READ)
proj_matL = proj_matrix.getNode('ProjectiveMatL').mat()
proj_matR = proj_matrix.getNode('ProjectiveMatR').mat()
proj_matrix.release()

cv_file = cv.FileStorage()
cv_file.open('stereoMapProva4.xml', cv.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()
cv_file.release()


focal_lenght = [intrinsic_paramR[0,0], intrinsic_paramR[1,1]]
principal_pointL = [intrinsic_paramL[0,2], intrinsic_paramL[1,2]]
principal_pointR = [intrinsic_paramR[0,2], intrinsic_paramR[1,2]]
lenses_spacing = 0.035 #35mm

RTR = cv.hconcat([np.eye(3), np.zeros((3,1))])
PR = intrinsic_paramR @ RTR

RTL = cv.hconcat([rotL, translL])
PL = intrinsic_paramL @ RTL

'''#-----------------------------------------------------------
#distorted 416*416 #ImgL1/R1
#points_L =np.array([[61, 98], [98, 93], [136, 88], [176, 82], [57, 164]], dtype=np.float64)
#points_R =np.array([[43, 99], [79, 93], [117, 88], [155, 83], [40, 164]], dtype=np.float64)


#distorted 416*416 #ImgL8/R8
points_L =np.array([[127, 92], [158, 90], [190, 89], [221, 88], [125, 144]], dtype=np.float64)
points_R =np.array([[114, 91], [146, 89], [177, 88], [208, 87], [112, 144]], dtype=np.float64)

#980*1080
#points_R =np.array([[163, 104], [255, 100], [348, 97], [157, 285]], dtype=np.float64)
#points_L =np.array([[206, 100], [300, 96], [396, 93], [204, 282]], dtype=np.float64)
#points_L =np.array([[61], [98]], dtype=np.float64)
#points_R =np.array([[43], [99]], dtype=np.float64)

center_point_right = [314, 44]
center_point_left = [380, 44]


#frameL = cv.imread('calib416x416/stereoLeft/imageL1.png')
#frameR = cv.imread('calib416x416/stereoRight/imageR1.png')

frameL = cv.imread('calib416x416/stereoLeft/imageL8.png')
frameR = cv.imread('calib416x416/stereoRight/imageR8.png')

#frame_right, frame_left = calibration.undistortRectify(frameR, frameL)
#cv.imwrite('images/stereoLeft/imageLRect' + '.png', frame_left)
#cv.imwrite('images/stereoRight/imageRRect' + '.png', frame_right)

plt.imshow(frameL)
plt.scatter(points_L[:,0], points_L[:,1])
plt.show()
 
plt.imshow(frameR)
plt.scatter(points_R[:,0], points_R[:,1])
plt.show()

#depth = tri.find_depth(center_point_right, center_point_left, frameR, frameL, lenses_spacing, max_f, alpha)
#print('depth:', depth)
#------------------------------------------------------------------------------------'''

#-------------------------------chessboard-------------------------------------------

chessboardSize = (13,9)
frameSize = (416,416)  #modify to full size image

##!!!keep into account the size of the images that are processed by the neural network
## do the calibration for that size, rectify the images and make the prediction on the rectified ones

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * 20  #size of the square on the chessboard in mm -- results are going to be in mm for translations
#print(objp)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.


imagesLeft = sorted(glob.glob('test_images/stereoLeft/*.png'))
imagesRight = sorted(glob.glob('test_images/stereoRight/*.png'))

for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv.imread(imgLeft)
    imgR = cv.imread(imgRight)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if retL and retR == True:

        objpoints.append(objp)

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

     # Draw and display the corners
        cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv.imshow('img left', imgL)
        cv.imwrite('chessboardL.png', imgL)
        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv.imshow('img right', imgR)
        cv.waitKey(1000)

#print(np.shape(cornersL))
cornersL = cornersL[:,0,:]
cornersR = cornersR[:,0,:]
print(cornersL)


#cv.destroyAllWindows()

#-------------------------------------------------------------------------------



undistort_L = cv.undistortPoints(cornersL.transpose(), intrinsic_paramL, distL)
undistort_R = cv.undistortPoints(cornersR.transpose(), intrinsic_paramR, distR)

position = cv.triangulatePoints(RTR, RTL, undistort_R, undistort_L) #PR,PL only identity and extrinsic
#position = cv.triangulatePoints(proj_matL, proj_matR, points_L.transpose(), points_R.transpose())
homog_points = position.transpose()
euclid_points = cv.convertPointsFromHomogeneous(homog_points)

'''#Manually
x = []
y = []
z = []

for i in range(5):
    xL = points_L[i,0] - principal_pointL[0]
    yL = points_L[i,1] - principal_pointL[1]
    xR = points_R[i,0] - principal_pointR[0]
    yR = points_R[i,1] - principal_pointR[1]
    Z = lenses_spacing*focal_lenght[0]/(abs(xL)+abs(xR))
    z.append(Z)
    X = Z*xL/focal_lenght[0]
    x.append(X)
    Y = Z*yL/focal_lenght[1]
    y.append(Y)'''


print('position:', euclid_points)


x_acc = 0
y_acc = 0
err_x = 0
err_y = 0

#provare a fare la media di piu immagini 

for i in range(chessboardSize[1]):
    for j in range(chessboardSize[0]-1):
        x_acc = abs(euclid_points[(j+1)+i*13,0,0] - euclid_points[j+i*13,0,0])
        err_x += abs(x_acc-20)

for i in range(np.shape(euclid_points)[0] - 13):
    y_acc = abs(euclid_points[(i+13),0,1] - euclid_points[i,0,1])
    err_y += abs(y_acc -20)


print("i", i)

print("y",err_y/104)
print("x",err_x/108)
print("Calib_error_x", calib_error_x)
print("Calib_error_y", calib_error_y)

'''im1 = cv.imread("calib416x416/stereoLeft/imageL0.png")
frame_right = cv.remap(im1, stereoMapR_x, stereoMapR_y, cv.INTER_LANCZOS4, cv.BORDER_CONSTANT, 0)


cv.imshow("frame right", frame_right) 
cv.waitKey(1000)
cv.imwrite("undistorted.png", frame_right)'''

