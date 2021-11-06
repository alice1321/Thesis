import numpy as np
import cv2 as cv
import glob


################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (13,9)
frameSize = (1080,960)  #modify to full size image

##!!!keep into account the size of the images that are processed by the neural network
## do the calibration for that size, rectify the images and make the prediction on the rectified ones

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * 20  #size of the square on the chessboard in mm -- results are going to be in mm for translations
print(objp)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.


imagesLeft = sorted(glob.glob('images/stereoLeft/*.png'))
imagesRight = sorted(glob.glob('images/stereoRight/*.png'))

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
        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv.imshow('img right', imgR)
        cv.waitKey(1000)


cv.destroyAllWindows()




############## CALIBRATION #######################################################

retL, cameraMatrixL, distL, rvecsL, tvecsL, stdDeviationsIntrinsicsL, stdDeviationsExtrinsicsL, RMSL = cv.calibrateCameraExtended(objpoints, imgpointsL, frameSize, None, None )
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))  #try also keeping principal poin in center

retR, cameraMatrixR, distR, rvecsR, tvecsR, stdDeviationsIntrinsicsR, stdDeviationsExtrinsicsR, RMSR = cv.calibrateCameraExtended(objpoints, imgpointsR, frameSize, None, None)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))  #set center principal point flag!!!



########## Stereo Vision Calibration #############################################

#flags = 0
#flags = cv.CALIB_USE_INTRINSIC_GUESS
flags = cv.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated(good)
# Hence intrinsic parameters are the same 

criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 300, 0.001)

# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
#retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix, perViewErrors = cv.stereoCalibrateExtended(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)
#retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix, perViewErrors = cv.stereoCalibrateExtended(objpoints, imgpointsR, imgpointsL, newCameraMatrixR, distR, newCameraMatrixL, distL, grayL.shape[::-1], criteria_stereo, flags)

saved_param = cv.FileStorage('CameraCalib2_960x1080pp.xml', cv.FILE_STORAGE_WRITE)
saved_param.write('IntrinsciParamL', newCameraMatrixL)
saved_param.write('IntrinsicParamR', newCameraMatrixR)
saved_param.write('EssentialMatrix', essentialMatrix)
saved_param.write('FundametalMatrix', fundamentalMatrix)
saved_param.write('Error', perViewErrors)
saved_param.write('Rotation', rot)
saved_param.write('Translation', trans) #camera L wrt R
saved_param.write('distL', distL)
saved_param.write('distR',distR)

saved_param.release()

########## Stereo Rectification #################################################

rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

projective_matrixes = cv.FileStorage('projective_mat2_960x1080.xml', cv.FILE_STORAGE_WRITE)
projective_matrixes.write('ProjectiveMatL', projMatrixL)
projective_matrixes.write('ProjectiveMatR', projMatrixR)

projective_matrixes.release()

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

print("Saving parameters!")
cv_file = cv.FileStorage('stereoMap2_960x1080.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])

cv_file.release()
#the stereomap is used to rectify and undistort the images; afterwards we can compute the depth'''
