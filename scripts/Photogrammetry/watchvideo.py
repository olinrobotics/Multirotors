import numpy as np
import cv2

cap = cv2.VideoCapture('test2.avi')

npz_calib_file = np.load('calibration_data.npz')

distCoeff = npz_calib_file['distCoeff']
intrinsic_matrix = npz_calib_file['intrinsic_matrix']

npz_calib_file.close()

while(cap.isOpened()):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    undistort = cv2.undistort(frame, intrinsic_matrix, distCoeff, None)


    cv2.imshow('frame',undistort)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()