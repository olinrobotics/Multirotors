import numpy as np
import cv2

cap = cv2.VideoCapture('Videos/normal_lens_hallway.mp4')
cap2 = cv2.VideoCapture('Videos/adjusted_lens_hallway.mp4')

npz_calib_file = np.load('calibration_data.npz')

distCoeff = npz_calib_file['distCoeff']
intrinsic_matrix = npz_calib_file['intrinsic_matrix']

npz_calib_file.close()

while(cap.isOpened() and cap2.isOpened()):
	ret, frame = cap.read()
	ret2, frame2 = cap2.read()
	# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	undistort = cv2.undistort(frame, intrinsic_matrix, distCoeff, None)
	cv2.imshow('virtual',undistort)
	cv2.imshow('physical',frame2)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cap2.release()
cv2.destroyAllWindows()