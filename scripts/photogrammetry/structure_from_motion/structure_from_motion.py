#structure_from_motion.py
#Victoria Preston, credits to Chris Rillhan and https://www.packtpub.com/sites/default/files/9781849517829_Chapter_04.pdf
#Python 2.7.10, OpenCV 3.0.0 and Numpy 1.9.2


#This program takes stills from a video, then reconstructs the scene using specialized structure from motion techniques 

import cv2, sys
import numpy as np
from matplotlib import pyplot as plt
import lktracker
import homography
import sfm
import sift

#Import Information
filename = 'test2.avi'
#Input number of stills to collect to build scene
n_stills = 2
#Image resolution
image_size = (1920, 1080)

#The ImageCollect function can be used manually, or set-up to auto collect the specified number of images from a filmstream when the timing on the stream is given. This preliminary function requires a human to select the images to stitch together into a scene.

def ImageCollect(filename, n_stills, auto_collect = False):
    #Collect Calibration Images
    print('-----------------------------------------------------------------')
    print('Loading video...')

    #Load the file given to the function
    video = cv2.VideoCapture(filename)
    #Checks to see if a the video was properly imported
    status = video.isOpened()

    if status == True:
        
        #Collect metadata about the file.
        FPS = video.get(cv2.CAP_PROP_FPS)
        FrameDuration = 1/(FPS/1000)
        width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
        size = (int(width), int(height))
        total_frames = video.get(cv2.CAP_PROP_FRAME_COUNT)

        #Initializes the frame counter and collected_image counter
        current_frame = 0
        collected_images = 0

        #Video loop.  Press spacebar to collect images.  ESC terminates the function.
        if not auto_collect:
            while current_frame < total_frames:
                success, image = video.read()
                current_frame = video.get(cv2.CAP_PROP_POS_FRAMES)
                cv2.imshow('Video', image)
                k = cv2.waitKey(int(FrameDuration)) #You can change the playback speed here
                if collected_images == n_stills: 
                    break
                if k == 32:
                    collected_images += 1
                    cv2.imwrite('Motion_Image' + str(collected_images) + '.png', image)
                    print(str(collected_images) + ' images collected.')
                if k == 27:
                    break
    
        #Clean up
        video.release()
        cv2.destroyAllWindows()
    else:
        print('Error: Could not load video')
        sys.exit()

#The optical flow function uses this API from OpenCV to map key points and motion from one collected frame to the next

def dense_opt_flow(n_stills): #dense optical flow
    cap = cv2.VideoCapture("test2.avi")

    ret, frame1 = cap.read()
    prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(frame1)
    hsv[...,1] = 255

    while(1):
        ret, frame2 = cap.read()
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

        cv2.imshow('frame2',bgr)
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('opticalfb.png',frame2)
            cv2.imwrite('opticalhsv.png',bgr)
        prvs = next

    cap.release()
    cv2.destroyAllWindows()

def sparse_opt_flow(n_stills): #great for tracking some features
    cap = cv2.VideoCapture("test2.avi")

    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 7, blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create some random colors
    color = np.random.randint(0,255,(100,3))

    # Take first frame and find corners in it
    ret, old_frame = cap.read()
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)

    while(1):
        ret,frame = cap.read()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]

        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)

        cv2.imshow('frame',img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)

    cv2.destroyAllWindows()
    cap.release()

def book_opt_flow(n_stills):
    for i in range(2, n_stills + 1):
    
        #Loading images
        img1 = cv2.imread('Motion_Image' + str(i-1) + '.png')
        img2 = cv2.imread('Motion_Image' + str(i) + '.png')

        #Detect keypoints in the left and right images
        ffd = cv2.FastFeatureDetector_create()
        left_kpts = ffd.detect(img1, None)
        right_kpts = ffd.detect(img2,None)

        left_pts = cv2.KeyPoint_convert(left_kpts)
        #in the C++ code they establish an empty array here for right pts

        prevgray = cv2.cvtColor(img1,cv2.COLOR_RGB2GRAY)
        gray = cv2.cvtColor(img2,cv2.COLOR_RGB2GRAY)

        right_pts, vstatus, verror = cv2.calcOpticalFlowPyrLK(prevgray, gray, left_pts, None)

        right_points_to_find = np.array([])
        right_points_to_find_back_index = np.array([])
        right_features = np.array([])

        for i in range(0,len(vstatus)):
            if vstatus[i] and verror[i] < 12.0:
                right_points_to_find_back_index = np.append(right_points_to_find_back_index, i)
                right_points_to_find = np.append(right_points_to_find, right_pts[i])
            else:
                vstatus[i] = 0

        right_features = np.append(right_features, cv2.KeyPoint_convert(right_kpts))

        bf = cv2.BFMatcher()
        # print type(right_points_to_find[0]), type(right_features[0])
        matches = bf.knnMatch(right_points_to_find.astype('float32'),right_features.astype('float32'),k=2)

        nearest_neighbors = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                nearest_neighbors.append([m])

        img3 = cv2.drawMatchesKnn(img1,right_kpts,prevgray,left_kpts,nearest_neighbors,flags=2, outImg=None)

        plt.imshow(img3),plt.show(
)
def python_opt_flow(n_stills):
    innames = []
    for i in range(1, n_stills + 1):
        innames.append('Motion_Image' + str(i) + '.png')

    lkt = lktracker.LKTracker(innames)

    lkt.detect_points()
    lkt.draw()
    for i in range(len(innames)-1):
        lkt.track_points()
        lkt.draw()

    for im,ft in lkt.track():
        print 'tracking %d features' %len(ft)

    plt.figure()
    plt.imshow(im)
    for p in ft:
        plt.plot(p[0],p[1],'bo')
    for t in lkt.tracks:
        plt.plot([p[0] for p in t], [p[1] for p in t])
    plt.axis('off')
    plt.show()


#The matched key points function just uses strong features to map one image to the next
def matched_key_points(n_stills):
    for i in range(2, n_stills + 1):
    
        #Loading images
        img1 = cv2.imread('Motion_Image' + str(i-1) + '.png')
        img2 = cv2.imread('Motion_Image' + str(i) + '.png')

        kaze = cv2.KAZE_create() #note: opencv3 does not have SIFT or SURF

        kp1, des1 = kaze.detectAndCompute(img1,None)
        kp2, des2 = kaze.detectAndCompute(img2,None)

        bf = cv2.BFMatcher()
        # print type(des1)
        matches = bf.knnMatch(des1,des2,k=2)

        # print matches
        
        good = []
        for m,n in matches:
            if m.distance < 0.05*n.distance:
                good.append([m])

        # matches = sorted(matches, key = lambda x:x.distance)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,flags=2,outImg =None)

        plt.imshow(img3),plt.show()

        #FLANN might be something to look into in the future



print("Starting camera calibration....")
print("Step 1: Image Collection")
print("We will playback the calibration video.  Press the spacebar to save")
print("calibration images.")
print(" ")
print('We will collect ' + str(n_stills) + ' calibration images.')

ImageCollect(filename, n_stills)

print(' ')
print('All the calibration images are collected.')
print('------------------------------------------------------------------------')
print('Step 2: Calibration')
print('We will analyze the images take and calibrate the camera.')
print('Press the esc button to close the image windows as they appear.')
print(' ')

# matched_key_points(n_stills)
# book_opt_flow(n_stills)
# python_opt_flow(n_stills)

