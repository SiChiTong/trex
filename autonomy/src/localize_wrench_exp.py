#!/usr/bin/env python

""" idwrench.py - Version 1.0 2016-11-17

    This software chooses the correct wrench for the MBZIRC UGV
    challenge in an RGB image and outputs an estimate of its 3D location
    in space relative to the camera [x,y,z]
    Made by Jonathan Hodges
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
from scipy.cluster import vq
import scipy.stats

class idwrench():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('idwrench', anonymous=True)
        
        # Enable shutdown in rospy
        rospy.on_shutdown(self.shutdown)

        # Define parameters
        self.lim_type = 0
        self.n_wr = 6
        self.segment_median_value = 3
        self.segment_area_threshold = 10
        self.segment_kernel_sz = 8
        self.save_flag = True
        self.preview_flag = False           # Should we preview images
        self.live = False
        self.dir = '/home/ugv/'
        self.box_width = 1.0

        # Tweaking parameters - Need to adjust for different distances from camera
        self.number_of_wrenches = 1
        self.area_min_thresh = 100 # Minimum contour area to be a wrench
        self.max_circ_diam = 100 # Maximum circle diameter considered
        self.canny_param = [60, 20] # Canny edge detection thresholds
        self.p2crop = 3 # Portion of image to crop for circle detection

        """
        self.d_mu = 22.0 # Diameter of correct wrench in pixels
        self.d_sig = 3.8 # Uncertainty in diameter
        self.l_mu = 450 # Length of correct wrench in pixels 
        self.l_sig = 25.0 # Uncertainty in length
        self.a_mu = 17500 # Area of correct wrench in pixels
        self.a_sig = 2000 # Uncertainty in area
        self.vote_wt = [0.33,0.33,0.33] # Weight of each criteria in voting (d,l,a)
        """

        self.d_mu = 21.9 # Diameter of correct wrench in pixels
        self.d_sig = 4.0 # Uncertainty in diameter
        self.l_mu = 700 # Length of correct wrench in pixels 
        self.l_sig = 25.0 # Uncertainty in length
        self.a_mu = 32250 # Area of correct wrench in pixels
        self.a_sig = 2000 # Uncertainty in area
        self.vote_wt = [0.33,0.33,0.33] # Weight of each criteria in voting (d,l,a)

        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920/2
        self.camera_pix_v = 1080/2
        # Projection matrix is from Luan
        self.projectionMatrix = np.array([(530.125732, 0.000000, 318.753955, 0.000000), (0.000000, 532.849243, 231.863630, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)])

        # Counters
        self.ct = 0

	    # Establish publishers and subscribers
        self.bridge = CvBridge()
        self.id_pub = rospy.Publisher("/output/wrench_id_image",Image,queue_size = 1)
        self.binary_pub = rospy.Publisher("/output/wrench_binary_image",Image,queue_size = 1)
        self.prob_pub = rospy.Publisher("/output/wrench_prob_image",Image,queue_size = 1)
        self.probid_pub = rospy.Publisher("/output/wrench_prob_id_image",Image,queue_size = 1)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.image_output = rospy.Publisher("/output/keyevent_image",Image, queue_size=1)
        print("Waiting for an image.")
        self.img = None
        if self.live:
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        else:
            self.img = cv2.imread(self.dir+'/experiment_frame.jpg')

        while self.img is None:
            print("Waiting for an image.")
            rospy.sleep(0.5)
        self.process_img(np.copy(self.img))

    def process_img(self,img):
        if self.save_flag:
            cv2.imwrite(self.dir+'wrenchID_0_raw.png',img)
        self.preview_flag = False
        img_crop = self.detect_box_edge(img.copy())
        img_crop_invert = 255-img_crop.copy()

        # Determine ideal limits for brightness/contrast adjustment
        lims = self.stretchlim(img_crop_invert,self.lim_type)
        # Adjust the brightness/contrast of the RGB image based on limits
        img_adj = self.imadjust(img_crop_invert.copy(),lims)
        if self.preview_flag:
            print "img_adj"
            cv2.imshow('img',img_adj)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if self.save_flag:
            cv2.imwrite(self.dir+'wrenchID_2_adj.png',img_adj)
        # Remove Background from adjusted brightness/contrast image
        img_remove = self.back_ground_remove(img_adj.copy())
        img_remove = img_adj.copy()
        img_remove[:,0:600,:] = 255
        img_remove[:,650:,:] = 255
        img_remove[0:50,:,:] = 255
        if self.preview_flag:
            cv2.imshow('img',img_remove)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if self.save_flag:
            cv2.imwrite(self.dir+'wrenchID_3_bg.png',img_remove)
        # Convert the image to binary
        img_seg, img_gray = self.image_segmentation(img_remove.copy(),
            self.segment_median_value,self.segment_area_threshold,self.segment_kernel_sz)

        # Convert the image to grayscale
        #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Apply 2-D median filter to the grayscale image
        #img_gray = cv2.medianBlur(img_gray,11)
        if self.preview_flag:
            print "img_raw"
            cv2.imshow('img',img_gray)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        ret,binary = cv2.threshold(img_gray, 254, 255,cv2.THRESH_BINARY)


        # Convert the grayscale image to binary using Otsu's method
        """
        ret,binary = cv2.threshold(img_gray, 25, 255,cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8)
        binary_erode = cv2.dilate(binary, kernel, iterations=5)
        binary_dilate = cv2.erode(binary_erode, kernel, iterations=5)
        r,c = np.where(binary_dilate == 0)
        print(np.shape(r),r[0])
        print(np.min(r),np.max(r),np.min(c),np.max(c))
        """
        #box_width_px = np.max(c)-np.min(c)
        #print(box_width_px,self.camera_pix_h,self.camera_fov_h)
        #theta = float(box_width_px)/self.camera_pix_h*self.camera_fov_h
        self.xA = 1.1251-0.4240
        #self.xA = 0.5*self.box_width/np.tan(theta/2)

        if self.preview_flag:
            print "binary"
            cv2.imshow('img',binary)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        #img[:,0:600,:] = 0
        #img[0:600,:] = 0

        """
        # Crop image to remove gripper and edge of box
        img_crop = self.detect_box_edge(img.copy())
        img_crop_invert = 255-img_crop.copy()
        if self.save_flag:
            cv2.imwrite(self.dir+'wrenchID_1_crop.png',img_crop_invert)
        if self.preview_flag:
            print "img_crop"
            cv2.imshow('img',img_crop_invert)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        """


        #img_seg, img_gray = self.image_segmentation(img_crop_invert.copy(),
        #    self.segment_median_value,self.segment_area_threshold,self.segment_kernel_sz)
        sz = np.shape(img)
        if self.save_flag:
            cv2.imwrite(self.dir+'wrenchID_4_gray.png',img_gray)
            cv2.imwrite(self.dir+'wrenchID_5_seg.png',img_seg)
        # Crop image for circle detection
        params, contours = self.detect_geometry(img_seg,img_crop.copy())
        for i in range(0,len(contours)):
            (hi1,hi2,len2,wid2) = cv2.boundingRect(contours[i])
            img_gray_hou = np.copy(img_gray[0:sz[0]/1.2, 0:sz[1]])
            #img_gray_hou = np.copy(img_gray)#np.copy(img_gray[hi2:hi2+int(wid2/2),hi1:hi1+len2])
            circles, img_all_circles = self.detect_circle(img_gray_hou)
            params[i,5] = circles[0,2]
            params[i,6] = circles[0,0]#+hi1
            params[i,7] = circles[0,1]#+hi2
        if self.preview_flag:
            cv2.imshow('img',img_all_circles)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
                
        img_gray_hou = np.copy(img_gray[0:sz[0]/self.p2crop, 0:sz[1]]) 

        print params
        # Vote using the three parameters to determine correct wrench
        vote_result, wrench_ind = self.voting(params)
        # Visualize the probabilities and the best match
        img_kmeans, img_id, img_kmeans_id = self.visualize_probability(img_crop, vote_result,
            wrench_ind, params, contours)
        #cv2.imwrite('/home/jonathan/wrenchID_7_prob.png',img_kmeans)
        #cv2.imwrite('/home/jonathan/wrenchID_8_id.png',img_kmeans_id)
        # Publish results
        self.id_pub.publish(self.bridge.cv2_to_imgmsg(img_id, "bgr8"))
        self.prob_pub.publish(self.bridge.cv2_to_imgmsg(img_kmeans, "bgr8"))
        self.probid_pub.publish(self.bridge.cv2_to_imgmsg(img_kmeans_id, "bgr8"))
        self.binary_pub.publish(self.bridge.cv2_to_imgmsg(img_seg, "8UC1"))
        self.image_output.publish(self.bridge.cv2_to_imgmsg(img_kmeans_id, "bgr8"))
        try:
            ee_position = rospy.get_param('ee_position')
            wrench_position = rospy.get_param('wrench')
            xA = wrench_position[0]-ee_position[0]
        except:
            pass
        #print("img_kmeans:",img_kmeans)
        row = int(round(params[wrench_ind,7]))
        col = int(round(params[wrench_ind,6]))
        
        print("params:",params)
        print(row,col)
        self.wrench_id_px = np.array([row,col],dtype=np.float32)
        camera_y_mx = self.xA*np.arctan(self.camera_fov_h/2)
        camera_y_mn = -1*self.xA*np.arctan(self.camera_fov_h/2)
        camera_z_mx = self.xA*np.arctan(self.camera_fov_v/2)
        camera_z_mn = -1*self.xA*np.arctan(self.camera_fov_v/2)
        # Convert the wrench pixel location to m
        print "Camera limits: ", camera_y_mx, camera_y_mn
        print "Term1: ", (1-(col/self.camera_pix_h))
        print "Term2: ", (camera_y_mx-camera_y_mn)
        print "Term3: ", camera_y_mn
        wrenc_y = (1-(float(col)/self.camera_pix_h))*(camera_y_mx-camera_y_mn)+camera_y_mn
        wrenc_z = (1-(float(row)/self.camera_pix_v))*(camera_z_mx-camera_z_mn)+camera_z_mn
        self.wrench_id_m = np.array([self.xA, wrenc_y, wrenc_z],dtype=np.float32)
        print self.wrench_id_m
        # Create figure
        self.img = cv2.resize(self.img, (0,0), fx=2, fy=2) 
        cv2.circle(self.img,(2*col,2*row), 30, (0,255,0), 2, cv2.CV_AA)
        lcol = 20
        lrow = 120+20

        cv2.line(self.img,(self.camera_pix_h+20,self.camera_pix_v),(self.camera_pix_h-20,self.camera_pix_v),(255,0,0),5)
        cv2.line(self.img,(self.camera_pix_h,self.camera_pix_v-20),(self.camera_pix_h,self.camera_pix_v+20),(255,0,0),5)
        cv2.rectangle(self.img,(lcol+50,lrow-120),(lcol+550,lrow+120),(0,255,0),3)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.img,'x=%.4f m'%self.wrench_id_m[0],(lcol+60,lrow-70), font, 2,(255,255,255),2)
        cv2.putText(self.img,'y=%.4f m'%self.wrench_id_m[1],(lcol+60,lrow+10), font, 2,(255,255,255),2)
        cv2.putText(self.img,'z=%.4f m'%self.wrench_id_m[2],(lcol+60,lrow+90), font, 2,(255,255,255),2)
        cv2.imwrite(self.dir+'wrenchID_position.png',self.img)
        rospy.set_param('wrench_ID',[float(self.wrench_id_px[0]), 
            float(self.wrench_id_px[1])])
        rospy.set_param('wrench_ID_m',[float(self.wrench_id_m[0]), 
            float(self.wrench_id_m[1]), float(self.wrench_id_m[2])])
        rospy.set_param('smach_state','wrenchFound')
        if self.save_flag:
            try:
                cv2.imwrite(self.dir+'wrenchID_0_raw.png',img)
                cv2.imwrite(self.dir+'wrenchID_1_crop.png',img_crop)
                cv2.imwrite(self.dir+'wrenchID_2_adj.png',img_adj)
                cv2.imwrite(self.dir+'wrenchID_3_bg.png',img_remove)
                cv2.imwrite(self.dir+'wrenchID_4_gray.png',img_gray)
                cv2.imwrite(self.dir+'wrenchID_5_seg.png',img_seg)
                cv2.imwrite(self.dir+'wrenchID_6_allcircles.png',img_all_circles)
                cv2.imwrite(self.dir+'wrenchID_7_prob.png',img_kmeans)
                cv2.imwrite(self.dir+'wrenchID_8_id.png',img_kmeans_id)
            except:
                rospy.signal_shutdown('Ending node.')
        else:
            rospy.set_param('smach_state','wrenchNotFound')
            rospy.signal_shutdown('Ending node.')


    # This subroutine crops the RGB image to remove the gripper and the edge of
    # the box if it is visible.
    def detect_box_edge(self,img):
        sz = np.shape(img)
        #img = img[0:sz[0]*69/96,0:sz[1]]
        img = img[0:sz[0],0:sz[1]]
        sz = np.shape(img)
        img_edge = cv2.Canny(img,self.canny_param[0],self.canny_param[1])
        nnz = np.zeros([sz[1],1])
        kernel = np.ones((1,5), np.uint8)
        img_edge2 = cv2.dilate(img_edge, kernel, iterations=10)
        if self.preview_flag:
            cv2.imshow('img',img_edge2)
            cv2.waitKey(0)
        for i in range(0,sz[1]):
            tmp = np.count_nonzero(img_edge2[:,i])
            if tmp:
                nnz[i,0] = tmp
        ind = nnz[:,0].argsort()
        col = ind[sz[1]-1]
        mx = nnz[col,0]
        if mx >= 0.7*sz[0]:
            col = ind[sz[1]-1]
        else:
            col = sz[1]
        #col = 1300
        #row = 2100
        img = img[0:sz[0],0:col+20]
        #img = img[500:row,800:col]
        #img = img[0:row,0:col]
        return img

    # This subroutine computes the limits to be used in the brightness/contrast
    # adjustment routine. The flag denotes whether to fix the upper limit based
    # on the highest one percent of the data or not. 0 = Dynamic fix, 1 = Fixed
    # at an intensity of 255.
    def stretchlim(self,img,flag):
        # Determine size of image in pixels
        sz = np.shape(img)
        num_of_px = sz[0]*sz[1]
        # Determine one percent of total pixels (for use in image adjust code)
        one_perc = math.floor(num_of_px*0.01)
        lims = np.zeros((sz[2],2))
        # Compute lower/upper 1% threshold for each channel
        for i in range(0,sz[2]):
            hist,bins = np.histogram(img[:,:,i].ravel(),255,[0,255])
            val = 0; j = 0;
            while val < one_perc:
                val = val+hist[j]
                j = j +1
            lims[i,0] = j-80
            if flag == 0:
                val = 0; j = 0;
                while val < one_perc:
                    val = val+hist[254-j]
                    j = j + 1
                lims[i,1] = 254-j-80
            if flag == 1:
                lims[i,1] = 255
        return lims

    # This subroutine adjusts the intensities in each channel of the RGB image
    # using the limits supplied by stretchlim. Returns the adjusted image.
    def imadjust(self,img,lims):
        img2 = np.copy(img)
        sz = np.shape(img2)
        # Loop through each channel in the image
        for i in range(0,sz[2]):
            I2 = img2[:,:,i]
            # Set intensities above and below threshold to caps to prevent
            # overflow in 8bit numbers.
            I2[I2 > lims[i,1]] = lims[i,1]
            I2[I2 < lims[i,0]] = lims[i,0]
            # Scale the intensity in the channel
            img2[:,:,i] = (I2-lims[i,0])/(lims[i,1]-lims[i,0])*255
        return img2

    # This subroutine removes the background from the RGB image by increasing
    # the intensity in each channel of the image by (1/2) of the maximum value
    # within that channel. Returns the RGB image after background removal.
    def back_ground_remove(self,I):
        # Determine size of image in pixels
        sz = np.shape(I)
        # Initialize intensity array
        i = np.zeros((sz[2],1))
        # Initialize updated intensity matrix
        I3 = I.copy()
        # Loop through each channel of the image
        for j in range(0,sz[2]):
            # Caculate the intensity histogram of one channel
            hist,bins = np.histogram(I[:,:,j].ravel(),255,[0,255])
            I2 = I[:,:,j].copy()
            # Find the most common bin in the histogram
            i[j] = np.argmax(hist)
            # Fix overflow problems by setting values greater than the
            # modifier such that they will be maxed after addition
            I2[I2 > 255-i[j]*0.5] = 255-i[j]*0.5
            # Add the intensity modifier
            I2 = I2+0.5*i[j]
            # Update intensity matrix
            I3[:,:,j] = I2
        return I3

    # This subroutine converts the RGB image without background to binary.
    # It returns the binary and grayscale images.
    def image_segmentation(self,img1,median_value,area_threshold,kernel_sz):
        # Convert the image to grayscale
        img2 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        # Apply 2-D median filter to the grayscale image
        img2 = cv2.medianBlur(img2,median_value)
        # Convert the grayscale image to binary using Otsu's method
        ret,final = cv2.threshold(img2, 0, 255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Find contours within the binary image
        (cnts, _) = cv2.findContours(final.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize mask for image
        mask = np.ones(final.shape[:2], dtype="uint8") * 255
        # Loop through each detected contour
        for c in cnts:
            # Ignore contours which are too small to reduce noise
            area = cv2.contourArea(c)
            if area < area_threshold:
                # Add contour to mask for image
                cv2.drawContours(mask,[c], -1, 0, -1)
        # Flip bits in the binary image from the bask
        final2 = cv2.bitwise_and(final, final, mask=mask)
        # Close gaps in the image using an ellipse kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_sz,kernel_sz))
        remove = cv2.morphologyEx(final2, cv2.MORPH_OPEN, kernel)
        # Invert the image
        remove = (255-remove)
        return remove, img2

    # This subroutine detects all the circles in the image, segments them into
    # NUMBER_OF_WRENCHES bins and returns the (row,col) coordinates of the centers
    # and the mean radius of each group.
    def detect_circle(self,img_gray_hou):
        # Detect the circles using a hough transform
        num_of_circles = 0
        cp0 = self.canny_param[0]
        cp1 = self.canny_param[1]
        mcd = self.max_circ_diam
        if self.preview_flag:
            cv2.imshow('TESTING',img_gray_hou)
            cv2.waitKey(0)
        while num_of_circles == 0:
            circles = cv2.HoughCircles(img_gray_hou, cv2.cv.CV_HOUGH_GRADIENT,1,1,
                np.array([]),cp0,cp1,0,mcd)
            cp0 = cp0*0.9
            cp1 = cp1*0.9
            mcd = mcd+1
            num_of_circles = len(circles)
            print("n,cp0,cp1,mcd:",num_of_circles,cp0,cp1,mcd)
        img_hou_all = img_gray_hou.copy()
        print "NUMBER OF CIRCLES: ", np.shape(circles)
        center_x = circles[0,:,0]
        center_y = circles[0,:,1]
        radius = circles[0,:,2]
        for n in range(len(circles[0,:,1])):
            cv2.circle(img_hou_all,(center_x[n],center_y[n]), radius[n],
                (0,0,244), 2, cv2.CV_AA)
        cv2.imwrite('/home/ugv/wrenchID_6_allcircles.png',img_hou_all)
        #cv2.imshow('All Circles',img_hou_all)
        #cv2.waitKey(0)
        # Establish matrix of features to use for quanitzation
        z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
        # Run K-means to determine centers and to which group each point belongs.
        term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
        flag = cv2.KMEANS_RANDOM_CENTERS
        ret, labels, centers = cv2.kmeans(z, 1, term_crit, 100, flag)
        print centers
        # Find average radius within each K-means group
        rad_kmean = np.zeros([1,1])
        radius = circles[0,:,2]
        for i in range(0,1):
            locs = np.where(labels == i)
            rad = radius[locs[0]]
            print "i, rad: ", i, rad
            rad_kmean[i] = np.mean(rad)
            print "i, rad_kmean: ", i, rad_kmean[i]
        # Store center coordinates from K-means for sorting
        circs = np.zeros([1,3])
        circs[:,0:2] = centers
        circs[:,2:] = rad_kmean

        # Sort circles by x-axis
        circs = circs[circs[:,0].argsort()]
        return circs, img_hou_all

    # This subroutine determines the length and area of each segmented object
    # in the binary image. It returns length, area, and contours sorted by
    # column pixels.
    def detect_geometry(self,img_seg,img):

        # Find contours in binary image
        cnt, hie = cv2.findContours(img_seg,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        # Remove contours that are too small (noise) or too far to the left (valve)
        print "Original number of contours: ", len(cnt)
        bad_flag = 1
        while bad_flag > 0:
            bad_flag = 0
            cen2 = np.zeros([len(cnt),2])
            cnt2 = []
            #cen2 = np.zeros([20,2])
            ct = 0
            for c in range(0,len(cnt)):
                area = cv2.contourArea(cnt[c])
                M = cv2.moments(cnt[c])
                if area > self.area_min_thresh:
                    cen2[ct,0] = int(M["m01"] / M["m00"])
                    cen2[ct,1] = int(M["m10"] / M["m00"])
                    cnt2.append(cnt[c])
                    ct = ct+1
            cen2_mn = np.nanmean(cen2[0:ct-1,1])
            cen2_std = np.nanstd(cen2[0:ct-1,1])
            cen3 = cen2[:,1]
            cen2 = cen3[np.nonzero(cen3)]
            print(cen2)
            print "STD: ", cen2_std
            print "NUMBER OF CONTOURS: ", len(cen2)
            if cen2_std > 250: #len(cen2) > 7:
                bad_flag = 1
                ind = np.argsort(cen2)
                cnt3 = []
                for c in range(0,len(cnt2)):
                    if c != ind[0]:
                        cnt3.append(cnt2[c])
                cnt = cnt3
            else:
                if len(cnt) > self.number_of_wrenches:
                    cnt3 = []
                    min_row = []
                    for c in range(0,len(cnt2)):
                        (hi1,hi2,len2,wid2) = cv2.boundingRect(cnt[c])
                        min_row.append(hi2)
                    print min_row
                    ind = np.argsort(np.asarray(min_row))
                    for c in range(0,len(cnt2)):
                        if c != ind[0]:
                            cnt3.append(cnt2[c])
                    bad_flag = 1
                    cnt = cnt3
                    cnt2 = cnt3
                else:
                    cnt = cnt2
                    bad_flag = 0
            
        print "New number of contours: ", len(cnt)

        # Update parameters using only the good contours
        cnt_len = len(cnt)
        cnt2 = []; hie2 = np.zeros([cnt_len,12,4]); ct = 0
        cen2 = np.zeros([cnt_len,2]); len2 = np.zeros([cnt_len,2])
        area2 = np.zeros([cnt_len,1])
        for c in range(0,cnt_len):
            area = cv2.contourArea(cnt[c])
            M = cv2.moments(cnt[c])
            cen2[c,1] = int(M["m01"] / M["m00"])
            cen2[c,0] = int(M["m10"] / M["m00"])
            area2[c] = area
            cnt2.append(cnt[c])
            (hi1,hi2,len2[c,0],len2[c,1]) = cv2.boundingRect(cnt[c])
            cv2.drawContours(img, cnt[c], -1, (0,c,255-c), 3)
        if self.preview_flag:
            cv2.imshow('img_seg',img)
            cv2.waitKey(0)

        # Store all relevant features in a single matrix
        params = np.zeros([self.number_of_wrenches,8])
        params[:,0:2] = cen2
        params[:,2:4] = len2
        params[:,4:5] = area2.reshape((self.number_of_wrenches,1))
        # Sort contour list by x position
        ind = np.argsort(params[:,0])
        cnt2 = []
        for i in range(0,len(cnt)):
            cnt2.append(cnt[ind[i]])
        # Sort feature matrix by x position of centroid
        params = params[params[:,0].argsort()]

        return params, cnt2

    # This subroutine applies a Gaussian voting algorithm to determine which
    # wrench is the correct one using the diameter, length, and area from each
    # wrench
    def voting(self,params):
        votes = np.zeros([self.n_wr,3])
        votes_ideal = np.zeros([1,3])
        # Store the maximum possible probability for each parameter based
        # on the Gaussian distribution
        votes_ideal[0,0] = scipy.stats.norm(self.d_mu, self.d_sig).pdf(self.d_mu)
        votes_ideal[0,1] = scipy.stats.norm(self.l_mu, self.l_sig).pdf(self.l_mu)
        votes_ideal[0,2] = scipy.stats.norm(self.a_mu, self.a_sig).pdf(self.a_mu)
        # Compute the probability for each wrench using each parameter
        # based on the Gaussian distribution
        for i in range(0,self.number_of_wrenches):
            votes[i,0] = scipy.stats.norm(self.d_mu, self.d_sig).pdf(params[i,5])
            votes[i,1] = scipy.stats.norm(self.l_mu, self.l_sig).pdf(params[i,3])
            votes[i,2] = scipy.stats.norm(self.a_mu, self.a_sig).pdf(params[i,4])
        # Scale the probabilities based on the maximum possible for each
        # parameter
        votes = votes/votes_ideal
        vote_result = np.zeros([self.n_wr,1])
        # Sum the probabilities based on the weight values for each parameter
        vote_result = np.dot(votes,self.vote_wt)
        ind = vote_result.argsort()
        return vote_result, ind[self.n_wr-1]

    # This subroutine generates an image showing the probability that each
    # wrench is the correct one. It returns one image with all the probabilities
    # shown and one image with only the best match shown.
    def visualize_probability(self,img, vote_result, n, params, cnt):
        img_kmeans = img.copy()
        # Visualize the probabilities
        for i in range(min(self.n_wr,len(cnt))):
            c = int(round(vote_result[i]*255))
            #cv2.circle(img_kmeans,(int(circs[i,0]),int(circs[i,1])), 
            #    int(circs[i,2]), (0,c,255-c), 2, cv2.CV_AA)
            #print "d,x,y: ", params[i,5:8]
            #print "params[i,:]: ", params[i,:]
            cv2.circle(img_kmeans,(int(params[i,6]),int(params[i,7])), 
                int(params[i,5]), (0,c,255-c), 2, cv2.CV_AA)
            cv2.drawContours(img_kmeans, cnt[i], -1, (0,c,255-c), 3)
            #cv2.imshow('Prob',img_kmeans)
            #cv2.waitKey(0)
        img_kmeans_id = img_kmeans.copy()
        # Visualize the best match
        img_id = img.copy()
        #cv2.circle(img_id,(int(circs[n,0]),int(circs[n,1])), 
        #    int(circs[n,2]), (0,255,0), 2, cv2.CV_AA)
        print "N EQUALS: ", n
        print "NUMBER OF CONTOURS EQUALS: ", len(cnt)
        cv2.circle(img_id,(int(params[n,6]),int(params[n,7])), 
                int(params[n,5]), (0,255,0), 2, cv2.CV_AA)
        try:
            cv2.drawContours(img_id, cnt[n], -1, (0,255,0), 3)
        except:
            pass
        #cv2.circle(img_kmeans_id,(int(circs[n,0]),int(circs[n,1])), 
        #    int(circs[n,2]), (255,0,0), 2, cv2.CV_AA)
        cv2.circle(img_kmeans_id,(int(params[n,6]),int(params[n,7])), 
                int(params[n,5]), (255,0,0), 2, cv2.CV_AA)
        try:
            cv2.drawContours(img_kmeans_id, cnt[n], -1, (255,0,0), 3)
        except:
            pass
        return img_kmeans, img_id, img_kmeans_id


    # shutdown runs when this node dies
    def shutdown(self):
        rospy.sleep(0.1)

    # callback_wrench takes the RGB image and determines the (x,y,z) location of
    # the correct wrench.
    def callback(self, data):
        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = img
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        idwrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idwrench finished.")

