#!/usr/bin/env python
import os
import sys
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class zed_opencv_wire_detector():
    def __init__(self):

        # Required to convert ROS images to OpenCV images
        self._cv_bridge = CvBridge()

        # Subscribes to the rectified, color image topic
        self.image_sub = message_filters.Subscriber('/zed2/zed_node/left/image_rect_color', Image, queue_size = 1)

        # Subscribes to the depth topic (Not currently used)
        self.depth_sub = message_filters.Subscriber('/zed2/zed_node/depth/depth_registered', Image, queue_size = 1)

        # Subscrubes to camera information
        self.cam_info_sub = rospy.Subscriber('/zed2/zed_node/rgb/camera_info', CameraInfo, self.camera_info, queue_size = 1)

        # Time syncrhonization
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        ts.registerCallback(self.callback)

        # Topic that publishes the ROS images with wires highlighted green
        self._img_pub = rospy.Publisher('/obj_det/wire', Image, queue_size = 1)


    def callback(self, image_msg, depth_msg):

        # Converts the incoming image topic as a cv image to allow our cv operations to take place, but makes sure that it can convert!
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        depth_img = self._cv_bridge.imgmsg_to_cv2(depth_msg)

        mask_images = [];
        
        #Converts the input imag e
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Range for lower reds
        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        red_mask1 = cv2.inRange(hsv, lower_red, upper_red)
        #mask_images = red_mask1
        #mask_images = np.hstack((mask_images,red_mask1))
        
        # Range for upper reds
        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        red_mask2 = cv2.inRange(hsv,lower_red,upper_red)
        #mask_images.append(red_mask2)
        mask_images = red_mask1 + red_mask2
        #mask_images = np.hstack((mask_images,red_mask2))
                
        # Range for greens
        lower_green = np.array([25, 87, 111])
        upper_green = np.array([102, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        #mask_images.append(green_mask)
        #mask_images = np.hstack((mask_images,green_mask))
                        
        # Range for whites
        lower_white = np.array([220, 220, 220])
        upper_white = np.array([255, 255, 255])
        white_mask = cv2.inRange(cv_image, lower_white, upper_white) # Note that white uses RGB, not HSV
        #mask_images.append(white_mask)
        mask_images = np.hstack((mask_images,white_mask+green_mask))
                
        kernel_1 = np.zeros((9,9), np.uint8) # 9x9 array of zeros
        kernel_2 = np.zeros((9,9), np.uint8) # 9x9 array of zeros
        kernel_3 = np.zeros((9,9), np.uint8) # 9x9 array of zeros
        
        # Makes the kernel be all zeros execpt for the 5th element of each, which are set to 1
        for k in kernel_1:
            k[4] = 1

        # Makes the kernel have a diagonal of 1s from the bottom left to the top right
        cnt = 8
        for k in kernel_2:
            k[cnt] = 1
            cnt = cnt - 1
            
        kernel_3 = np.fliplr(kernel_2)

        mask = red_mask1 + red_mask2 + green_mask + white_mask # Combines the mask for all colors
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel_1) # Uses a dilation morph on the first kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel_2) # Uses a dilation morph on the second kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel_3) # Uses a dilation morph on the second kernel        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_1) # Uses an opening morph on the first kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_2) # Uses an opening morph on the second kernel

        # Creates and applies the contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im = np.copy(cv_image)
        cv2.drawContours(im, contours, -1, (0, 255, 0), 1)

        # Converts back to an image to publish to it's new topic
        self._img_pub.publish(self._cv_bridge.cv2_to_imgmsg(im, "bgr8"))
        
        # construct the montages for the images
        mask_images = np.hstack((mask_images, mask))
        cv2.namedWindow("Montage", cv2.WINDOW_NORMAL)
        cv2.imshow("Montage", mask_images)
        cv2.waitKey(1)
        
        
    # Allows resizing of cv2 images with a given aspect ratio width x height
    def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
        dim = None
        (h, w) = image.shape[:2]

        if w is None and h is None:
            return image
        if w is None:
            r = height / float(h)
            dim = (int(w * r), height)
        else:
            r = width / float(w)
            dim = (width, int(h * r))
        return cv2.resize(image, dim, interpolation = inter)

    # Needed to publish camera information
    def camera_info(self, data):
        self.img_h = data.height
        self.img_w = data.width
        self.h_res = 90.0/self.img_w
        self.v_res = 80.0/self.img_h
        self.cam_info_sub.unregister()

    # Main
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('zed_opencv_wire_detector')
    wire_det = zed_opencv_wire_detector()
    wire_det.main()
