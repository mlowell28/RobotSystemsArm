#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
class Perception:
    def __init__(self, range_rgb = None):
        
        self.range_rgb = range_rgb
    
        if range_rgb == None:    
            self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            }

    def setTargetColor(self, target_color):
        self.target_color = target_color

# find outline of largest area
# contour list, each contour is checked and largest area selected
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # for each saved contour
            contour_area_temp = math.fabs(cv2.contourArea(c))  # calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # the contour of the maximum area is valid to filter out interference only if
                    area_max_contour = c     # the area is greater than 300

        return area_max_contour, contour_area_max  # return largest outline
    
    
    def locateObject(self, img, color, mask = None, roi=None):
        size = (640, 480)
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
         
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        
        if roi != None:
            frame_gb = getMaskROI(frame_gb, roi, size)    

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert image to LAB space
        
        area_max = 0
        areaMaxContour = 0
        

        frame_mask = cv2.inRange(frame_lab, color_range[color][0], color_range[color][1])  # Threshold color of camera image to be range around expected block color
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # filter out detections outside of block
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # filter out non-detections within object
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # generate contours
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find contour with max area
        
        img_centerx, img_centery, world_x, world_y, roi = None, None, None, None, None
        
        if area_max > 2500:  # if maximum area is above threshold
            
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box) #get ROI area

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # get center pixel cordinates of block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) # convert block pixel cordinates to world position
            
            cv2.drawContours(img, [box], -1, self.range_rgb[color], 2)
            cv2.putText(img, '(' + str(img_centerx) + ',' + str(img_centery) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color], 1) # draw the center point on image
         
        return (img_centerx, img_centery), (world_x, world_y), roi, img 
    
    
    def run(self, color = 'red'):
               
        my_camera = Camera.Camera()
        my_camera.camera_open()
        
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                pixel_location, world_location, roi, img = self.locateObject(frame, color)           
                cv2.imshow('Frame', img)
                key = cv2.waitKey(1)
                if key == 27:
                    break
                    
        my_camera.camera_close()
        cv2.destroyAllWindows()
    
        
if __name__ == '__main__':
    
    perception_object = Perception()
    perception_object.run('red')
    

