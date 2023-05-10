#!/usr/bin/env python

import os
import cv2
# import torch

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from PIL import Image, ImageDraw

class StopSignDetector:
  def __init__(self, threshold=0.5):
    # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
    self.threshold = threshold
    self.results = None

  def predict(self, img):
    """
    Takes in a path or numpy array representing an image
    returns whether or not there is a stop sign
    """
    # for on robot change the contours line and comment out some of image plots

    img_copy = img.copy()

    # x = len(img[0])

	  # # write a more efficient way to zero out the top and bottom of the image
	  # img_copy[:170, :x] = [0,0,0]
	  # img_copy[250:len(img), :x] = [0,0,0]

    stop = img_copy
    # image passed in is RGB
    hsv_stop = cv2.cvtColor(stop, cv2.COLOR_RGB2HSV)

    # need to be tuned to match the hsv values for the stop sign from the zed camera
    lower = (0,48,170)
    upper = (10,255,255)

    mask = cv2.inRange(hsv_stop, lower, upper)
    #define kernel size  
    kernel = np.ones((6,6),np.uint8)

    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    result = cv2.bitwise_and(stop, stop, mask=mask)

    # Return bounding box
    # change to _, contours, _ = ... for on robot
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # view image with contours drawn
    # output = cv2.drawContours(result, contours, -1, (0, 0, 255), 3)
    # cv2.imshow("Output", output)
    # cv2.waitKey()

    max_ind = 0
    max = 0
    for ind in range(len(contours)):
      if len(contours[ind]) > max:
        max = len(contours[ind])
        max_ind = ind

    if len(contours)>0:
      x, y, w, h = cv2.boundingRect(contours[max_ind])

      # add bounding boxes to images
      stop = cv2.rectangle(stop, (x, y), (x+w, y+h), (255, 0, 0), 20)
      stop = cv2.cvtColor(stop, cv2.COLOR_BGR2RGB)

      #cv2.imshow("Bounding Rectangle", stop)
      #cv2.waitKey(0)
      #cv2.destroyAllWindows()

      return 1, [x,y,x+w,y+h] # these may be in wrong order

    else:
      return False, [0,0,0,0]


# Utilities

# Image
def read_image(path):
    rgb_im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    return rgb_im

def draw_rect(im, xmin, ymin, xmax, ymax):
    box = xmin, ymin, xmax, ymax
    img = Image.fromarray(im)
    imgd = ImageDraw.Draw(img)
    imgd.rectangle(box, outline='red')
    return img

def draw_box(im, box):
    img = Image.fromarray(im)
    imgd = ImageDraw.Draw(img)
    imgd.rectangle(box, outline='red')
    return img

# Detecting Utils

THRESHOLD = 0.7

def is_stop_sign(df, label='stop sign', threshold=THRESHOLD):
    confidences = df[df['confidence'] > threshold]
    return len(confidences[confidences['name'] == label]) != 0 # If a stop sign has been detected

def get_bounding_box(df, label='stop sign', threshold=THRESHOLD):
    if not is_stop_sign(df, label=label, threshold=threshold): return (0, 0, 0, 0)
    confidences = df[df['confidence'] > threshold]
    stop_sign = confidences[confidences['name'] == label].head(1)
    coords = stop_sign.xmin, stop_sign.ymin, stop_sign.xmax, stop_sign.ymax
    return [coord.values[0] for coord in coords]

# if __name__=="__main__":
#     detector = StopSignDetector()
#     path = "/Users/haleysanchez/racecar_docker/home/racecar_ws/src/final_challenge2023/city_driving/test_images/IMG_5011.jpeg"
#     img_msg = cv2.imread(path)
#     rgb_img = cv2.cvtColor(img_msg, cv2.COLOR_BGR2RGB)
#     see_stop_sign, bounding_box = detector.predict(rgb_img)
#     print(see_stop_sign, bounding_box)

