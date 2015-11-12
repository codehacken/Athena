########################################################
# detectColor.py
#
#   This module does color segmentation part of the project
#
#  Author : Nisha Pillai
#  Date : 11/02/2015
#
########################################################


import cv2
import numpy as np

BOUNDARIES1 = {
    'red': ([170, 160, 60], [180, 255, 255]),
    'blue': ([110, 50, 50], [130, 255, 255]),
    'green': ([38, 50, 50], [75, 255, 255]),
    }

# Ideally boundary lower upper values should be set by the joint learning framework. 
# Here values are added statically for testing the modules

BOUNDARIES2 = {
    'red': ([0, 0, 60], [50, 50, 255]),
    'blue': ([86, 31, 4], [220, 88, 50]),
    'yellow': ([25, 146, 190], [62, 174, 250]),
    }

BOUNDARIES = {
    'red': ([4, 0, 29], [195, 183, 239]),
    'blue': ([86, 31, 4], [220, 88, 50]),
    'yellow': ([25, 146, 190], [62, 174, 250]),
    }


def selectRange(color):
   for k,v in BOUNDARIES.iteritems():
      if k == color :
         return v
       

def selectColor(image,color):
   (lower,upper) = selectRange(color)
   lower = np.array(lower, dtype = "uint8")
   upper = np.array(upper, dtype = "uint8")

   m = cv2.inRange(image, lower, upper)
   result = cv2.bitwise_and(image, image, mask = m)
   cv2.imshow("images",result)
   cv2.waitKey(0)



def findColorFromPixel(image,x,y) :
   pixel = image[x,y]
   return pixel


def findAllPixels(image_copy,cnt,x,y,w,h):
   points = []
   pixels = []
   gray_threshold = 20
   for i in range(x,x+w) :
      for j in range(y,y+h) :
         dst = cv2.pointPolygonTest(cnt,(i,j),False)
         if dst == 1.0 :
            b, g, r = image_copy[j,i]
            px = [int(b),int(g),int(r)]
            diff = px[0] - px[1]
            if abs(px[0]-px[1]) > gray_threshold or abs(px[0]-px[2]) > gray_threshold or abs(px[1]-px[2]) > gray_threshold :
               points.append([i,j])
               pixels.append(px)


   pixNp = np.array(pixels)
   return pixNp  


def findAverageRGB(pixNp) :
   A = np.vstack([pixNp])
   return np.average(A, axis=0, weights=A.astype(bool))

def findMinMaxOfPixels(pixNp):
   minPix = np.min(pixNp, axis=0)
   maxPix = np.max(pixNp, axis=0)
   return (minPix,maxPix)

def findUniquePixels(pixNp):
   pixNp = set(map(tuple,pixNp))
   return pixNp

def compare_items(pix1, pix2):
   inter = pix1.intersection(pix2)
   equalRate = float(len(pix1) - len(inter))/float(len(pix1))
   print "Color is compared : ", equalRate

