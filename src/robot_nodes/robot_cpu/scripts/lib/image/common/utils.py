########################################################
# Utils.py

#   This script handles common parsing of the image 
#             processing module
#      
#
#  Author : Nisha Pillai
#  Date : 11/02/2015
#
########################################################

import cv2
import numpy as np
objectAttributes = []

def imageRead(imageFile):
   #image = cv2.imread(imageFile)
   return imageFile

def killWindows() :
   cv2.destroyAllWindows()

def colorToGray(image):
   imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
   return imgray

def threshChange(image):
   ret,threshold = cv2.threshold(image,127,255,0)
   return threshold


def findContour(threshold):
   contours, hie = cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
   return contours


def selectContour(imageFile,contours):
   for index in range(len(contours)) :
      print "Object ",index
      im = imageRead(imageFile)
      cv2.drawContours(im, contours, index, (0,255,0), 3)
      cnt = contours[index]
      # cv2.imshow("Contour",im)
      # cv2.waitKey(0)
      # killWindows()

   while True:
      try:
         contNo  = raw_input("Please enter The object Identification No: ")
         if not contNo.strip():
            continue
         contNo = int(contNo)
      except ValueError:
         print("Sorry, I didn't understand that.")
         continue
      if contNo < 0:
         print("Sorry, your response must not be negative.")
         continue
      else :
         break   
   
   return contNo

def boundingRectangle(contour):
   x,y,w,h = cv2.boundingRect(contour)
   return (x,y,w,h)

def drawRectangle(image,cnt) :
   im = image
   x,y,w,h = boundingRectangle(cnt)
   img = cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,255),2)
   # cv2.imshow("Contour",im)
   # cv2.waitKey(0)
   # killWindows()
   return im

def objectIdentification1(imageFile) :
   image = imageRead(imageFile)
   imgray = colorToGray(image)
   thresh = threshChange(imgray)
   contours = findContour(thresh)
   contNo = selectContour(imageFile,contours)
   cnt = contours[contNo]
   drawRectangle(imageFile,cnt) 
   return cnt

def draw_contour(image, c, i):
        # compute the center of the contour area and draw a circle
        # representing the center
        M = cv2.moments(c)
#       cX = int(M["m10"] / M["m00"])
#       cY = int(M["m01"] / M["m00"])

        # draw the countour number on the image
#       cv2.putText(image, "#{}".format(i + 1), (cX - 20, cY), cv2.FONT_HERSHEY_SIMPLEX,
#               1.0, (255, 255, 255), 2)

        # return the image with the contour number drawn on it
        image = drawRectangle(image,c)

        return image

def objectIdentification(imageFile) :
   image = imageRead(imageFile) 
   accumEdged = np.zeros(image.shape[:2], dtype="uint8")
   # loop over the blue, green, and red channels, respectively
   for chan in cv2.split(image):
        # blur the channel, extract edges from it, and accumulate the set
        # of edges for the image
        chan = cv2.medianBlur(chan, 11)
        edged = cv2.Canny(chan, 50, 200)
        accumEdged = cv2.bitwise_or(accumEdged, edged)
# show the accumulated edge map
   # cv2.imshow("Edge Map", accumEdged)
   # find contours in the accumulated image, keeping only the largest
   # ones
   (cnts, _) = cv2.findContours(accumEdged.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
   orig = image.copy()

   print len(cnts)
   newC  = cnts[0]
   oldArea = 0.0
   for c in cnts : 
      area = cv2.contourArea(c)
      print "Area",area
      if area > oldArea :
         newC = c
     
   orig = draw_contour(orig, newC, 1)

   # show the original, unsorted contour image
   # cv2.imshow("Unsorted", orig)
   return newC

def addColorShapeAttributes(cnt,pixels) :
   attr = {'color' : pixels, 'shape' : cnt}
   objectAttributes.append(attr)


def displayColorShapeAttributes():
   for obj in objectAttributes:
      print obj

def getColorShapeAttributes() :
   return objectAttributes
