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

objectAttributes = []

def imageRead(imageFile):
   #image = cv2.imread(imageFile)
   image = imageFile
   return image

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
      cv2.imshow("Contour",im)
      cv2.waitKey(0)
      killWindows()

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

def drawRectangle(imageFile,cnt) :
   im = imageRead(imageFile)
   x,y,w,h = boundingRectangle(cnt)
   img = cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,255),2)
   cv2.imshow("Object Selection ",im)
   cv2.waitKey(0)
   killWindows()


def objectIdentification(imageFile) :
   image = imageRead(imageFile)
   imgray = colorToGray(image)
   thresh = threshChange(imgray)
   contours = findContour(thresh)
   contNo = selectContour(imageFile,contours)
   cnt = contours[contNo]
   drawRectangle(imageFile,cnt) 
   return cnt

 
def addColorShapeAttributes(cnt,pixels) :
   attr = {'color' : pixels, 'shape' : cnt}
   objectAttributes.append(attr)


def displayColorShapeAttributes():
   for obj in objectAttributes:
      print obj

def getColorShapeAttributes() :
   return objectAttributes

