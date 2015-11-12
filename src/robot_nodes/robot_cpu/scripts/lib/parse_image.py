########################################################
#parse_image.py

#   This script handles the training and testing part of
#      Color attributes of an object in the picture
#
#  Author : Nisha Pillai
#  Date : 11/02/2015
#
########################################################

import cv2
import copy
from lib.color import detectColor as dc
from lib.common import utils
from lib.common import argParser
from lib.shape import shapeUtils as su

args = argParser.argument_parse()

imageFile = args["image"]

while True:

   image = utils.imageRead(imageFile)
   image_copy = copy.copy(image)


   cnt = utils.objectIdentification(imageFile)
   x,y,w,h = utils.boundingRectangle(cnt)

   pixNp = dc.findAllPixels(image_copy,cnt,x,y,w,h)
   print "Number of Pixels inside the contour : ", len(pixNp)

   avgB,avgG,avgR = dc.findAverageRGB(pixNp)
   print "Average  B G R value inside the Contour : ", avgB,avgG,avgR

   min,max = dc.findMinMaxOfPixels(pixNp)
   print "Min and Max B G R value inside the contour : ",min, max

   pixNp = dc.findUniquePixels(pixNp)
   print "Number of Unique Pixels inside the Contour : ",len(pixNp)

   x1 = (x + x + w) / 2
   y1 = (y + y + h) / 2
   print "B G R of Pixel from the middle of the image : ",dc.findColorFromPixel(image_copy,y1,x1)
   #print "Object Shape : ",cnt

   attrs = utils.getColorShapeAttributes()
   for obj in attrs:
      print "Object Comparison : "
      su.compare_items(obj['shape'],cnt)
      dc.compare_items(obj['color'],pixNp)
#      print obj['shape']

   utils.addColorShapeAttributes(cnt,pixNp)

#   utils.displayColorShapeAttributes()
   
 
#   dc.selectColor(image_copy,'yellow')
   
   imageFile =  ""
   while not imageFile:
      n = raw_input( " New Image : ")
      imageFile =  n.strip()