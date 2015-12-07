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
from image.color import detectColor as dc
from image.common import utils
from image.common import argParser
from image.shape import shapeUtils as su
from framework import JointModel as jm
from alframework import ALUniRobotDrivenModel as al


def imageRead(imageFile):
   image = cv2.imread(imageFile)
   return image

args = argParser.argument_parse()

imageFile = args["image"]

image1 = imageRead(imageFile)
image_copy = copy.copy(image1)

def getShapeColor(image):
   cnt = utils.objectIdentification(image)
   x,y,w,h = utils.boundingRectangle(cnt)

   pixNp = dc.findAllPixels(image_copy,cnt,x,y,w,h)
   #print "Number of Pixels inside the contour : ", len(pixNp)

   avgB,avgG,avgR = dc.findAverageRGB(pixNp)
   #print "Average  B G R value inside the Contour : ", avgB,avgG,avgR

   min,max = dc.findMinMaxOfPixels(pixNp)
   #print "Min and Max B G R value inside the contour : ",min, max

   pixNp = dc.findUniquePixels(pixNp)
   #print "Number of Unique Pixels inside the Contour : ",len(pixNp)

   x1 = (x + x + w) / 2
   y1 = (y + y + h) / 2
   #print "B G R of Pixel from the middle of the image : ",dc.findColorFromPixel(image_copy,y1,x1)
   #print "Object Shape : ",cnt

   attrs = utils.getColorShapeAttributes()

   for obj in attrs:
     #print "Object Comparison : "
     su.compare_items(obj['shape'],cnt)
     dc.compare_items(obj['color'],pixNp)
     #print obj['shape']

   utils.addColorShapeAttributes(cnt,pixNp)
   return cnt,pixNp


#utils.displayColorShapeAttributes() 
#dc.selectColor(image_copy,'yellow')

########################################################
# karan testing joint model
########################################################


[cnt,pixNp]  = getShapeColor(image1)
image = {}
image['shape'] = cnt
image['color'] = pixNp
#print pixNp

al = al()

qType = 0
words = []
examplePolarity = ""

al.add_word_example_pair(qType, words, image, examplePolarity)

print "-----------------1------------------------"
qType = 1
words = ["red","cube"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

qType = 1
words = ["red1","square"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)
'''
image1 = imageRead("frame0001.jpg")
[cnt,pixNp]  = getShapeColor(image1)
image = {}
image['shape'] = cnt
image['color'] = pixNp

print "------------------2--------------------------"
qType = 2
words = ["yes"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

qType = 2
words = ["no"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

image1 = imageRead("frame0006.jpg")
[cnt,pixNp]  = getShapeColor(image1)
image = {}
image['shape'] = cnt
image['color'] = pixNp

print "--------------------------3------------------------"
qType = 3
words = ["yes"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

qType = 3
words = ["no"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

image1 = imageRead("frame0007.jpg")
[cnt,pixNp]  = getShapeColor(image1)
image = {}
image['shape'] = cnt
image['color'] = pixNp

print "----------------------------------4---------------"
qType = 4
words = ["yes"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

qType = 4
words = ["no"]
examplePolarity = "+"
al.add_word_example_pair(qType, words, image, examplePolarity)

'''
'''
# test known example

jointModel = jm()
jointModel.add_word_example_pair("blue", image, "+")
jointModel.add_word_example_pair("thing", image, "+")
jointModel.add_word_example_pair("roller", image, "+")

#print(jointModel.knownWords)
for word in jointModel.knownWords:
	for classifier in jointModel.knownWords[word]:
		#print(classifier.word, type(classifier))
		pass		

#result = jointModel.classify_word_example("blue", image)
#result = jointModel.classify_word_example("thing", image)
#result = jointModel.classify_word_example("roller", image)
print(len(result[1]))
#for classifier in result[1]:
#	print(classifier.word, type(classifier))
'''
'''
# test novel scene

jointModel = jm()
jointModel.add_word_example_pair("blue", image, "+")
jointModel.add_word_example_pair("thing", image, "+")
jointModel.add_word_example_pair("roller", image, "+")

print(jointModel.knownWords)
for word in jointModel.knownWords:
	for classifier in jointModel.knownWords[word]:
		print(classifier.word, type(classifier))

[bestGuessWord, isConfidentGuess, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores] = jointModel.classify_example(image)
#print(bestGuessWord)
#print(isConfidentGuess)
#print(bestGuessMaxScore)
#print(wordMaxProabilityScores)
#print(wordProbabilityScores)

#print(wordProbabilityScores['blue'])
#print(len(wordProbabilityScores['blue'][1]))
#print(wordProbabilityScores['thing'])
#print(len(wordProbabilityScores['thing'][1]))
print(wordProbabilityScores['roller'])
print(len(wordProbabilityScores['roller'][1]))
'''
