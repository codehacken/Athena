"""
All processing functions.
AUTHOR: Ashwinkumar Ganesan, Karan K. Budhraja, Nisha Pillai, Gurpreet Singh.
"""

# import image processing library
from imagelib.common import utils
from imagelib.color import detectColor as dc
from imagelib.shape import shapeUtils as su

# import language processing library
import LanguageModule

# write the main processing node for the model
# a joint model object is maintained in the main cpu loop
# later, a language model will also be maintained in main cpu loop 
def process_model(cv_image, message, jointModelObject):
    print("Process Model: " + message)

	# convert cv image into processing format
	# TODO: this needs to be corrected
	# we do not read a file
	# we convert from one format to the other
	image = utils.imageRead(imageFile)
	
	# extract color and shape of image
	image_copy = copy.copy(image)
	cnt = utils.objectIdentification(imageFile)
	[x, y, w, h] = utils.boundingRectangle(cnt)
	pixNp = dc.findAllPixels(image_copy, cnt, x, y, w, h)
	pixNp = dc.findUniquePixels(pixNp)

	# store image data as dictionary
	imageData = {}
	imageData['color'] = pixNp
	imageData['shape'] = cnt

	# extract keywords from message
	languageObject = LanguageModule.LanguageModule(message)
	[positiveLanguageData, negativeLanguageData] = languageObject.process_content()

	# for each keyword
	# add keyword, image pair to joint model
	for keyword in positiveLanguageData:
		jointModelObject.add_word_example_pair(keyword, imageData, "+")
	
	for keyword in negativeLanguageData:
		jointModelObject.add_word_example_pair(keyword, imageData, "-")

