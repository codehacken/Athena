"""
All processing functions.
AUTHOR: Ashwinkumar Ganesan, Karan K. Budhraja, Nisha Pillai, Gurpreet Singh.
"""

# import image processing library
from lib.image.common import utils
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su

# import language processing library
from lib.lang.nlp import LanguageModule as lm

# This is to initialize the joint model
# either with a precomputed model or by computing it in the function.
def initialize_model():
    # Write all initializations here.
    # The returned value is a joint model.
    return None

# write the main processing node for the model
# a joint model object is maintained in the main cpu loop
# later, a language model will also be maintained in main cpu loop
def process_model(cv_image, message, jointModelObject, print_message, if_test):
    print("Process Model: " + message)
    # convert cv image into processing format
    # TODO: this needs to be corrected
    # we do not read a file
    # we convert from one format to the other
    #image = utils.imageRead(imageFile)
    image = cv_image

    # extract color and shape of image
    # image_copy = copy.copy(image)
    cnt = utils.objectIdentification(cv_image)
    [x, y, w, h] = utils.boundingRectangle(cnt)
    pixNp = dc.findAllPixels(image, cnt, x, y, w, h)
    pixNp = dc.findUniquePixels(pixNp)

    # store image data as dictionary
    imageData = {}
    imageData['color'] = pixNp
    imageData['shape'] = cnt

    # There is no processing of a language for the testing phase.
    if(not if_test):
        # extract keywords from message
        languageObject = lm(message)
        [positiveLanguageData, negativeLanguageData] = languageObject.process_content()

        # for each keyword
        # add keyword, image pair to joint model
        for keyword in positiveLanguageData:
            jointModelObject.add_word_example_pair(keyword, imageData, "+")

        for keyword in negativeLanguageData:
            jointModelObject.add_word_example_pair(keyword, imageData, "-")

    # Testing Phase.
    elif(if_test):
        # Call print message to publish the message to the output node.
        # print_message(some_message)

        # call novel scene
        [bestGuessWord, isConfidentGuess, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores, maxScoreObj] = jointModelObject.classify_example(imageData)

        # use wordMaxProabilityScores, bestGuessWord and maxScoreObj
        index = 0
        for word in wordMaxProabilityScores:
            print_message(str(index+1) + " " + word + " " + wordMaxProabilityScores[word])

        # print new line for cleanliness
        print_message("")
        
        # print the conclusion
        print_message("This is the " + str(type(maxScoreObj)) + " " + bestGuessWord)

        # This is temporary code.
        """
        print(dir(jointModelObject))
        print(jointModelObject.knownWords)
    
        a = jointModelObject.classify_word_example('red', imageData)
        print(a)
    
        a = jointModelObject.classify_example(imageData)
        print(a)
        """
