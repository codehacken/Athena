"""
All processing functions.
AUTHOR: Ashwinkumar Ganesan, Karan K. Budhraja, Nisha Pillai, Gurpreet Singh.
"""

# import image processing library
from lib.image.common import utils
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su

import pickle
# import language processing library
from lib.lang.nlp import LanguageModule as lm

question_asked = False

# This is to initialize the joint model
# either with a precomputed model or by computing it in the function.
def initialize_model():
    # Write all initializations here.
    # The returned value is a joint model.
    try:
        with open('data/pickle/passive_jointModelObject.pickle', 'rb') as handle:
            jointModelObject = pickle.load(handle)
    except EOFError:
        print("Could not read joint model")
        jointModelObject = None
        pass

    return jointModelObject

# write the main processing node for the model
# a joint model object is maintained in the main cpu loop
# later, a language model will also be maintained in main cpu loop
def add_example(cv_image, message, jointModelObject, print_message, example_count):
    # convert cv image into processing format
    # TODO: this needs to be corrected
    # we do not read a file
    # we convert from one format to the other
    # image = utils.imageRead(imageFile)
    print("Adding an Example...")
    if cv_image == None:
        print_message("Unable to capture an Image.")
        return

    image = cv_image

    # extract color and shape of image
    # image_copy = copy.copy(image)
    cnt = utils.objectIdentification(cv_image)
    [x, y, w, h] = utils.boundingRectangle(cnt)
    pixNp = dc.findAllPixels(image, cnt, x, y, w, h)
    pixNp = dc.findUniquePixels(pixNp)

    # store image data as dictionary
    imageData = {'color': pixNp, 'shape': cnt}
    print("Printing the size of RGB value " + str(len(pixNp)))

    # extract keywords from message
    languageObject = lm(message)
    [positiveLanguageData, negativeLanguageData] = languageObject.process_content()

    # for each keyword
    # add keyword, image pair to joint model
    for keyword in positiveLanguageData:
        jointModelObject.add_word_example_pair(keyword, imageData, "+")

    for keyword in negativeLanguageData:
        jointModelObject.add_word_example_pair(keyword, imageData, "-")

    # Send ACK to output Node that the concept has been added.
    # Pickle the data to store training information.
    # Store size is the number of examples after which the model is stored to a pickle file.
    store_size = 20
    if ((example_count % store_size) == 0):
        with open('data/pickle/passive_jointModelObject.pickle', 'wb') as handle:
            pickle.dump(jointModelObject, handle)

    print_message("Example Object - Concept Added. Number of Examples Added: " + str(example_count))

def test_example(cv_image, message, jointModelObject, print_message):
    # convert cv image into processing format
    # TODO: this needs to be corrected
    # we do not read a file
    # we convert from one format to the other
    #image = utils.imageRead(imageFile)
    print("Test an Example...")
    if cv_image == None:
        print_message("Unable to capture an Image.")
        return

    image = cv_image

    # extract color and shape of image
    # image_copy = copy.copy(image)
    cnt = utils.objectIdentification(cv_image)
    [x, y, w, h] = utils.boundingRectangle(cnt)
    pixNp = dc.findAllPixels(image, cnt, x, y, w, h)
    pixNp = dc.findUniquePixels(pixNp)

    # store image data as dictionary
    imageData = {'color': pixNp, 'shape': cnt}
    print("Printing the size of RGB value " + str(len(pixNp)))

    # call novel scene
    [bestGuessWord, isConfidentGuess, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores,
     maxScoreObj] = jointModelObject.classify_example(imageData)

    # use wordMaxProabilityScores, bestGuessWord and maxScoreObj
    for idx, word in enumerate(wordMaxProabilityScores):
        print_message(str(idx+1) + " " + word + " " + str(wordMaxProabilityScores[word]))

    # print new line for cleanliness
    print_message(" ")

    # print the conclusion
    print_message("This is the " + maxScoreObj._type_ + " " + bestGuessWord)

def learn_example(cv_image, message, al_framework, print_message, ask_question):
    # convert cv image into processing format
    # TODO: this needs to be corrected
    # we do not read a file
    # we convert from one format to the other
    # image = utils.imageRead(imageFile)

    print("Learning an Example....")

    #if(not question_asked):
    """
    if cv_image == None:
        print_message("Unable to capture an Image.")
        return

    image = cv_image

    # extract color and shape of image
    # image_copy = copy.copy(image)
    cnt = utils.objectIdentification(cv_image)
    [x, y, w, h] = utils.boundingRectangle(cnt)
    pixNp = dc.findAllPixels(image, cnt, x, y, w, h)
    pixNp = dc.findUniquePixels(pixNp)

    # store image data as dictionary
    imageData = {'color': pixNp, 'shape': cnt}
    print("Printing the size of RGB value " + str(len(pixNp)))

    # extract keywords from message
    languageObject = lm(message)
    [positiveLanguageData, negativeLanguageData] = languageObject.process_content()

    # for each keyword
    # add keyword, image pair to joint model
    for keyword in positiveLanguageData:
        jointModelObject.add_word_example_pair(keyword, imageData, "+")

    for keyword in negativeLanguageData:
        jointModelObject.add_word_example_pair(keyword, imageData, "-")


    ask_question(msg_id, msg_str)
    """
    #else:
    print_message("Do Something")
    #question_asked = False

    # Send ACK to output Node that the concept has been added.
    #print_message("Example Object - Concept Added.")
