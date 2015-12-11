"""
All processing functions.
AUTHOR: Ashwinkumar Ganesan, Karan K. Budhraja, Nisha Pillai, Gurpreet Singh.
"""

# import image processing library
from lib.image.common import utils
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su
from time import ctime

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
    store_size = 8
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

    print(ctime())
    # call novel scene
    [isConfidentGuess, bestGuessWord, bestGuessObj, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores] = jointModelObject.classify_example(imageData)

    # form a dictionary of score: word
    wordScoreDictionary = {}
    for word in wordMaxProabilityScores:
        if(wordMaxProabilityScores[word][0] not in wordScoreDictionary.keys()):
            wordScoreDictionary[wordMaxProabilityScores[word][0]] = [word]
        else:
            wordScoreDictionary[wordMaxProabilityScores[word][0]].append(word)

    # now print these in ascending order
    idx = 0
    for wordScore in sorted(wordScoreDictionary.keys(),reverse=True):
        for word in wordScoreDictionary[wordScore]:
            print_message(str(++idx) + ". " + wordMaxProabilityScores[word][1]._type_ + " " + word + " " + str(wordMaxProabilityScores[word][0]) + " (" + str(wordMaxProabilityScores[word][0]) + ")")

    # use wordMaxProabilityScores, bestGuessWord and maxScoreObj
    '''
    for idx, word in enumerate(wordMaxProabilityScores):
        print_message(str(idx+1) + ". " + wordMaxProabilityScores[word][1]._type_ + " " + word + " " + str(wordMaxProabilityScores[word][0]) + " (" + str(wordMaxProabilityScores[word][0]) + ")")
    '''
    
    # print new line for cleanliness
    print_message(" ")

    # print the conclusion
    print_message("This is the " + bestGuessObj._type_ + " " + bestGuessWord)
    print(ctime())

# This function used to send questions for the user to answer while using
# Active Learning.
def learn_example(cv_image, message, msg_id, al_framework, print_message, 
                  ask_question, end_exchange):
    print_message("Asking Question....")
    [msg_id, msg_str] = al_framework.add_word_example_pair(msg_id, cv_image, message)
    if msg_id = 6:
        print_message("End Conversation....")
        print_message(msg_str)
        end_exchange(msg_str)
    else:
        ask_question(msg_id, msg_str)
    
# This function is to start the conversation with the Robot while using Active Learning.
def start_conversation(cv_image, message, al_framework, print_message, ask_question, end_exchange):
    print("Starting a Conversation....")
    print_message("Starting a Conversation....")
    [msg_id, msg_str] = al_framework.add_word_example_pair(0, cv_image, "")
    if msg_id = 6:
        print_message("End Conversation....")
        print_message(msg_str)
        end_exchange(msg_str)
    else:
        ask_question(msg_id, msg_str)
