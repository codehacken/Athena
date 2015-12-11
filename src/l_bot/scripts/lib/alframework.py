'''
@Description: This code attempts to model the active learning architecture which extends the joint architecture model presented in A Joint Model of Language and Perception for Grounded Attribute Learning
                          Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf

This code is extended/modified from Joint Architecture developed by Karan Budhraja
'''

__author__      = "Nisha Pillai"
__email__       = "npillai1@umbc.edu"

# Image processing Lib.
from lib.image.common import utils
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su

from framework import JointModel as jm
from framework import ObjSynonymColor as ObjColor
from framework import ObjSynonymShape as ObjShape

# import language processing library
from lib.lang.nlp import LanguageModule as lm

# JointModel constants
JM_GUESS_SCORE_THRESHOLD = 0.8

BEST_SYN_SCORE = 0.7
BEST_SIM_SCORE = 0.95
BEST_NEG_SYN_SCORE = 0.4
LEAST_LOW_SCORE    = 0.3

AL_QUESTIONNAIRE = {1 : "What is this?", 
                    2 : "Is this same as ", 
                    3 : "Is this thing like ", 
                    4 : "Is this totally different from ", 
                    5 : "Can you show me a ",
                    6 : "Okay, You May Now Provide next Object..."}

'''
bla bla bla
joint model class which will contain many word based classifiers
for synonyms, two classifiers may have large confidence in a situation. merging of examples should then be considered
we will not deal with synonyms for now
since the model is data driven, we do not need to train classifiers. we simply store data an calculate at test time
'''
class ALUniRobotDrivenModel:

        # creating an empty model if the joint model is not passed to it.
        def __init__(self, joint_model=None):
                # known words and their classifiers
                if joint_model == None:
                    self.jModel = jm()
                else:
                    self.jModel = joint_model
                
                self.questionFlag = 0      
                self.synCandidates = []
                self.simCandidates = []
                self.negSynCandidates = []
                self.wordInQuestion = ""  
                self.newWords = []
                self.counter = 0  
                self.lowConfCandidates = []

        def prepare_questions(self,example):
           for word in self.jModel.knownWords.keys():
              [isWordExampleConsistent, probabilityScores] = self.jModel.classify_word_example(word, example)
              for score in probabilityScores.values():
                 if score >= BEST_SIM_SCORE:
                    self.simCandidates.append(word)
	         elif score >= BEST_SYN_SCORE:
                    self.synCandidates.append(word)
                 elif score <= BEST_NEG_SYN_SCORE:
                    self.negSynCandidates.append(word)
           self.simCandidates = list(set(self.simCandidates))
           self.synCandidates = list(set(self.synCandidates))
           self.negSynCandidates = list(set(self.negSynCandidates))
        
        def prepareLowConfQuestions(self):
           self.lowConfCandidates = []
           probScores = self.jModel.get_known_words()
           totScore = 0.0
           lowScores = []
           for word,score in probScores.items() :
              totScore += score
              if score < LEAST_LOW_SCORE :
                 lowScores.append(word)

           avScore = float(totScore)/float(len(propScores))
           if avScore > 0.5 :
              self.lowConfCandidates = lowScores      
     

        def arRemove(self,candidates,word):
           temp = set(candidates)
           if word in temp:
              while word in candidates: candidates.remove(word)
           return candidates        

        def wordRemovalfromQuestCandidates(self,word):
           self.synCandidates = self.arRemove(self.synCandidates,word)
           self.simCandidates = self.arRemove(self.simCandidates,word)
           self.negSynCandidates = self.arRemove(self.negSynCandidates,word) 

        def wordsAdd(self,words,example,examplePolarity):
           self.newWords = words
           for word in words:
              self.jModel.add_word_example_pair(word, example, examplePolarity)
              self.wordRemovalfromQuestCandidates(word)

        def differentWordAdd(self,word2,example,examplePolarity) :
           self.jModel.add_word_example_pair(word2, example, examplePolarity)

        def wordSynAdd(self,word1,word2,example,examplePolarity) :
           wordClassifiers = self.jModel.knownWords[word1]
           cObj = ObjColor(word1, word2, example, examplePolarity)
           sObj = ObjShape(word1, word2, example, examplePolarity)
           wordClassifiers.append(cObj)
           wordClassifiers.append(sObj)

        def wordAddPositiveExample(self,words,example) :
           word = words[0]
           word = word.lower()
           if word == "yes" or word == "y" :
              examplePolarity = "+"
           else :
              examplePolarity = "-"
           word2 = self.wordInQuestion
           self.differentWordAdd(word2,example,examplePolarity) 
           self.wordRemovalfromQuestCandidates(word2)


        def wordAddPosSynExample(self,words,example) :
           word = words[0]
           word = word.lower()
           word2 = self.wordInQuestion
           if word == "yes" or word == "y" :
              examplePolarity = "+"
           else :
               examplePolarity = "-"

           for word1 in self.newWords :
              self.wordSynAdd(word1,word2,example,examplePolarity)
              
           self.wordRemovalfromQuestCandidates(word2)


        def wordNegExample(self,words,example,examplePolarity) :
           word = words[0]
           word = word.lower()
           examplePolarity = "-"
           word2 = self.wordInQuestion
           if word == "yes" or word == "y" :
              for word1 in self.newWords :
                 self.wordSynAdd(word1,word2,example,examplePolarity)

           self.wordRemovalfromQuestCandidates(word2)




        # add a word-example pair to the model
        # word: string
        # example: image
        # example polairty: global definition (constant)
        def add_word_example_pair(self,qType,cv_image,message):
                example = self.processImage(cv_image)
                words = []
                if qType != 1 :
                   words = [message]
                   examplePolarity = "+"
                
                #Default Return sentence
                sentenceInReturn = ""
                wordSize = len(words)
                self.questionFlag = 0
                if qType == 0 :
                   self.counter = self.counter + 1
                   if self.counter % 20 == 0 :
                      self.prepareLowConfQuestions()
                   else :
                      self.lowConfCandidates = []
                   #self.prepare_questions(example)
                   self.questionFlag = 1
                   #if wordSize == 0 :
                   #   self.questionFlag = 1
                   #else :
                   #   self.wordsAdd(words,example,examplePolarity)
                elif qType == 1 :
                   self.prepare_questions(example)
                   [positiveLanguageData, negativeLanguageData] = self.processLanguage(message)
                   if len(positiveLanguageData) > 0 :
                      self.wordsAdd(positiveLanguageData,example,"+")
                   if len(negativeLanguageData) > 0 :
                      self.wordsAdd(negativeLanguageData,example,"-")

                elif qType == 2 :
                   self.wordAddPositiveExample(words,example)
                elif qType == 3 :
                   self.wordAddPosSynExample(words,example)
                elif qType == 4 :
                   self.wordNegExample(words,example,examplePolarity)
                elif qType == 5 :
                   if wordSize > 0 :
                      word = words[0]
                      word = word.lower()
                      if word == 'no' :   # answer is changable according to majority opinion
                         pass
                   word2 = self.wordInQuestion
                   examplePolarity = "+"
                   self.differentWordAdd(word2,example,examplePolarity) 
                   self.wordRemovalfromQuestCandidates(word2)
                
      
                if(self.questionFlag == 1) :
                   sentenceInReturn = AL_QUESTIONNAIRE[self.questionFlag]
                else :
                      ##Need to add fifth question
                   if len(self.synCandidates) == 0 and len(self.simCandidates) == 0 and len(self.negSynCandidates) == 0 and len(self.lowConfCandidates) == 0 :
                         self.wordInQuestion = ""
                         self.newWords = []
                         self.questionFlag = 6
                         sentenceInReturn = AL_QUESTIONNAIRE[self.questionFlag]
                         
                   else :
                      if len(self.simCandidates) > 0 :
                         self.wordInQuestion = self.simCandidates.pop()
                         self.questionFlag = 2
                      elif len(self.synCandidates) > 0 :
                         self.wordInQuestion = self.synCandidates.pop()
                         self.questionFlag = 3
                      elif len(self.negSynCandidates) > 0 :
                         self.wordInQuestion = self.negSynCandidates.pop()
                         self.questionFlag = 4
                      elif len(self.lowConfCandidates) > 0 :
                         self.wordInQuestion = self.lowConfCandidates.pop()
                         self.questionFlag = 5
             
                      sentenceInReturn =  AL_QUESTIONNAIRE[self.questionFlag] + self.wordInQuestion + " ? "

                return [self.questionFlag,sentenceInReturn]


        # This function processes the image received from the kinect.
        @staticmethod
        def processImage(self, cv_image):
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
            
            # Return the image information.
            return imageData

        
        # This function processes the language from the 
        @staticmethod
        def processLanguage(self, message):
            # extract keywords from message
            languageObject = lm(message)
            [positiveLanguageData, negativeLanguageData] = languageObject.process_content()

            # Return the list of words processed.
            return [positiveLanguageData, negativeLanguageData]

'''
main function
'''
def main():
	
	pass

main()
