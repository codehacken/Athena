'''
@Description: This code attempts to model the active learning architecture which extends the joint architecture model presented in A Joint Model of Language and Perception for Grounded Attribute Learning
                          Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf

This code is extended/modified from Joint Architecture developed by Karan Budhraja
'''

__author__      = "Nisha Pillai"
__email__       = "npillai1@umbc.edu"

from framework import JointModel as jm


# JointModel constants
JM_GUESS_SCORE_THRESHOLD = 0.8

BEST_SYN_SCORE = 0.7
BEST_SIM_SCORE = 0.95
BEST_NEG_SYN_SCORE = 0.4

AL_QUESTIONNAIRE = {1 : "What is this?", 2 : "Is this same as ", 3 : "Is this thing like ", 4 : "Is this totally different from ", 5 : "Can you show me a "}

'''
bla bla bla
joint model class which will contain many word based classifiers
for synonyms, two classifiers may have large confidence in a situation. merging of examples should then be considered
we will not deal with synonyms for now
since the model is data driven, we do not need to train classifiers. we simply store data an calculate at test time
'''
class ALUniRobotDrivenModel:

        # creating an empty model
        def __init__(self):
                # known words and their classifiers
                self.jModel = jm()
              
                self.questionFlag = 0      
                self.synCandidates = []
                self.simCandidates = []
                self.negSynCandidates = []
                self.wordInQuestion = ""  
                self.newWords = []
   

        def prepare_questions(self,example):
           for word in self.jModel.knownWords.keys():
              [isWordExampleConsistent, probabilityScores] = self.jModel.classify_word_example(word, example)
              print (word,isWordExampleConsistent, probabilityScores)
              
              for score in probabilityScores.values():
                 if score >= BEST_SIM_SCORE:
                    self.simCandidates.append(word)
	         elif score >= BEST_SYN_SCORE:
                    self.synCandidates.append(word)
                 elif score <= BEST_NEG_SYN_SCORE:
                    self.negSynCandidates.append(word)
        
        
        def wordRemovalfromQuestCandidates(word):
           self.synCandidates.remove(word)
           self.simCandidates.remove(word)
           self.negSynCandidates.remove(word)

        def wordsAdd(self,words,example,examplePolarity):
           self.newWords = words
           for word in words:
              self.jModel.add_word_example(self.jModel,word, example, examplePolarity)
              self.wordRemovalfromQuestCandidates(word)

        def differentWordAdd(self,word1,word2,example,examplePolarity) :
           self.jModel.knownWords[word1].append(self.jModel.ObjColor(word2, example, examplePolarity))
  	   self.jModel.knownWords[word1].append(self.jModel.ObjShape(word2, example, examplePolarity))
           self.jModel.knownWords[word2].append(self.jModel.ObjColor(word1, example, examplePolarity))
           self.jModel.knownWords[word2].append(self.jModel.ObjShape(word1, example, examplePolarity))
           for knownWord in self.jModel.knownWords.keys():
              self.jModel.knownWords[word1].append(self.jModel.ObjSynonym(word2, knownWord, example, examplePolarity))
              self.jModel.knownWords[word2].append(self.jModel.ObjSynonym(word1, knownWord, example, examplePolarity))



        def wordAddPositiveExample(self,words,example,examplePolarity) :
           word = words[0]
           word = word.lower()
           if word == "yes" or word == "y" :
              word2 = self.wordInQuestion
              for word1 in self.newWords :
                 self.differentWordAdd(self,word1,word2,example,examplePolarity) 
           self.wordRemovalfromQuestCandidates(word2)


        def wordAddPosSynExample(self,words,example,examplePolarity) :
           word = words[0]
           word = word.lower()
           if word == "yes" or word == "y" :
              word2 = self.wordInQuestion
              for word1 in self.newWords :
                 self.jModel.knownWords[word1].append(self.jModel.ObjSynonymColor(word1, word2, example, examplePolarity))
                 self.jModel.knownWords[word1].append(self.jModel.ObjSynonymShape(word1, word2, example, examplePolarity))
           else :
              pass
              # add it to negative example
           self.wordRemovalfromQuestCandidates(word2)


        def wordNegExample(self,words,example,examplePolarity) :
           word = words[0]
           word = word.lower()
           if word == "yes" or word == "y" :
              word2 = self.wordInQuestion
              for word1 in self.newWords :
                 # add it to negative example
                 pass
           self.wordRemovalfromQuestCandidates(word2)




        # add a word-example pair to the model
        # word: string
        # example: image
        # example polairty: global definition (constant)
        def add_word_example_pair(self,qType, words, example, examplePolarity):


                #Default Return sentence
                sentenceInReturn = "Okay"
                wordSize = len(words)
                self.questionFlag = 0
                if qType == 0 :
                   self.prepare_questions(example)
                   if wordSize == 0 :
                      self.questionFlag = 1
                   else :
                      self.wordsAdd(self,words,example,examplePolarity)
                elif qType == 1 :
                   self.prepare_questions(example)
                   if wordSize > 0 :
                      self.wordsAdd(self,words,example,examplePolarity)
                elif qType == 2 :
                   self.wordAddPositiveExample(self,words,example,examplePolarity)
                elif qType == 3 :
                   self.wordAddPosSynExample(self,words,example,examplePolarity)
                elif qType == 4 :
                   self.wordNegExample(self,words,example,examplePolarity)
                elif qType == 5 :
                   if wordSize > 0 :
                      word = words[0]
                      word = word.lower()
                      if word == 'no' :   # answer is changable according to majority opinion
                         pass
                   word2 = self.wordInQuestion
                   self.differentWordAdd(self,word2,word2,example,examplePolarity) 
                   self.wordRemovalfromQuestCandidates(word2)
     
                    
                if(self.questionFlag == 1) :
                   sentenceInReturn = AL_QUESTIONNAIRE[self.questionFlag]
                else :
                      ##Need to add fifth question
                   if len(self.synCandidates) == 0 and len(self.simCandidates) == 0 and len(self.negSynCandidates) == 0 :
                      self.wordInQuestion = ""
                      self.newWords = []
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
             
                      sentenceInReturn =  AL_QUESTIONNAIRE[self.questionFlag] + self.wordInQuestion + " ? "

                 return [self.questionFlag,sentenceInReturn]


'''
main function
'''
def main():
	
	pass

main()
