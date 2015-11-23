'''
@Description: This code attempts to model the active learning architecture which extends the joint architecture model presented in A Joint Model of Language and Perception for Grounded Attribute Learning
                          Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf

This code is extended/modified from Joint Architecture developed by Karan Budhraja
'''

__author__      = "Nisha Pillai"
__email__       = "npillai1@umbc.edu"

# libraries used
from abc import ABCMeta
from abc import abstractmethod
from image.color import detectColor as dc
from image.shape import shapeUtils as su

# ObjWord constants
# possible example polarities
OW_POSITIVE_POLARITY = "+"
OW_NEGATIVE_POLARITY = "-"
# comparison threshold values to consider items as duplicates
OW_COLOR_DUPLICATE_THRESHOLD = 1
OW_SHAPE_DUPLICATE_THRESHOLD = 1
# comparison threshold values to consider items as matched or mismatched
# this is for classification purposes
OW_COLOR_POSITIVE_EXAMPLE_THRESHOLD = 0.8
OW_COLOR_NEGATIVE_EXAMPLE_THRESHOLD = 0.2
OW_COLOR_CLASSIFICATION_THRESHOLD = 0.9
OW_SHAPE_POSITIVE_EXAMPLE_THRESHOLD = 0.8
OW_SHAPE_NEGATIVE_EXAMPLE_THRESHOLD = 0.2
OW_SHAPE_CLASSIFICATION_THESHOLD = 0.9

# JointModel constants
JM_GUESS_SCORE_THRESHOLD = 0.8

BEST_SYN_SCORE = 0.7

AL_QUESTIONNAIRE = {1 : "Which Color & Shape is this?", 2 : "Is this a synonym of "}

'''
class created corresponding to classifier for each word
the name is derived from the notation obj-color and obj-shape used in the paper
each object corresponds to a word
each object corresponds to examples related to that word. examples may be positive or negative
'''
class ObjWord:

	# enable creation of abstract methods
	__metaclass__ = ABCMeta

	# creating an object corresponding to a new word
	# word: string
	# example: image
	# example polairty: global definition (constant)
	def __init__(self, word, example, examplePolarity):
		self.word = word
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# each derived class must define its item comparison method
	# this is used to compare two items on different features
	# the features are specified by the derived class 

	# compare two items and get a score
	# item 1: image corresponding to word 
	# item 2: image corresponding to word
	@abstractmethod
	def compare_items(self, item1, item2):
		pass

	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_duplicate_threshold(self):
		pass

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_positive_example_threshold(self):
		pass

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_negative_example_threshold(self):
		pass

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_classification_threshold(self):
		pass

	# check for duplicates
	# item 1: image corresponding to word
	# item 2: image corresponding to word
	def is_equal(self, item1, item2):
		if(compare_items(item1, item2) >= self.get_duplicate_threshold()):
			return True		
		else:
			return False

	# each derived class must define its example comparison method
	# ignore this if duplicate examples are not to be filtered
	# example: candidate example 
	# existing examples: list of known examples
	def is_known_example(self, example, existingExamples):
		# initial assumption
		isKnown = False		

		# check for any match
		# more convoluted but faster this way because no if condition
		for existingExample in existingExamples:
			isKnown = isKnown or is_equal(example, existingExample)

		# return answer
		return isKnown

	# add a new example
	# example: image 
	# example polairty: global definition (constant)
	def add_example(self, example, examplePolarity):
		if(examplePolarity == OW_POSITIVE_POLARITY):
			# positive example
			if(self.is_known_example(example, self.positiveExamples) == False):
				self.positiveExamples.append(example)
		elif(examplePolarity == OW_NEGATIVE_POLARITY):
			# negative example
			if(self.is_known_example(example, self.negativeExamples) == False):
				self.negativeExamples.append(example)
		else:
			# neither positive nor negative example. ignore
			pass

	# get classification (probability) score for this classifier based on known examples
	def calculate_probability_score(self, example, additionalPositiveExamples=[], additionalNegatveExamples=[]):

		# add additional positive examples if any
		positiveExamples = self.positiveExamples + additionalPositiveExamples
	
		# add additional negative examples if any
		negativeExamples = self.negativeExamples + additionalNegatveExamples

		# check against this classifier
		correctExamples = 0

		# check against positive examples
		for positiveExample in positiveExamples:
			if(self.compare_items(example, positiveExample) > self.get_positive_example_threshold):
				correctExamples += 1

		# check against negative examples
		for negativeExample in negativeExamples:
			if(self.compare_items(example, negativeExample) < self.get_negative_example_threshold()):
				correctExamples += 1

		# compute p(example|word)
		totalExamples = len(positiveExamples) + len(negativeExamples)
		pExampleGivenWord = correctExamples/float(totalExamples)

		# p(word) = totalExamples / examples over all worlds
		# the denominator is constant for all word scores. ignore it
		# consider non-normalized version of p(word) to calculate score
		probabilityScore = pExampleGivenWord*totalExamples

		# return the score
		return probabilityScore

'''
initialization: each word may correspond to a color, shape or synonym
null hypothesis is not a class. it is a conclusion if no class matches
'''
class ObjColor(ObjWord):
	
	# compare two items in terms of colors and get a score
	# item 1: image 
	# item 2: image 
	def compare_items(self, item1, item2):
		# exctract colors from two images and compare them		
		dc.compare_items(item1['color'], item2['color'])

	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	def get_duplicate_threshold(self):
		return OW_COLOR_DUPLICATE_THRESHOLD

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	def get_positive_example_threshold(self):
		return OW_COLOR_POSITIVE_EXAMPLE_THRESHOLD

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	def get_negative_example_threshold(self):
		return OW_COLOR_NEGATIVE_EXAMPLE_THRESHOLD

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	def get_classification_threshold(self):
		return OW_COLOR_CLASSIFICATION_THRESHOLD

class ObjShape(ObjWord):

	# compare two items in terms of shape and get a score
	# item 1: image 
	# item 2: image
	def compare_items(self, item1, item2):
		# exctract shapes from two images and compare them		
		su.compare_items(item1['shape'], item2['shape'])
		
	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	def get_duplicate_threshold(self):
		return OW_SHAPE_DUPLICATE_THRESHOLD

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	def get_positive_example_threshold(self):
		return OW_SHAPE_POSITIVE_EXAMPLE_THRESHOLD

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	def get_negative_example_threshold(self):
		return OW_SHAPE_NEGATIVE_EXAMPLE_THRESHOLD

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	def get_classification_threshold(self):
		return OW_SHAPE_CLASSIFICATION_THESHOLD

class ObjSynonymColor(ObjWord):

	# re-uses some of ObjWord but not as much as Color and Shape do
	# only re-uses example addition code

	# creating a word as a synonym for another word
	# word: string
	# synonym: string
	# we do not use the other constructor
	# try to use @override
	def __init__(self, word, synonym, example, examplePolarity):
		self.word = word
		self.synonym = synonym
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# dummy implementations of abstract methods
	# needed to instantiate ObjSynonym
	# not used by ObjSynonym objects
	def compare_items(self, item1, item2):
		pass
	def get_duplicate_threshold(self):
		pass
	def get_positive_example_threshold(self):
		pass
	def get_negative_example_threshold(self):
		pass
	def get_classification_threshold(self):
		pass

class ObjSynonymShape(ObjWord):

	# re-uses some of ObjWord but not as much as Color and Shape do
	# only re-uses example addition code

	# creating a word as a synonym for another word
	# word: string
	# synonym: string
	# we do not use the other constructor
	# try to use @override
	def __init__(self, word, synonym, example, examplePolarity):
		self.word = word
		self.synonym = synonym
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# dummy implementations of abstract methods
	# needed to instantiate ObjSynonym
	# not used by ObjSynonym objects
	def compare_items(self, item1, item2):
		pass
	def get_duplicate_threshold(self):
		pass
	def get_positive_example_threshold(self):
		pass
	def get_negative_example_threshold(self):
		pass
	def get_classification_threshold(self):
		pass


'''
joint model class which will contain many word based classifiers
for synonyms, two classifiers may have large confidence in a situation. merging of examples should then be considered
we will not deal with synonyms for now
since the model is data driven, we do not need to train classifiers. we simply store data an calculate at test time
'''
class ALUniRobotDrivenModel:

        # creating an empty model
        def __init__(self):
                # known words and their classifiers
                self.knownWords = {}
                self.minimumGuessScore = JM_GUESS_SCORE_THRESHOLD
              
                self.questionFlag = 0      
                self.synCandidates = []
                self.wordInQuestion = ""  
                self.newWord = ""
   

        def prepare_questions(self,example):
           self.questionFlag = 1
           for word in self.knownWords.keys():
              [isWordExampleConsistent, probabilityScores] = self.classify_word_example(word, example)
              print (word,isWordExampleConsistent, probabilityScores)
              maxScore = max(probabilityScores.values())
	      if(maxScore >= BEST_SYN_SCORE):
                 self.synCandidates.append(word)
        

        # add a word-example pair to the model
        # word: string
        # example: image
        # example polairty: global definition (constant)
        def add_word_example(self,word, example, examplePolarity):
        
           currKnownWords = self.knownWords.keys()
           # check if it is a new word
           if(word not in self.knownWords.keys()):
              # new word. add possibly associated classifiers
              # limited to initialization
              self.knownWords[word] = []
              self.knownWords[word].append(ObjColor(word, example, examplePolarity))
              self.knownWords[word].append(ObjShape(word, example, examplePolarity))

              # add possibilities of being a synonym
              # this will not contain redundant information like (a b), (a c) and (b c)
              # this is because syonyms are added in order
              for knownWord in currKnownWords:
                 # word may be a synonym of knownWord
                 # when classifying, synonyms are checked for all classifier types
                 # e.g. color, shape
                 self.knownWords[word].append(ObjSynonymColor(word, knownWord, example, examplePolarity))
                 self.knownWords[word].append(ObjSynonymShape(word, knownWord, example, examplePolarity))
           else:
              # known word. just add the example
              # add in all word objects (where adding an example is possible)                 
              for classifier in self.knownWords[word]:
                 # assume all types to qualify for example addition
                 classifier.add_example(example, examplePolarity)

       

        # add a word-example pair to the model
        # word: string
        # example: image
        # example polairty: global definition (constant)
        def add_word_example_pair(self,word, example, examplePolarity):
                #Default Return sentence
                sentenceInReturn = "Okay"

                if word and word.strip() : 
                   self.prepare_questions(example)
                   sentenceInReturn = AL_QUESTIONNAIRE[1]
                else :
                   if self.questionFlag == 1 :
                      self.add_word_example(self,word, example, examplePolarity)
                      self.synCandidates.remove(word)
                      self.newWord = word
                   else:
                      word = word.lower()
                      
                      if word == "yes" or word == "y" :
 
                         word1 = self.wordInQuestion
                         word2 = self.newWord
                         self.knownWords[word1].append(ObjSynonymColor(word1, word2, example, examplePolarity))
                         self.knownWords[word1].append(ObjSynonymShape(word1, word2, example, examplePolarity))
                   
                    word1 = self.wordInQuestion
                    self.synCandidates.remove(word1)

                    if len(self.synCandidates) == 0:
                       self.questionFlag = 0
                       self.synCandidates = []
                       self.wordInQuestion = ""
                       self.newWord = ""
                    else :
                       self.questionFlag = 2
                       word2 = self.synCandidates.pop()
                       self.wordInQuestion = word2
                       sentenceInReturn =  AL_QUESTIONNAIRE[2] + word2 + " ? "

                 return sentenceInReturn

	'''
	experiment: trained attributes
	'''
	# classify a word with corresponding example and get positive or negative confirmation
	# if the classifier is confident, then we don't know
	# e.g. "is this green?"
	# word: string
	# example: image
	# classificationScores: dictionary of classification scores per classifier
	def classify_word_example(self, word, example):

		probabilityScores = {}

		# check all classifiers related to this word
		for classifier in self.knownWords[word]:
			if("Synonym" not in str(type(classifier))):
				# use non-synonym classifiers directly
				probabilityScore = classifier.calculate_probability_score(example)
			else:
				# use synonym classifiers indirectly
				# add positive and negative examples known for the word but not the synonym
				# we do not care about the return values for recursive calls
				# we only want to populate probabilityScores in each recursion
	
				if("Color" in str(type(classifier))):
					searchType = "ObjColor"					
				elif("Shape" in str(type(classifier))):
					searchType = "ObjShape"					
				else:
					# should never come here for given initialization
					pass

				for synonymClassifier in self.knownWords[classifier.synonym]:
					if(searchType in str(type(synonymClassifier))):
						# will only enter this once
						# break is efficient but not neccessary
						synonymClassifierObj = synonymClassifier
						break;

				probabilityScore = synonymClassifierObj.calculate_probability_score(example, classifier.positiveExamples, classifier.negativeExamples)
			
			# add score to classification scores
			probabilityScores[classifier] = probabilityScore

		# now we have accumulated all the scores
		# check if any of the scores exceed the threshold
		# initially assume inconsistency
		isWordExampleConsistent = False

		# compare for positive
		# more convoluted but faster this way because no if condition
		for classifier in probabilityScores.keys():
			isWordExampleConsistent = isWordExampleConsistent or (probabilityScores[classifier] >= classifier.get_classification_threshold())

		# return the consistency decision and probability scores
		return [isWordExampleConsistent, probabilityScores]

	'''
	experiment: novel scene
	'''	
	# classify a new example and get corresponding word
	# if no classifier is confident, then it is a new category of example. we do not handle this right now
	# e.g. "what is this?"
	# e.g. of bayes' rule: p(cube|example) = p(example|cube) * p(cube) / p(example)
	# p(example) is constant across all word classifications and can be ignored when comparing them
    # p(example|cube): the fraction of examples in "cube" which matched the current example
    # p(cube): the fraction of examples under "cube" relative to examples over all known words
	# p(cube) = totalExamples of cube / total examples of all words
	# the denominator is constant for all word scores. ignore it
	# consider non-normalized version of p(cube) to calculate score
	# example: image
	def classify_example(self, example):

		# check against each known word
		# maximum probability score data corresponding to a word		
		wordMaxProabilityScores = {}
		# all probability score data corresponding to a word
		wordProbabilityScores = {}

		# maintain best guess
		bestGuessWord = ""
		bestGuessMaxScore = 0
	
		# calculate word probability scores
		# check all associated classifiers
		for word in self.knownWords.keys():
			[isWordExampleConsistent, probabilityScores] = self.classify_word_example(word, example)
                        print (word,isWordExampleConsistent, probabilityScores)
			# select maximum score corresponding to best interpretation			
			maxScore = max(probabilityScores.values())			

			# add to probability scores
			wordMaxProabilityScores[word] = maxScore
			wordProbabilityScores[word] = [isWordExampleConsistent, probabilityScores]

			# update best guess if possible
			if(maxScore > bestGuessMaxScore):
				bestGuessWord = word
				bestGuessMaxScore = maxScore

		# guess confidence
		# initial assumption
		isConfidentGuess = False

		if(bestGuessMaxScore >= self.minimumGuessScore):
			isConfidentGuess = True
		
		# return everything known to man
		return [bestGuessWord, isConfidentGuess, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores]
		
	'''
	experiment: novel english
	'''	
	# TODO
	def classify_word(self, word):
		pass

'''
main function
'''
def main():
	
	pass

main()
