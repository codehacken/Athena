'''
@Description: This code attempts to model the joint learning architecture presented in A Joint Model of Language and Perception for Grounded Attribute Learning
			  Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf
'''

__author__	= "Karan K. Budhraja"
__email__	= "karanb1@umbc.edu"

# libraries used
from abc import ABCMeta
from abc import abstractmethod
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su

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
OW_COLOR_CLASSIFICATION_THESHOLD = 0.9
OW_SHAPE_POSITIVE_EXAMPLE_THRESHOLD = 0.8
OW_SHAPE_NEGATIVE_EXAMPLE_THRESHOLD = 0.2
OW_SHAPE_CLASSIFICATION_THESHOLD = 0.9

# JointModel constants
JM_GUESS_SCORE_THRESHOLD = 0.8

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
			if(is_known_example(example, positiveExamples) == False):
				self.positiveExamples.append(example)
		elif(examplePolarity == OW_NEGATIVE_POLARITY):
			# negative example
			if(is_known_example(example, negativeExamples) == False):
				self.negativeExamples.append(example)
		else:
			# neither positive nor negative example. ignore
			pass

	# get classification (probability) score for this classifier based on known examples
	def calculate_probability_score(self, example, additionalPositiveExamples=[], additionalNegatveExamples=[]):

		# add additional positive examples if any
		positiveExamples = self.positiveExamples + additionalPositiveExamples
	
		# add additional negative examples if any
		negatveExamples = self.negativeExamples + additionalNegatveExamples

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
		dc.compare_items(item1['shape'], item2['shape'])

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
		return OW_COLOR_CLASSIFICATION_THESHOLD

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

class ObjSynonym(ObjWord):

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
class JointModel:

	# creating an empty model
	def __init__(self):
		# known words and their classifiers
		self.knownWords = {}
		self.minimumGuessScore = JM_GUESS_SCORE_THRESHOLD

	# add a word-example pair to the model
	# word: string
	# example: image
	# example polairty: global definition (constant)
	def add_word_example_pair(self, word, example, examplePolarity):
		# check if it is a new word
		if(word not in knownWords.keys()):
			# new word. add possibly associated classifiers
			# limited to initialization
			knownWords[word] = []
			knownWords[word].append(ObjColor(word, example, examplePolarity))
			knownWords[word].append(ObjShape(word, example, examplePolarity))

			# add possibilities of being a synonym
			# this will not contain redundant information like (a b), (a c) and (b c)
			# this is because syonyms are added in order
			for knownWord in knownWords.keys():
				# word may be a synonym of knownWord
				# when classifying, synonyms are checked for all classifier types
				# e.g. color, shape
				knownWords[word].append(ObjSynonym(word, knownWord, example, examplePolarity))
		else:
			# known word. just add the example
			# add in all word objects (where adding an example is possible)			
			for classifier in knownWords[word]:
				# assume all types to qualify for example addition
				classifier.add_example(example, examplePolarity)

	'''
	experiment: trained attributes
	'''
	# classify a word with corresponding example and get positive or negative confirmation
	# if the classifier is confident, then we don't know
	# e.g. "is this green?"
	# word: string
	# example: image
	# classificationScores: dictionary of classification scores per classifier
	def classify_word_example(self, word, example, probabilityScores = {}, additionalPositiveExamples=[], additionalNegatveExamples=[]):

		# check all classifiers related to this word
		for classifier in knownWords[word]:
			if(type(classifier) is not ObjSynonym):
				# use non-synonym classifiers directly
				probabilityScore = classifier.calculate_probability_score(example, additionalPositiveExamples, additionalNegatveExamples)

				# add score to classification scores
				classificationScores[classifier] = probabilityScore
			else:
				# use synonym classifiers indirectly
				# add positive and negative examples known for the word but not the synonym
				# we do not care about the return values for recursive calls
				# we only want to populate probabilityScores in each recursion
				classify_word_example(classifier.synonym, example, probabilityScores, classifier.positiveExamples, classifier.negativeExamples)

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
		wordMaxProabilityScores = {}
		wordProbabilityScores = {}

		# maintain best guess
		bestGuessWord = ""
		bestGuessMaxScore = 0

		# calculate word probability scores
		# check all associated classifiers
		for word in knownWords.keys():
			[isWordExampleConsistent, probabilityScores] = knwonWords[word].classify_word_example(word, example)
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