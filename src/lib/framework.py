'''
@Description: This code attempts to model the joint learning architecture presented in A Joint Model of Language and Perception for Grounded Attribute Learning
			  Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf
'''

__author__	= "Karan K. Budhraja"
__email__	= "karanb1@umbc.edu"

# possible example polarities
POSITIVE_POLARITY = "+"
NEGATIVE_POLARITY = "-"

'''
class created corresponding to classifier for each word
the name is derived from the notation obj-color and obj-shape used in the paper
each object corresponds to a word
each object corresponds to examples related to that word. examples may be positive or negative
'''
class ObjWord:

	# creating an object corresponding to a new word
	# word: string
	# example: image. TODO: duplicate check may not work as is for images 
	# example polairty: global definition (constant)
	def __init__(self, word, example, examplePolarity):
		self.word = word
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# add a new example
	# example: image. TODO: duplicate check may not work as is for images 
	# example polairty: global definition (constant)
	def add_example(self, example, examplePolarity):
		if(examplePolarity == POSITIVE_POLARITY):
			# positive example
			if(example not in positiveExamples):
				self.positiveExamples.append(example)
		elif(examplePolarity == NEGATIVE_POLARITY):
			# negative example
			if(example not in negativeExamples):
				self.negativeExamples.append(example)
		else:
			# neither positive nor negative example. ignore
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
		self.knownWords = {}

	# add a word-example pair to the model
	# word: string
	# example: image
	# example polairty: global definition (constant)
	def add_word(self, word, example, examplePolarity):
		# check if it is a new word
		if(word not in knownWords.keys()):
			# new word
			knownWords[word] = ObjWord(word, example, examplePolarity)
		else:
			# known word. just add the example
			knownWords[word].add_example(example, examplePolarity)

	# TODO: all of the functions below. what the joint model is used for
	# currently using a bayesian classifier. not sure if the model in the paper is this simple

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
	# example: image
	def classify_example(self, example):

		# check against each known word
		pExampleGivenWord = {}
		pWord = {}
		wordExampleCount = []
		wordProbabilityScore = {}

		# calculate word probability scores
		totalWordExampleCount = sum(wordExampleCount)
		for word in knownWords.keys():
			wordProbabilityScore[word] = classify_word_example(word, example)

		# sort probability scores in decending order
		wordProbabilityScores = wordProbabilityScore.values()
		sortedWordProbabilityScores = sorted(wordProbabilityScores, reverse=True)

		# reverse map the wordProbabilityScore dictionary
		probabilityScoreDictionary = {}
		for word in wordProbabilityScore.keys():		
			wordProbabilityScoreValue = wordProbabilityScore[word]

			# check if already in dictionary
			if(wordProbabilityScoreValue in probabilityScoreDictionary.keys()):
				# append to list				
				probabilityScoreDictionary[wordProbabilityScoreValue].append(word)
			else:
				# create a new entry
				probabilityScoreDictionary[wordProbabilityScoreValue] = [word]

		# best word is word with highest probability score
		return probabilityScoreDictionary

	'''
	experiment: trained attributes
	'''	
	# classify a word with corresponding example and get positive or negative confirmation
	# if the classifier is confident, then we don't know
	# e.g. "is this green?"
	# word: string
	# example: image
	def classify_word_example(self, word, example):

		# check against this word
		correctExamples = 0

		# TODO: checking against examples will be confidence based using image matching
		# check against positive examples
		positiveExamples = knownWords[word].positiveExamples
		for positiveExample in positiveExamples:
			if(example == positiveExample):
				correctExamples += 1

		# check against negative examples
		negativeExamples = knownWords[word].negativeExamples
		for negativeExample in negativeExamples:
			if(example != negativeExample):
				correctExamples += 1

		# compute p(example|word)
		pExampleGivenWord = correctExamples/float(totalExamples)

		# calculate pWord
		totalWordExampleCount = 0
		for knownWord in knownWords.keys():
			totalWordExampleCount += len(knownWords[knownWord].positiveExamples) + len(knownWords[knownWord].negativeExamples)

		pWord = (len(knownWords[word].positiveExamples) + len(knownWords[word].negativeExamples)) / float(totalWordExampleCount)

		# calculate word probability score
		wordProbabilityScore = pExampleGivenWord*pWord

		# TODO: are we confident about this score?

		# return the score. used by other functions
		return wordProbabilityScore

	'''
	experiment: novel english
	'''	
	def classify_word(self, word):
		pass

'''
main function
'''
def main():
	
	pass





main()
