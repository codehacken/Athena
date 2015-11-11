import nltk
from nltk.tokenize import PunktSentenceTokenizer
from nltk.chunk.regexp import RegexpChunkRule

class LanguageModule:
  tokenized=[]
  def __init__(self,sentence):
     f = open('training_data', 'r')
     train_text=f.read()
     #data=open('data2','r')
     #test_data=data.read()
     custom_sent_tokenizer = PunktSentenceTokenizer(train_text)
     self.tokenized = custom_sent_tokenizer.tokenize(sentence)

  def process_content(self):
     keywordList=[]
     rule1="""Chunk: {<DT>?<JJ>?<NN>}"""
     rule2="""Chunk:{<DT><NN>}"""
     rule3="""Chunk:{<RB><JJ><NN>?}"""
     rule4="""Chunk:{<NN><JJ>}"""
     rule5="""Chunk:{<VBN><IN><NN>}"""
     rule5="""Chunk:}<NN><IN>{"""
     rule6="""Chunk:{<CD><JJ>*<NN><VBN><NN>}"""
     rule7="""Chunk:{<DT><NN>}"""
     rule8="""Chunk:{<IN>?<RB><IN>?<JJ>}"""
     rule9="""Chunk:{<VBZ><JJ>}"""
     rule10="""Chunk:{<RB><DT>?<JJ>*<NN>}"""
     rule11="""Chunk:{<VBN><IN>?<NN>}"""
     rule12="""Chunk:{<WDP|WP|VBZ><DT>?<JJ>?<NN>?}"""
     rule13="""Chunk:{<WP><NN>}"""
     rule14="""Chunk:{<RB><VB>}"""
     rule16="""Chunk:{<NN><IN><NN>}"""
     # rule15="""Chunk:{<PRP$><JJ>?<NN>?}"""
     RuleList=[rule1,rule2,rule3,rule4,rule5,rule6,rule7,rule8,rule9,
               rule10,rule11,rule12,rule14,rule16]
     try:
         for i in self.tokenized[:5]:
             words = nltk.word_tokenize(i)
             tagged = nltk.pos_tag(words)
       #      print tagged
             for rule in RuleList:
	         parser=nltk.RegexpParser(rule)
                 result = parser.parse(tagged)
                 subtreeList=[]
                 for subtree in result.subtrees(filter=lambda t: t.label() == 'Chunk'):
		    for leaves in subtree:
                        keywordList.append(leaves)
                  
     except Exception as e:
        print(str(e))
     keyword= [item for item in keywordList if item[1] not in ['IN','VBZ'] 
             and item[0] not in ['color', 'three','dimensional','solid','called',
                                 'figure','geometric','picture','depicts','surface'
                                  ,'equal','length','shape']]
     negative_adverbs=[keyword[keyword.index(word)+1][0] for word in keyword if word[0] in ['not','Not']]
     negative_nouns= [keyword[keyword.index(word)+2][0] for word in keyword if word[0] in ['not','Not'] 
                      and keyword[keyword.index(word)+2][1] in ['NN'] ]
     negative_examples=negative_adverbs+negative_nouns
     positive_examples=[positive_word for positive_word in keyword if  positive_word[0] not in [item1 for item1 in negative_examples]]
     positive_examples=[positive_example[0] for positive_example in positive_examples 
                        if positive_example[0] not in['not','Not','a','the'] and positive_example[1] not in ['DT']]
     positive_examples = list(set(positive_examples))
     negative_examples = list(set(negative_examples))
     
     return_data=[positive_examples,negative_examples]
     return return_data 
     #print(positive_examples) 
     #print(negative_examples)
