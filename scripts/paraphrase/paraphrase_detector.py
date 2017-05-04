import numpy as np
import skipthoughts
import scipy.spatial.distance as sd
#uncomment the following to load the skipthoughts model here
#import skipthoughts

class Paraphrase_detector:

	# pass in a skipthought encoder or uncomment required code to create an encoder here
	def __init__(self, skipthought_encoder):

		self.objects = {}
		self.places = {}
		self.poses = {}
		print "loading paraphrase detector"

		# These are the built-in command primitives for poses. The user will  
		# add pose locations to be used with the commands
		self.pose_sentences = ['enact pose ', 'move hand to ']

		# These are the built-in command primitives for objects. The user will  
		# add object names to be used with the commands
		self.object_sentences = ['grasp object ', 'place object ', 'put down object ', 'pick up object ']

		# These are the built-in command primitives for places. The user will  
		# add place names to be used with the commands
		self.place_sentences = ['go to ', 'go ']


		# Uncomment these if the skipthoughts model is not loaded elsewere
		#self.model = skipthoughts.load_model()
		#self.encoder = skipthoughts.Encoder(self.model)

		# use the skipthought encoder passed in
		self.encoder = skipthought_encoder

		# this stores all command sentences which have been created by the user's input
		self.active_sentences = []

		# these are commands which cannot be processed by the paraphrase detector
		# because they contain specific numbers in the input and those would be lost
		# in a paraphrase detection 
		self.start_words_to_skip = ['move by', 'follow me for']


	# Let the paraphrase detector know what sentences are in the vocabulary. Add one of 
	# 3 kinds of sentences: pose, object, or place. new_token should be whatever new ID of that kind should be added
	# for example: add_sentence('pose', 'home') would add a new pose called home that could be called by any 
	# of the pose sentences in self.pose_sentences defined above
	def add_sentence(self, sentence_type, new_token):
		if sentence_type == 'pose':
			for sentence in self.pose_sentences:
				new_sentence = [sentence + new_token]
				sentence_vector = self.encoder.encode(new_sentence)
				self.active_sentences.append((sentence_vector, new_sentence))

		elif sentence_type == 'object':
			for sentence in self.object_sentences:
				new_sentence = [sentence + new_token]
				sentence_vector = self.encoder.encode(new_sentence)
				self.active_sentences.append((sentence_vector, new_sentence))

		elif sentence_type == 'place':
			for sentence in self.place_sentences:
				new_sentence = [sentence + new_token]
				sentence_vector = self.encoder.encode(new_sentence)
				self.active_sentences.append((sentence_vector, new_sentence))
		else:
			print "invalid input sentence type"

	# use this to add an entirely new command
	def add_command(self,command):
		sentence_vector = self.encoder.encode(command)
		self.active_sentences.append((sentence_vector, command))

	#For internal use only. 
	#For paraphrase detection, use by check_task_list_for_paraphrases
	def get_closest_sentence(self, input_sentence, distance_metric="cosine"):
		sentence_distances = []
		input_sentence_vector = self.encoder.encode([input_sentence])

		for sentence in self.active_sentences:
			distance = sd.cdist(sentence[0], input_sentence_vector, distance_metric)
			sentence_distances.append((distance, sentence[1]))

		sentence_distances.sort()

		return sentence_distances[0][1][0]


	# use this to detect paraphrases
	def check_task_list_for_paraphrases(self, input_string):
		strings = input_string.split(" and then ")
		number_of_and_thens = len(strings) - 1

		string_to_return = ""

		for sentence in strings: 
			already_done = False

			# check all starts_with_words_to_skip before looking for paraphrase
			for skip_words in self.start_words_to_skip:
				if sentence.startswith(skip_words):
					if number_of_and_thens > 0:
						string_to_return = string_to_return + sentence + " and then "
						number_of_and_thens -= 1
						already_done = True
					else: 
						string_to_return = string_to_return + sentence
						already_done = True


			if not already_done: 
				if number_of_and_thens > 0:
					parsable_sentence = self.get_closest_sentence(sentence)
					string_to_return = string_to_return + parsable_sentence + " and then "
					number_of_and_thens -= 1

				else: 
					parsable_sentence = self.get_closest_sentence(sentence)
					string_to_return = string_to_return + parsable_sentence

		return string_to_return



	#Add a new start word to skip when doing paraphrase detection
	def add_start_word_to_skip(self, new_skip_word):
		self.start_words_to_skip.append(new_skip_word)


	# get all active command sentences and their associated vector representations
	def get_active_sentences(self):
		return self.active_sentences


	# load up some objects, poses, and commands
	# use for testing or change for use in a regular environment
	def instantiate_test_env(self):
		self.add_sentence('object', 'mug')
		self.add_sentence('pose', 'home')
		self.add_sentence('object', 'bottle')
		self.add_sentence('object', 'block')
		self.add_sentence('place', 'base')
		self.add_sentence('place', 'kitchen')
