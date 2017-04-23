import numpy as np
#import skipthoughts
import scipy.spatial.distance as sd

class Paraphrase_detector:


	def __init__(self, skipthought_encoder):

		self.objects = {}
		self.places = {}
		self.poses = {}
		print "loading paraphrase detector"
		self.pose_sentences = ['enact pose ', 'move hand to ']

		self.object_sentences = ['grasp object ', 'place object ', 'put down object ', 'pick up object ']

		self.place_sentences = ['go to ', 'go ']

		#self.model = skipthoughts.load_model()

		#self.encoder = skipthoughts.Encoder(self.model)
		self.encoder = skipthought_encoder
		self.active_sentences = []

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




	def get_closest_sentence(self, input_sentence, distance_metric="cosine"):
		sentence_distances = []
		input_sentence_vector = self.encoder.encode([input_sentence])

		for sentence in self.active_sentences:
			distance = sd.cdist(sentence[0], input_sentence_vector, distance_metric)
			sentence_distances.append((distance, sentence[1]))

		sentence_distances.sort()

		return sentence_distances[0][1][0]


	def check_task_list_for_paraphrases(self, input_string):
		strings = input_string.split(" and then ")
		number_of_and_thens = len(strings) - 1

		string_to_return = ""

		for sentence in strings: 
			if sentence.startswith("move by"):
				if number_of_and_thens > 0:
					string_to_return = string_to_return + sentence + " and then "
					number_of_and_thens -= 1
				else: 
					string_to_return = string_to_return + sentence
			else: 
				if number_of_and_thens > 0:
					parsable_sentence = self.get_closest_sentence(sentence)
					string_to_return = string_to_return + parsable_sentence + " and then "
					number_of_and_thens -= 1

				else: 
					parsable_sentence = self.get_closest_sentence(sentence)
					string_to_return = string_to_return + parsable_sentence

		return string_to_return




	def get_active_sentences(self):
		return self.active_sentences

	def instantiate_test_env(self):
		self.add_sentence('object', 'mug')
		self.add_sentence('pose', 'home')
		self.add_sentence('object', 'bottle')
		self.add_sentence('object', 'block')
		self.add_sentence('place', 'base')
		self.add_sentence('place', 'kitchen')