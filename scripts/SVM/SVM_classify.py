#!/usr/bin/env python

# OpenCV2
import cv2

# ROS
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# numpy
import numpy as np
from numpy.linalg import norm

import sys
import argparse

sys.path.insert(0, '../')
from adaptive_filt import *
sys.path.insert(0, 'SVM/')

''' Class to save or load the SVM model. The filetype
can be anything really. YAML is the proper extension.
'''
class StatModel(object):
    def load(self):
    	self.models = [cv2.ml.SVM_load('numbers_model.yaml'), \
    					cv2.ml.SVM_load('letters_model.yaml')]
    def save(self, fn):
        self.models[0].save(fn + 'numbers_model.yaml')
        self.models[1].save(fn + 'letters_model.yaml')

''' SVM classifier. It will load two models: one for number
classification and one for letter classification. The type
of images it will receive depends on the argument passed
when executing the script. Defaults to rgb images.
'''
class SVM(StatModel):
	def __init__(self, C=1, gamma=1, img_type='rgb', ld=False):
		if img_type == 'rgb':
			self.img_type = '8UC1'
		else:
			self.img_type = '16UC1'
		if ld:
			self.load()
		else:
			self.models = [cv2.ml.SVM_create(), cv2.ml.SVM_create()]
			for model in self.models:
				model.setKernel(cv2.ml.SVM_LINEAR)
				model.setType(cv2.ml.SVM_C_SVC)
				model.setGamma(gamma)
				model.setC(C)
			self.train()

		self.pub = rospy.Publisher('asl/letter', std_msgs.msg.String, queue_size=1)
		self.sub = rospy.Subscriber('asl/hands/right', Image, self.callback, queue_size=1)
		self.subType = rospy.Subscriber('asl/hands/type', std_msgs.msg.Bool, self.typeCallback, queue_size=1)
		self.bridge = CvBridge()

		self.ask = True
		self.type = 0 # 0 for numbers and 1 for letters
		self.symbol = ''
		self.image = None

	def callback(self, data):
		if not self.ask:
			return
		self.ask = False
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, self.img_type)
			prediction = self.predict(self.image)
			self.symbol = unichr(int(prediction[0]))

		except CvBridgeError as e:
			print(e)

		self.ask = True

	def typeCallback(self, data):
		self.type = data.data

	def request(self):
		self.pub.publish(self.symbol)

	def test_accuracy(self):
		acc = []
		for i in range(65, 91):
			n = 1
			success = 0
			while(True):
				filename = '../../samples/validation/'
				if self.img_type == '16UC1':
					filename += 'depth/'
				else:
					filename += 'rgb/raw/'
				filename += unichr(i).lower() + '/' + str(n) + '.jpg'
				self.image = cv2.imread(filename, 0)

				if self.image == None:
					acc.append(success / n)
					break

				height, width = 120, 120
				self.image = cv2.resize(self.image, (width, height))
				prediction = self.predict(self.image)
				if unichr((prediction[0])).lower() == unichr(i).lower():
					success += 1.0
				n += 1

		keys = [unichr(key) for key in range(65, 91)]
		return keys, acc

	def predict(self, img, tp=None):
		if tp == None:
			tp = self.type
		return self.models[tp].predict(self.preprocess_hog([img]))[1].ravel()

	# This function returns a numpy array of features. Chosen
	# features are HOG and non-zero pixels in rows and columns
	def preprocess_hog(self, imgs):
		samples = []
		n = 0
		for img in imgs:
			img = adaptive_filter(img)
			gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
			gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)

			mag, ang = cv2.cartToPolar(gx, gy)
			bin_n = 10
			bin = np.int32(bin_n*ang/(2*np.pi))
			bin_cells = bin[:100,:100], bin[100:,:100], bin[:100,100:], bin[100:,100:]
			mag_cells = mag[:100,:100], mag[100:,:100], mag[:100,100:], mag[100:,100:]
			hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
			hist = np.hstack(hists)

			eps = 1e-7
			hist /= hist.sum() + eps
			hist = np.sqrt(hist)
			hist /= norm(hist) + eps

			# Counting non zero values
			n_zero = img != 0
			n_zero_cols = n_zero.sum(0) / norm(n_zero.sum(0) + eps)
			n_zero_rows = n_zero.sum(1) / norm(n_zero.sum(1) + eps)

			features = np.concatenate((hist, n_zero_rows[0::1]), axis=0) 	
			features = np.concatenate((features, n_zero_cols[0::1]), axis=0)	
			samples.append(features)

		return np.float32(samples)

	'''
	Both models are trained against the right image type. This step might
	take a while but once they are both loaded, classification is quick.
	'''
	def train(self):
		ratio = 2
		imgs_letters = []
		imgs_numbers = []
		labels_letters = []
		labels_numbers = []

		height, width = 120, 120

		for i in range(65, 91):
			n = 0
			while(True):
				filename = '../../samples/training/'
				if self.img_type == '8UC1':
					filename += 'rgb/raw/'
				else:
					filename += 'depth/'
				filename += unichr(i).lower() + '/' + str(n) + '.jpg'
				img = cv2.imread(filename, 0)

				# img filenames have to be consecutive
				if img is None:
					break

				#img = cv2.resize(img, (width, height))

				imgs_letters.append(img)
				labels_letters.append(i)
				n += ratio

		for i in range(48, 58):
			n = 0
			while(True):
				filename = '../../samples/training/'
				if self.img_type == '8UC1':
					filename += 'rgb/raw/'
				else:
					filename += 'depth/'
				filename += unichr(i).lower() + '/' + str(n) + '.jpg'
				img = cv2.imread(filename, 0)

				# img filenames have to be consecutive
				if img is None:
					break

				#img = cv2.resize(img, (width, height))

				imgs_numbers.append(img)
				labels_numbers.append(i)
				n += ratio

		samples_letters = self.preprocess_hog(imgs_letters)
		responses_letters = np.array(labels_letters)

		samples_numbers = self.preprocess_hog(imgs_numbers)
		responses_numbers = np.array(labels_numbers)

		self.models[1].train(samples_letters, cv2.ml.ROW_SAMPLE, responses_letters)
		self.models[0].train(samples_numbers, cv2.ml.ROW_SAMPLE, responses_numbers)
		self.save('')

def main(argv):
	print('Instancing the SVM classifier')
	svm = SVM(img_type=argv, ld=True) # Make ld=False to save a new model
	print('Classifier is loaded with both models trained with {0} images'.format(argv))
	a, b = svm.test_accuracy()
	print(b)

	rospy.init_node('classifier', anonymous=True)
	rate = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			if svm.ask:
				svm.request()
				rate.sleep()
	except KeyboardInterrupt:
		print('Shutting down')

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument(
		'--type',
		type=str,
		default='rgb',
		help='Samples to use for training and classification',
		required=False
		)

	args = parser.parse_args()
	main(argv=args.type)