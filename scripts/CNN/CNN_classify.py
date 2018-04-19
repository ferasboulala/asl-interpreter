#!/usr/bin/env python

import argparse
import sys
import os
import std_msgs.msg
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf

class Classifier:
	def __init__(self, img_type='rgb'):
		self.pub = rospy.Publisher('asl/letter', std_msgs.msg.String, queue_size=1)
		self.sub = rospy.Subscriber('asl/hands/right', Image, self.callback, queue_size=1)
		self.subType = rospy.Subscriber('asl/hands/type', std_msgs.msg.Bool, self.typeCallback, queue_size=1)
		self.bridge = CvBridge()

		if img_type == 'rgb':
			self.img_type = '8UC1'
		else:
			self.img_type = '16UC1'

		self.ask = True
		self.symbol = ''

		self.type = 0 # 0 for numbers and 1 for letters

		self.label_lines = []
		self.label_lines.append([line.rstrip() for line in \
			tf.gfile.GFile(img_type + "_logs/numbers_logs/trained_labels.txt")])
		self.label_lines.append([line.rstrip() for line in \
			tf.gfile.GFile(img_type + "_logs/letters_logs/trained_labels.txt")])

		self.sess = []
		with tf.gfile.FastGFile(img_type + "_logs/numbers_logs/trained_graph.pb", 'rb') as f:
			self.graph_def = tf.GraphDef()
			self.graph_def.ParseFromString(f.read())
			self._ = tf.import_graph_def(self.graph_def, name='')

		self.sess.append(tf.Session(self._))
		tf.reset_default_graph()

		with tf.gfile.FastGFile(img_type + "_logs/letters_logs/trained_graph.pb", 'rb') as f:
			self.graph_def = tf.GraphDef()
			self.graph_def.ParseFromString(f.read())
			self._ = tf.import_graph_def(self.graph_def, name='')

		self.sess.append(tf.Session(self._))	

		self.softmax_tensor = []
		self.softmax_tensor.append(self.sess[0].graph.get_tensor_by_name('final_result:0'))
		self.softmax_tensor.append(self.sess[1].graph.get_tensor_by_name('final_result:0'))

	def callback(self, data):
		if not self.ask:
			return
		self.ask = False
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, self.img_type)
			if self.img_type == '8UC1':
				self.image = cv2.Laplacian(self.image, cv2.CV_8U)
			self.detect(self.image)
		except CvBridgeError as e:
			print(e)
			return

	def typeCallback(self, data):
		self.type = data.data

	def request(self):
		self.pub.publish(self.symbol)

	def detect(self, image):
		image_data = cv2.imencode('.jpg', self.image)[1].tostring()
		predictions = self.sess[self.type].run(self.softmax_tensor[self.type], {'DecodeJpeg/contents:0': image_data})
		top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
		self.symbol = self.label_lines[self.type][top_k[0]]
		self.ask = True

def main(argv):
	print('Instancing the CNN classifier')
	viewer = Classifier(img_type=argv)
	print('Classifier is loaded with both models trained with {0} images'.format(argv))

	rospy.init_node('classifier', anonymous=True)
	rate = rospy.Rate(2)
	
	try:
		while not rospy.is_shutdown():
			if viewer.ask:
				viewer.request()
				rate.sleep()
	except KeyboardInterrupt:
		print('Shutting down')

if __name__ == "__main__":
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