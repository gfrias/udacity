import pickle
import numpy as np
import math
import sys
import glob, os

# Fix error with TF and Keras
import tensorflow as tf
tf.python.control_flow_ops = tf

from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping, ModelCheckpoint
from keras.models import model_from_json

from sklearn.utils import shuffle
from sklearn.preprocessing import LabelBinarizer

import cv2

TRAINING_SET = ['IMG/center_2016_12_01_13_30_48_404.jpg', 'IMG/center_2016_12_01_13_32_43_761.jpg', 'IMG/center_2016_12_01_13_32_53_966.jpg']
MAX = 100000

def load_log(filename):
	log = []

	f = open(filename);
	headers = f.readline().rstrip().split(',')
	for line in f:
		data = line.rstrip().split(',')
		entry = {}
		for i in range(len(headers)):
			val = data[i]
			if i >= 3:
				val = float(val)
			entry[headers[i]] = val
		if (abs(entry['steering']) < 0.8):
			log.append(entry) 
	
	return log

def load_logs():
	ret = []
	for file in glob.glob("data/*.csv"):
		log = load_log(file)
		ret.extend(log)

	return ret


def load_image(filename):
	img = cv2.imread('data/' + filename)
	return cv2.resize(img, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)

def load_data(log, filenames):
	files = dict(zip(filenames, [1]*len(filenames)))
	entries = [e for e in log]# if e['center'] in files]

	imgs = [e['center'] for e in entries]
	steering = [e['steering'] for e in entries]

	return imgs, np.asarray(steering)

def normalize(image_data):
	a = -0.5
	b = 0.5
	grayscale_min = 0
	grayscale_max = 255
	return a + ( ( (image_data - grayscale_min)*(b - a) )/( grayscale_max - grayscale_min ) )

def to_grayscale(rgb):
	#converting to grayscale using a similar approach to MatLab
	#https://nl.mathworks.com/help/matlab/ref/rgb2gray.html
	ret = np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

	return np.expand_dims(ret, axis=4)

def build_model3():
	    
	model = Sequential()

	model.add(Convolution2D(24, 5, 5, input_shape=(80, 160, 3), subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(36, 5, 5, subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(48, 5, 5, subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", activation='relu'))
	model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", activation='relu'))

	model.add(Flatten())
	model.add(Dropout(0.2))
	model.add(Activation('relu'))

	model.add(Dense(1164))
	model.add(Dropout(0.2))
	model.add(Activation('relu'))

	model.add(Dense(100))
	model.add(Dense(50))
	model.add(Dense(10))
	model.add(Dense(1))

	model.compile(optimizer=Adam(lr=0.0001), loss="mean_squared_error", metrics=['mean_absolute_error'])

	return model

def mirror(img):
	rimg=img.copy()
	return cv2.flip(img,1)

def load_model():
	with open('model.json', 'r') as jfile:
		print("load json")
		model = model_from_json(jfile.read())
		print ("json loaded")
	print("compiling model")
	model.compile("adam", "mse")
	print("loading weights")
	model.load_weights('model.h5')

	return model

def save_model(model):
	json_string = model.to_json()

	f = open("model.json", "w")
	f.write(json_string)
	f.close()

def train_model(model, Xs, ys, model_name):
	callbacks = [ModelCheckpoint(model_name, monitor='val_loss', verbose=1, save_best_only=False, save_weights_only=False, mode='auto', period=1)]
	print("training model")
	history = model.fit(Xs, ys, nb_epoch=3, shuffle=True, verbose=1, validation_split=0.2, callbacks=callbacks)
	return history

print("loading logs")
log = []
if len(sys.argv) > 1 and str(sys.argv[1]) == 'improve':
	log = load_log(str(sys.argv[2]))
else:
	log = load_logs()
	#log = load_log("data/extra.csv")

print("loading data")
X_train, y_train = load_data(log, TRAINING_SET)
print("shuffling")
X_train, y_train = shuffle(X_train, y_train)
print("sampling")
X_train, y_train = X_train[:MAX], y_train[:MAX]
print(len(X_train))
print("loading image")
X_train = np.asarray([load_image(filename) for filename in X_train])
print("building mirror images")
X_train2 = np.asarray([mirror(img) for img in X_train])
X_train = np.concatenate((X_train, X_train2), axis=0)
y_train2 = [-y for y in y_train]
y_train = np.concatenate((y_train, y_train2), axis=0)

print("normalizing images")
#X_normalized = normalize(to_grayscale(X_train))
X_normalized = normalize(X_train)

if len(sys.argv) > 1:
	if str(sys.argv[1]) == 'eval':
		print("eval model")
		model = load_model()
	elif str(sys.argv[1]) == 'improve':
		print("improve model")
		model = load_model()
		train_model(model, X_normalized, y_train, 'model.h5.im')
else:
	#callbacks = [EarlyStopping(monitor='loss',patience=5,verbose=1), ModelCheckpoint("model.h5", monitor='val_loss', verbose=1, save_best_only=False, save_weights_only=False, mode='auto', period=1)]
	model = build_model3()
	save_model(model)
	train_model(model, X_normalized, y_train, 'model.h5')
print("predicting")
y_pred = model.predict(X_normalized, batch_size=1)
print(y_pred)
print(y_train)

import matplotlib.pyplot as plt
line_train, = plt.plot(range(len(y_train)), y_train, label='train')
line_pred, = plt.plot(range(len(y_train)), y_pred, label='predicted')

plt.legend(handles=[line_train, line_pred])
plt.show()

#model.save('model.h5')

