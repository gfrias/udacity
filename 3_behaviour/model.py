import pickle
import numpy as np
import math

# Fix error with TF and Keras
import tensorflow as tf
tf.python.control_flow_ops = tf

from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping, ModelCheckpoint

from sklearn.utils import shuffle
from sklearn.preprocessing import LabelBinarizer

import cv2

TRAINING_SET = ['IMG/center_2016_12_01_13_30_48_404.jpg', 'IMG/center_2016_12_01_13_32_43_761.jpg', 'IMG/center_2016_12_01_13_32_53_966.jpg']

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

		log.append(entry)
	
	return log

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

	model.add(Convolution2D(24, 5, 5, input_shape=(80, 160, 1), subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(36, 5, 5, subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(48, 5, 5, subsample=(2, 2), border_mode="valid", activation='relu'))
	model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", activation='relu'))
	model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", activation='relu'))

	model.add(Flatten())
	#model.add(Dropout(0.2))
	model.add(Activation('relu'))

	model.add(Dense(1164))
	#model.add(Dropout(0.2))
	model.add(Activation('relu'))

	model.add(Dense(100))
	model.add(Dense(50))
	model.add(Dense(10))
	model.add(Dense(1))

	model.compile(optimizer=Adam(lr=0.001), loss="mean_squared_error", metrics=['mean_absolute_error'])

	return model

def build_model2():
	model = Sequential()

	model.add(Flatten(input_shape=(80, 160, 3)))
	model.add(Dense(1000))
	model.add(Dense(100))

	model.add(Dense(10))
	model.add(Dense(1))
	return model

def build_model():
	model = Sequential()

	model.add(Convolution2D(24, 5, 5, input_shape=(80, 160, 3)))
	#model.add(Activation('relu'))

	model.add(Convolution2D(36, 5, 5))
	#model.add(Activation('relu'))

	model.add(Convolution2D(48, 5, 5))
	#model.add(Activation('relu'))

	model.add(Convolution2D(64, 3, 3))
	#model.add(Activation('relu'))

	model.add(Convolution2D(64, 3, 3))
	#model.add(Activation('relu'))

	model.add(Flatten())

	model.add(Dense(1000))
	#model.add(Activation('relu'))

	model.add(Dense(100))
	#model.add(Activation('relu'))

	model.add(Dense(50))
	#model.add(Activation('relu'))

	model.add(Dense(10))
	#model.add(Activation('relu'))

	model.add(Dense(1))
	#model.add(Activation('relu'))

	return model

def evaluate_model(Xs, ys):
	metrics = model.evaluate(Xs, ys)
	for metric_i in range(len(model.metrics_names)):
		metric_name = model.metrics_names[metric_i]
		metric_value = metrics[metric_i]
		print('{}: {}'.format(metric_name, metric_value))

SIZE = 10000
log = load_log('data/driving_log.csv');
X_train, y_train = load_data(log, TRAINING_SET)
X_train, y_train = shuffle(X_train, y_train)
X_train, y_train = X_train[:SIZE], y_train[:SIZE]
print(len(X_train))
X_train = np.asarray([load_image(filename) for filename in X_train])

X_normalized = normalize(to_grayscale(X_train))
model = build_model3()
#model.compile('adam', 'mse', ['accuracy'])

#callbacks = [EarlyStopping(monitor='loss',patience=5,verbose=1), ModelCheckpoint("model.autosave", monitor='val_loss', verbose=1, save_best_only=False, save_weights_only=False, mode='auto', period=1)]
callbacks = [ModelCheckpoint("model.autosave", monitor='val_loss', verbose=1, save_best_only=False, save_weights_only=False, mode='auto', period=1)]
history = model.fit(X_normalized, y_train, nb_epoch=50, shuffle=True, verbose=1, validation_split=0.2, callbacks=callbacks)

y_pred = model.predict(X_normalized, batch_size=1)
print(y_pred)
print(y_train)

import matplotlib.pyplot as plt
plt.plot(range(len(y_train)), y_train)
plt.plot(range(len(y_train)), y_pred)
plt.show()

#evaluate_model(X_train, y_train)

model.save('model.h5')
json_string = model.to_json()

f = open("model.json", "w")
f.write(json_string)
f.close()

#
#X_test, y_test = load_data('test.p')
#X_normalized_test = normalize(X_test)
#y_one_hot_test = label_binarizer.fit_transform(y_test)
#
#evaluate_model(X_normalized_test, y_one_hot_test)
