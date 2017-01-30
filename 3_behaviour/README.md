Behavioral cloning
******************

Contents
--------
model.py - file used to train the network. With the params 'eval', used to graph the differences between the data and the predictions
drive.py - file provided by Udacity. Only differences were image scaling and normalization to fit the network's input and change throttle to 0.1 when angles < 0.1, else set to 0.05 
(so that the car would slow down in closed turns)
model.json - The model architecture.
model.h5 - The model weights.
README.md - this file

Data used
---------
*sample data provided by Udacity: https://d17h27t6h515a5.cloudfront.net/topher/2016/December/584f6edd_data/data.zip

Network architecture
--------------------
Based on nVidia's end to end driving paper: http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf

*Input images are scaled to half (from 160x320 to 80x160), kept in RGB (3 channels) and normalized - (no normalization, grayscale were tried with worse results)
*5 convolutional layers, with relu activation, 3 with sizes ({24,36,48},5,5) subsample (2,2) and valid padding and 2 with (64,3,3) and subsample(1,1).
*Flatten and 5 dense flat layers with sizes 1164,100,50,10 and 1
*Medium squared error as metric to optimize the loss, learning rate = 0.001

Model.py
--------
Images picked by training are the center ones of all the available csv files
Images are scaled half the original size
Normalized around zero
Mirrored images are generated, y value inverted
Shuffled
Used for training, 20% of the batch used for validation, picked randomly
Model is saved in each epoch using Keras callback
Results are graphed using matplotlib

Some notes
----------
*Model was more stable in RGB than in grayscale
*Image scaling brought good results while maintaining a decent size for the images in memory (no run out of memory exceptions)
*Training was done in CPU (tried creating a GPU instance in Amazon but was not enabled during the last week since they require an account to have at least one billing so that
they know the account is active and they won't be allocating resources to an inactive account (silly since they can always take it back)
*Training with keyboard or mouse was hard, inaccurate. Lots of editing for the recording had to be done.
*Training after a few epochs was enough, more than 3-4 would bring overfitting the data and instability while driving

