##Vehicle Detection Project

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Apply a color transform and append binned color features, as well as histograms of color, to the HOG feature vector. 
* Normalize all the features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (first with the test_video.mp4 and finally on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./examples/car_not_car.png
[image2]: ./examples/HOG_example.jpg
[image3]: ./examples/sliding_windows.jpg
[image4]: ./examples/sliding_window.jpg
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.out.mp4
[video2]: ./test_video.out.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

###Histogram of Oriented Gradients (HOG)

####1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the second code cell of the IPython notebook `project.ipynb` in the function `get_hog_features()`
I started by reading in all the `vehicle` and `non-vehicle` images, as seen in the first cell.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=9`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:


![alt text][image2]

####2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and found that the set listed above provided a good trade-off between a rich feature vector and small one so that the training is trained quickly. 

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM using 14208 images of vehicles and non-vehicles and I used 3552 to test them. To do so, I build feature vectors built from YCrCb color space, taking HOG features for the 3 channels, plus binned color and color histogram features on the same space. The vector is entirely built on `get_images_features()` function in cell #5. Before going to training, the features are scaled using StandardScaler so that they all have equal inference in the final results.
It's important to note that all the training images are png files, so all the values for each channel go between zero and one (float). When procesing tested images or the video, the data goes from 0 to 255 (integer). That's why in the function `detect()` in cell #11, there's the line `im_ycrcb = im_ycrcb/255.`

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I decided to search car images using the sliding window technique, I took a default window of 64x64 pixels, and scanned with 0.5 overlapping that provided good results from a strip going from y_start = 350 to y_end = 650.
I decided to use different scales: 32x32 (0.5), 64x64 (1) and 128x128 (2). In case for the smallest one, since the number of windows grows quite rapidely, I restricted them to close to the horizon (350 < y < 450).

![alt text][image3]

####2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on three scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result. I search through a strip as described on the item before. Here are some example images:

![alt text][image4]
---

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
This is my result for the [test video][video2]

And this is the result for the [project video][video1]


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then I used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected. (In the video, additionally, I used the last 5 heatmaps and calculated their mean as a way to smoothen the boxes movement and compensate for the missing detections in isolated frames) 

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are six frames and their corresponding heatmaps:

![alt text][image5]

### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]



---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

I tried different color spaces like RGB and HSV for both HOG and bin/histogram color features. I found they were more robust for YCrCb and I stick to it. I tried using 1 channel for the HOG transform (ch1) since it was the one whose vector representation looked more like a car, but I still found it more robust with the 3 channels. I also tried skipping the repeated images (computing a hash for the vector features) but found few examples of that and turned out not have much impact on the final result, so I kept it simple.

I believe that the pipeline is likely to fail in different lightning conditions such as afternoon, artifical light, shadows. I would try to train for those different conditions as well as extracting features in color spaces where the brightness is less relevant. Also, I found that when cars are close together due to perspective, boxes tend to blend which is something that should be addressed for a real life implementation.

