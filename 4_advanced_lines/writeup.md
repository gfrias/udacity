##**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image7]: ./output_images/chessboard/0.png "Original"
[image8]: ./output_images/chessboard/0.undist.png "Undistorted"
[image9]: ./output_images/camera/2.png "Original"
[image10]: ./output_images/camera/2.undist.png "Undistorted"
[image11]: ./output_images/binary/2.bin.png "Binary"
[image12]: ./output_images/warped/0.png "Original"
[image13]: ./output_images/warped/0.wrp.png "Warped"
[image14]: ./output_images/window/3.wnd.png "Windows"
[image15]: ./writeup_files/poly.png "Polynomial"
[image16]: ./writeup_files/radius.png "Radius"
[image17]: ./output_images/window/3.png "Overlay"
[image18]: ./output_images/warped/6.wrp.png "Shadows"
[image19]: ./writeup_files/s_sx.png "S or Sobel X"
[image20]: ./writeup_files/s_l_sx.png "L and (S or Sobel X)"

[video1]: ./project_video.lines.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!
###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "project.ipynb", defined by the functions `load_orig()`, `calibrate_camera()` and `build_undist()`

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

**Original**
![alt text][image7]
**Undistorted**
![alt text][image8]

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.
I apply in block #4 in project.ipnyb the same sequence of functions as in the chessboard step, first I list all the images in the `test_images/` directory, then I load them `load_orig()`, after that I undistort them `build_undist()` and finally I display them in the notebook `display()` and save them in `output_images/camera/` directory for future reference `save_images()`.

**Original**
![alt text][image9]
**Undistorted**
![alt text][image10]
####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
After some tests using sobelX, sobelY, both, and various color transforms and channel thresholding, I found that usign sobelX, channel S thresholding for enhancing the lines wheather they're yellow or white and channel L thresholding for eliminating the shadows (filtering out the very low lightness values) showed a good performance (see `s_l_and_sobelx()` in the notebook). The L channel is and AND operation with what was detected by either the sobelX or S channel.

![alt text][image11]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in block 6 in the notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```
src = np.float32(
    [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 60, img_size[1]],
    [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])

```
This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 585, 460      | 320, 0        | 
| 203, 720      | 320, 720      |
| 1127, 720     | 960, 720      |
| 695, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

**Original**
![alt text][image12]
**Warped**
![alt text][image13]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

I took as a starting point the histogram/sliding window approach explained in the classes.
First I use the histogram for the binary image to find the peaks for it and possible locations for the lane. Then I use sliding windows all the pixels that belong to the lane. For the next window, I get the mean value in x for both lanes so that the window is as centered as possible.
With the indices for the lit up pixels that belong to the lane, I use `np.polyfit()` to calculate a polynomial that best fit each of them.
Also, I keep a history in two global variables `left_fitx_history`, `right_fitx_history` as an array with the 5 (`SIZE`) latest 'valid' iterations for points for each lane and I use the mean to smoothen the lane.
A valid iteration is defined as one where the mean value for the difference is a reasonable value (between 600 and 800 pixels in this case).

![alt text][image14]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in the function `get_stats()` in the `project.ipynb` file. First I define the ratio meters to pixels in both x and y coordinates.
Then, I convert all points from pixel to meter scale and I fit both curves (left and right) to a 2nd degree polynomial.
![alt text][image15]

Once I got them, I can calculate the radius (in meters) by plugging the polynomials coefficients into the formula:

![alt text][image16]

Then I return the minimum of both as the radius of curvature.

As for the center, I substract the points to the right to the points to the left and find the mean for it. With that, I substract 1280/2 = half the image horizontally to that value and finally I convert it to meters.


####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in the notebook `project.ipnyb` in the function `draw_lines()` that takes as parameters the undistorted image plus an optional parameter whether to smooth with previous images (used in the video) or not (used in the static sample images since they're from very different moments).

Here is an example of my result on a test image:

![alt text][image17]



---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.lines.mp4) (updated!)

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

One of the problems faced was the shadows appearing in the video made line detection impossible using the classes approach (sobelX and S threshold). 
As it can be seen here

![alt text][image18]

Applying S or SobelX the result is dissapointing
![alt text][image19]

While for L and (S or Sobel X) is much improved
![alt text][image20]

Several alternatives were tested and I found out that adding an L threshold to wipe out the very dark parts was a good way to overcome this difficulty.

Some of the problems left to be addressed are making the edge detection more robust in high and low brightness, or cases like the [challenge video](./challenge_video.mp4) where a difference in the color of the pavement is shown approximately in the half of the lane, or when crossing the bridge. The existing color filters should be fine tuned and possibly new ones should be added.

