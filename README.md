# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

# Mid-Term Report

## 1. Data Buffer Optimization
To realize the data buffer, my solution is: at the first time the images are inserted into the buffer space, just simply insert the images; when buffer space is full, the first image in the buffer space should be released with “erase”, and a new image should be inserted to the last position of the buffer space with “push_back”.

## 2. Keypoint Detection
Function „detKeypointsShiTomasi” is for detector Shi-Tomasi
Function “detKeypointsHarris” is for detectors Harris
Detectors BRISK. FAST, ORB, AKAZE and SIFT are integrated in the function
“detKeypointsModern”, which can be chose by entering their corresponding “detectorType”.

## 3. Keypoint Removal
In the main program “MidTermProject_Camera_Student.cpp” (Line 109 – Line 122), keypoints from above are sifted which only the points “keypoints[i].pt” in the vehicle bounding box are selected.

## 4. Keypoint Descriptors
In function „descKeypoints“, all descriptors can be found and can be implemented.

## 5. Descriptor Matching
In “matching2D_Student.cpp” under the function “matchDescriptors”, all matching types (BF and FLANN) and selsctor types (NN and KNN) are included. It should be noticed that the descriptor format of BF and FLANN should fit the format of different descriptors.

## 6. Descriptor Distance Ratio
In “matching2D_Student.cpp”, from line 47 to 67 the kNN distance ratio is set so as to choose the best match pair.

## 7 - 9. Performance Evaluation
All results using different detectors and descriptions are listed below. Each number is the calculation of the average value of 10 images / 9 matches. AKAZE detector functions only when descriptor is AKAZE as well and SIFT detector with ORB description leads to out of memory. All results are documented in the file "uda_2d.xlsx".

The best three detectors which detect most key points (before target sifting) are:
1. BRISK
2. FAST
3. SIFT

The best three combinations which get most match points within the car box are:
1. BRISK-BRIEF
2. BRISK-SIFT
3. BRISK-FREAK

The best three detectors which owns the minimum detection time are:
1. FAST
2. ORB
3. SHIMATOSI

The best three combinations which owns the minimum detection time are:
1. HARRIS-BRIEF
2. ORB-BRIEF
3. FAST-BRIEF

Overall, the recommended combinations should balance the effectiveness of time consuming and key points finding and matching. The TOP3 detector-descriptor combinations are:
1. FAST-BRIEF
2. FAST-ORB
3. FAST-SIFT


