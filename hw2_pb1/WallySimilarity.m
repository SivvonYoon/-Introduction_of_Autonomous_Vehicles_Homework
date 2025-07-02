%{
1. Consider the problem from the well known children’s book “Where’s Wally” or “Where’s
Waldo” – the fun is trying to find Wally’s face in a crowd. 

In this exercise, you will implement a template matching algorithm. 
Using ZNCC algorithm, find where is wally.

In this exercise, you will implement a simple harris-corner detector, 
using only the knowledge you have so far obtained in the class. 

You will achieve this with the following steps:

First, you will evaluate the Harris score for each pixel of the input image. 

Then, you will select keypoints based on the Harris scores. 

In a next step, you will evaluate simple image patch descriptors at the selected keypoint locations. 

Finally, you will match these descriptors using
the SSD norm in order to find feature correspondences between frames.

For your homework, you have to implement your code in ‘harris.m’ and
‘selectKeypoints.m'
%}

function S = WallySimilarity(T, im, metric)

% TODO add all the other similarity metrics, including rank and census

% your code here