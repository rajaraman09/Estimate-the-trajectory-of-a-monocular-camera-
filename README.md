# Estimate-the-trajectory-of-a-monocular-camera-

 The technique employed to perform this assignment is to
estimate the essential matrix between successive frames, extract the rotation matrix R , the
translation vector T⃗ , and 3D plot the trajectory using the T⃗ vectors obtained between
each frame of the image sequence.

The required steps to complete are as follows:
• For two successive frames in the video, find point correspondences (more than 8). Use
the FLANN matching technique with SIFT features.
• With the correspondences, now estimate the fundamental matrix F . Use OpenCV
routines and make sure RanSaC is on while performing this estimation.
• Once F is estimated, use OpenCV routines and the intrinsic calibration parameters
obtained in Assignment 3 to estimate E , the essential matrix. (Pay attention to the
difference in resolution between the calibration images and the video itself. You will
need to resize the calibration images to that of the video sequence, and then calibrate
to obtain the intrinsic parameters).
• Decompose E into R and T⃗
• Add this translation to your 3D plot for the camera trajectory. Use GnuPlot to produce
the final 3D plot of the camera trajectory.
• Get the next frame in the video and repeat the previous steps.

