Binocular Calibration
======================
A uncompleted project for binocular self calibration.

Goal
-----
Given two images and camera intrinsic parameters, get the camera pose.

Procedure
---------
* get the matched feature points;
* find fundamental matrix;
* find essential matrix;
* recover camera pose by using svd decompose.

Data
----
![](./pic/1.png) ![](./pic/2.png)

Result
-------
![](./pic/rst.png)

Confused
--------
* What is our goal? Self calibration or optimize K,R,t? Or both?
* May need svd decompose in order to get epipolar points by **Fe=0**, but hard to do in template function for epipolar optimizing.
* Optimal solution can be done through soluting an equation fo higher degree, or may be caught in local minumum by iteration search for getting the spatial points.
