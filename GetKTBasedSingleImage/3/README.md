Camera Mode Parameter Optimization
====================================
An interface and an example of optimizing the camera intrinsic paramters, extrinsic paramters and distortion paramters which were estimated through a single planar image based on the fact that the planar points coordinate and the corresponding image points coordinate, and the principal point and image size were given was provided by this project.

List
----
* optimize.h: two function interfaces were provided.
* optimize.cpp: the realizing produce of two function interfaces.
* main.cpp: an example of the usage of this api.

Test Result
------------
No Noise Monitor Environment
----------------------------
result

![](./pic/monitor.jpg)


Real Environment
----------------------------
### 1st image
source image:
![](./pic/1.jpg)

result

![](./pic/img1_rst.jpg)


### 2nd image
source image: 
![](./pic/2.jpg)

result

![](./pic/img2_rst.jpg)


### 3rd image
source image: 
![](./pic/3.jpg)

result

![](./pic/img3_rst.jpg)

# Summary
From the test result, we can get the almost perfect estimated value and perfect optimized value in the no noise monitor environment. Meanwhile, we got the bad estimated value and optimized value in first image and the not good estimated value and optimized value in second image but the good estimated value and optimized value in third image . Thus, we can merely get good estimated value and optimized value when the target points close to the principal point were enough.

