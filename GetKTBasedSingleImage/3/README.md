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

![](./pic/monitor1.jpg)


Real Environment
----------------------------
The following pictures were taken by an Android phone.
* 1st image
source image:

![](./pic/1.jpg)

result

![](./pic/img1_rst.jpg)


* 2nd image
source image: 

![](./pic/2.jpg)

result

![](./pic/img2_rst.jpg)


* 3rd image
source image: 

![](./pic/3.jpg)

result

![](./pic/img3_rst.jpg)


* 4th image
source image: 

![](./pic/4.jpg)

result

![](./pic/img4_rst.jpg)


* 5th image
source image: 

![](./pic/5.jpg)

result

![](./pic/img5_rst.jpg)


The following pictures were from [matlab official websites](http://robots.stanford.edu/cs223b04/JeanYvesCalib/htmls/example.html).
* 6th image
source image: 

![](./pic/6.jpg)

result

![](./pic/img6_rst.jpg)


* 7th image
source image: 

![](./pic/7.jpg)

result

![](./pic/img7_rst.jpg)


* 8th image
source image: 

![](./pic/8.jpg)

result

![](./pic/img8_rst.jpg)


* 9th image
source image: 

![](./pic/9.jpg)

result

![](./pic/img9_rst.jpg)


# Summary
From the test result, we can get the almost perfect estimated value and perfect optimized value in the no noise monitor environment. Meanwhile, we got the not bad estimated value with Zhang's and the not bad optimized value in the real environment. Thus, **we can merely get good estimated value and optimized value when the target points close to the principal point were enough, besides, we also need to adjust parameters to get the good homography**.

