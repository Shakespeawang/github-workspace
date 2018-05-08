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
## No Noise Monitor Environment
True value:
A picture url of true value.

Estimated value:
A picture url of estimated value.

Optimized value:
A picture url of optimized value.


## Real Environment
### 1st image
source image: 
A picture url of true value.

True value:
A picture url of true value.

Estimated value:
A picture url of estimated value.

Optimized value:
A picture url of optimized value.


### 2nd image
source image: 
A picture url of true value.

True value:
A picture url of true value.

Estimated value:
A picture url of estimated value.

Optimized value:
A picture url of optimized value.


### 3rd image
source image: 
A picture url of true value.

True value:
A picture url of true value.

Estimated value:
A picture url of estimated value.

Optimized value:
A picture url of optimized value.

# Summary
From the test result, we can get the almost perfect estimated value and optimized value in the no noise monitor environment, meanwhile, we can merely get good estimated value and optimized value when the target points close to the principal point were enough.

