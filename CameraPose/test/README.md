图漾深度相机测试
================
相机样例
--------
![](./pic/camera0.png)

内参数
------
左相机

![](./pic/K_left.png)

右相机

![](./pic/K_right.png)

测试样例
--------
#0
---
左图
![](./pic/test0_left.jpg)

右图
![](./pic/test0_right.jpg)

筛选特征点
![](./pic/test0_match.jpg)

RANSAC掩码特征点
![](./pic/test0_ransac_match.jpg)

初始矫正
![](./pic/test0_init_rect.jpg)

优化矫正
![](./pic/test0_opt_rect.jpg)

初始视差图
![](./pic/test0_init_disp.jpg)

优化视差图
![](./pic/test0_opt_disp.jpg)

初始姿态

![](./pic/test0_init_RT.png)

优化姿态

![](./pic/test0_opt_RT.png)

#1
---
左图
![](./pic/test1_left.jpg)

右图
![](./pic/test1_right.jpg)

筛选特征点
![](./pic/test1_match.jpg)

RANSAC掩码特征点
![](./pic/test1_ransac_match.jpg)

初始矫正
![](./pic/test1_init_rect.jpg)

优化矫正
![](./pic/test1_opt_rect.jpg)

初始视差图
![](./pic/test1_init_disp.jpg)

优化视差图
![](./pic/test1_opt_disp.jpg)

初始姿态

![](./pic/test1_init_RT.png)

优化姿态

![](./pic/test1_opt_RT.png)


#2
---
左图
![](./pic/16_leftIR.jpg)

右图
![](./pic/16_rightIR.jpg)

筛选特征点
![](./pic/16_leftIR.jpg+16_rightIR.jpg.imageMatches..imageMatches.jpg)

RANSAC掩码特征点
![](./pic/16_leftIR.jpg+16_rightIR.jpg.imageMatches..matchAgain.jpg)

初始矫正
![](./pic/16_leftIR.jpg+16_rightIR.jpg.init..stereoRectify.jpg)

优化矫正
![](./pic/16_leftIR.jpg+16_rightIR.jpg.opt..stereoRectify.jpg)

初始视差图
![](./pic/16_leftIR.jpg+16_rightIR.jpg.init..disparity.jpg)

优化视差图
![](./pic/16_leftIR.jpg+16_rightIR.jpg.opt..disparity.jpg)

初始姿态

![](./pic/test2_init_RT.png)

优化姿态

![](./pic/test2_opt_RT.png)

总结
----
本算法良好运行的前提条件是：
* 拍摄的图像清晰度较高，肉眼辨识度良好
* 避免背景单一

展望
----
* 提高算法的鲁棒性
* 适应低辨识度和背景单一的场景

END.