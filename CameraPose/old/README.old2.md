
原始图像
--------
左
![](./pic/kitti_left.png)

右
![](./pic/kitti_right.png)

真值
----
我找不到，大部分只给了相机标定参数和左右图像，姿态需要自己求。

人为给定R,t
-----------
相机姿态R，T
![](./pic/given_rt.png)

矫正图左
![](./pic/given_rect_L.jpg)

矫正图右边
![](./pic/given_rect_R.jpg)

视差图
![](./pic/given_disp.jpg)

本质矩阵分解
------------
相机姿态R，T
![](./pic/e_rt.png)

矫正图左
![](./pic/e_rect_L.jpg)

矫正图右边
![](./pic/e_rect_R.jpg)

视差图
![](./pic/e_disp.jpg)

优化1
-----
相机姿态R，T
![](./pic/way1_rt.png)

矫正图左
![](./pic/way1_rect_L.jpg)

矫正图右边
![](./pic/way1_rect_R.jpg)

视差图
![](./pic/way1_disp.jpg)

相关的代码全是引用别人已经出版了的，除了优化部分，优化部分的目标函数是一点到另一点的极线的垂直距离，我能做的只有这么多了。