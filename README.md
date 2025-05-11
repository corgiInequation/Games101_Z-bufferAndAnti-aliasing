# Games101 Experiment2

## 实验内容

- 了解z-buffer的使用，以及使用SSAA进行抗锯齿
- 对于z-buffer的解决方案，即为使用一个depth_buf数组对每个像素点的当前深度进行存储。每次需要着色一个像素点的时候，则将当前的深度z和depth_buf中的深度z进行比较，如果当前深度小于depth_buf中的深度，则说明当前图像在更前面，应该着色。着色后记得更新depth_buf中的值即可
- 对于SSAA抗锯齿，主要思想即为将一个像素点拆分成4个，且使用super_depth_buf存储拆分后的像素点的缓冲，以及使用super_color存储拆分后的像素点的颜色，最后，对每个像素点求平均即可



## 实验结果

- 抗锯齿前
- <img src="C:\Users\i love china\AppData\Roaming\Typora\typora-user-images\image-20250511130850602.png" alt="image-20250511130850602" style="zoom:50%;" />
- 绿色三角形边缘有明显的锯齿状
- 抗锯齿后
- <img src="C:\Users\i love china\AppData\Roaming\Typora\typora-user-images\image-20250511131038302.png" alt="image-20250511131038302" style="zoom:50%;" />
- 可以看到，锯齿状有所改善