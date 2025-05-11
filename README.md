# Games101 Experiment 2

## Experiment Content

- Understand the use of the Z-buffer and apply SSAA (Super Sampling Anti-Aliasing) for anti-aliasing.
- The Z-buffer solution involves maintaining a `depth_buf` array to store the depth value of each pixel. When a pixel is about to be shaded, its depth `z` is compared with the value in `depth_buf`. If the current `z` is smaller, it indicates that the fragment is closer to the camera and should be shaded. After shading, the `depth_buf` should be updated accordingly.
- For SSAA anti-aliasing, the main idea is to divide each pixel into 4 sub-pixels. A `super_depth_buf` is used to store the depth of each sub-pixel, and `super_color` is used to store the color values. Finally, the color of each pixel is obtained by averaging the colors of its sub-pixels.

## Experiment Results

- Before anti-aliasing  
- <img src="https://github.com/corgiInequation/Games101_Z-bufferAndAnti-aliasing/blob/main/image1.png" alt="before-aa" width="50%" />  
- The edges of the green triangle show obvious jagged artifacts.  

- After anti-aliasing  
- <img src="https://github.com/corgiInequation/Games101_Z-bufferAndAnti-aliasing/blob/main/image2.png" alt="after-aa" width="50%" />  
- The jagged edges are visibly smoothed out.
