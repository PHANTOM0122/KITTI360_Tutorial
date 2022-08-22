# KITTI360_Tutorial
Tutorial codes for KITTI360 Dataset.
I try to make codes from data visualization to Object Detection/Segmentation.

- Requirements : Numpy, OpenCV, Matplotlib 

## Projecting points cloud into panorma Images
- ```Lidar_to_Panorama.py```
- Method from http://www.roboticsproceedings.org/rss12/p42.pdf

![depth_panorama](https://user-images.githubusercontent.com/50229148/185848007-da5af732-b3ce-45ce-9641-2f391fc3b059.gif)

## Visualize point clouds into Bire Eye's View
Convert Velodyne Coordinate to Image Coordinate. Then extract in range points
- ```BirdEyeView.py```




![BEV_img](https://user-images.githubusercontent.com/50229148/185840096-7e4edc61-1e6a-469e-bcfd-4759f34eb1d2.png)
