# KITTI360_Tutorial
Tutorial codes for KITTI360 Dataset.
I try to make codes from data visualization to Object Detection/Segmentation.

- Requirements : Numpy, OpenCV, Matplotlib 

## Projecting points cloud into panorma Images
- ```Lidar_to_Panorama.py```
- Method from http://www.roboticsproceedings.org/rss12/p42.pdf

![Panorama_depth](https://user-images.githubusercontent.com/50229148/185834774-a1427154-3d37-44db-ae61-26046d0b8d51.png)
https://user-images.githubusercontent.com/50229148/185847401-bafcaaf4-59ae-44e4-a4a8-bd07c50afe78.mp4

## Visualize point clouds into Bire Eye's View
Convert Velodyne Coordinate to Image Coordinate. Then extract in range points
- ```BirdEyeView.py```




![BEV_img](https://user-images.githubusercontent.com/50229148/185840096-7e4edc61-1e6a-469e-bcfd-4759f34eb1d2.png)
