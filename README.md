# KITTI360_Tutorial
Tutorial codes for KITTI360 Dataset.
I try to make codes from data visualization to Object Detection/Segmentation.

- Requirements : Numpy, OpenCV, Matplotlib 

## Projecting points cloud into panorma Depth Images
- ```Lidar_to_Panorama.py```
- Method from http://www.roboticsproceedings.org/rss12/p42.pdf

![depth_panorama](https://user-images.githubusercontent.com/50229148/185848007-da5af732-b3ce-45ce-9641-2f391fc3b059.gif)

## Visualize point clouds into Bire Eye's View
Convert Velodyne Coordinate to Image Coordinate. Then extract points in distance range. 
- ```BirdEyeView.py```

![BEV_2](https://user-images.githubusercontent.com/50229148/185849481-cb61493b-70df-468f-8a5d-6aa13fe2cd1b.gif)

## Project Velodyne points into Image
- ```Lidar_to_cam_projection.py```

![original](https://user-images.githubusercontent.com/50229148/186425305-f56acadc-3f57-43fc-9383-cdd89631995d.png)
![projection](https://user-images.githubusercontent.com/50229148/186425076-a3785736-5666-41e9-8a11-36f7f33d5157.png)
