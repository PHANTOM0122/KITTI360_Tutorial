"""
Visualize Velodyne point clouds into Bird Eye's View
"""

import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2

def load_point_clouds(bin_path):
    """
    Load point clouds from raw file
    .bin file to numpy array
    """
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4) # (x,y,z) coordinate + reflection
    return points[:,:3] # Return only x,y,z coordinate

def normalize_depth(val, min_val, max_val):
    """
    Convert to depth and normalize value from 0 to 255.
    Closer Distance has high value.
    """
    return (((max_val - val) / (max_val - min_val)) * 255).astype(np.uint8)

def in_range_points(points, x, y, z, x_range, y_range, z_range):
    """
    Extract in range points
    """
    return points[np.logical_and.reduce((x > x_range[0], x < x_range[1], y > y_range[0],
                                         y < y_range[1], z > z_range[0], z < z_range[1]))]

def point_from_BEV(points, x_range, y_range, z_range, scale):
    """
    - scale : For high resoultion
    - Velodyne coordinate : http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    dist = np.sqrt(x**2 + y**2) # Distance in XY plane

    # In range-points
    x_limit = in_range_points(x, x, y, z, x_range, y_range, z_range)
    y_limit = in_range_points(y, x, y, z, x_range, y_range, z_range)
    dist_lim = in_range_points(dist, x, y, z, x_range, y_range, z_range)

    # Convert Velodyne lidar coordinate into Image coordinate
    x_img = -(y_limit * scale).astype(np.int32)
    y_img = -(x_limit * scale).astype(np.int32)

    # Shift points to have positive coordinate value
    x_img += int(np.trunc(y_range[1] * scale))
    y_img += int(np.trunc(x_range[1] * scale))

    # Normalize distance value & convert to depth map
    max_dist = np.sqrt((max(x_range)**2) + (max(y_range)**2))
    dist_limit = normalize_depth(dist_lim, min_val=0, max_val=max_dist)

    # Numpy array to img
    x_size = int((y_range[1] - y_range[0])) # size are based on lidar coordinates
    y_size = int((x_range[1] - x_range[0]))
    img = np.zeros([y_size * scale + 1, x_size * scale + 1], dtype=np.uint8)
    img[y_img, x_img] = dist_limit

    return img

if __name__ == "__main__":

    bin_path = '/mnt/nas/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/0000000089.bin'
    x_range, y_range, z_range, scale = (-20, 20), (-20, 20), (-2, 2), 10 # Range/Scale parameters
    size = int((max(x_range) - min(x_range)) * scale), int((max(y_range) - min(y_range)) * scale) # Size for BEV Image based on lidar range coordinate

    points = load_point_clouds(bin_path)
    print(points.shape)

    BEV_img = point_from_BEV(points, x_range=(-20, 20), y_range=(-20, 20), z_range=(-2, 2), scale=10)

    # Display panorama image
    plt.subplots(1, 1, figsize=(13, 3))
    plt.imshow(BEV_img, cmap='Greys')
    plt.axis('off')
    plt.imsave('BEV_img.png', BEV_img, cmap='Greys')
    plt.show()
    print(BEV_img.shape)

    # Save all Panorama Images in Video
    velodyne_points = sorted(glob.glob('/mnt/nas/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/*.bin'))

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    vid = cv2.VideoWriter('BEV.avi', fourcc, 25.0, (1030, 66), False)

    for point in velodyne_points:
        velodyne_point = load_point_clouds(point)
        img = point_from_BEV(velodyne_point, x_range=(-20, 20), y_range=(-20, 20), z_range=(-2, 2), scale=10)
        vid.write(img)

    print('video saved')
    vid.release()

