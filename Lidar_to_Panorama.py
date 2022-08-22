"""
Convert point clouds from Velodyne HDL-64E to Panorama Images
The method is from : http://www.roboticsproceedings.org/rss12/p42.pdf
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

def normalize_disparity(val, min_val, max_val):
    """
    Normalize disparity value from 0 to 255.
    Closer Distance has low value.
    """
    return (((val - min_val) / (max_val - min_val)) * 255).astype(np.uint8)

def horizontal_range_points(m, n, fov):
    return np.logical_and(np.arctan2(n,m) > (-fov[1] * np.pi / 180), np.arctan2(n,m) < (-fov[0] * np.pi / 180))

def vertical_range_points(m, n, fov):
    return np.logical_and(np.arctan2(n, m) < (fov[1] * np.pi / 180), np.arctan2(n, m) > (fov[0] * np.pi / 180))

def FOV_filtering(points, x, y, z, dist, h_fov, v_fov):
    """
    Filter points according to FOV.
    The settings are different for each LiDAR sensor.
    This setting fits for Velodyne HDL_64E
    FOV(vertical) : +2 degree to -24.9 degree
    Angular resolution(vertical) : 0.4 degree
    FOV(horizontal) : 360 degree
    Angular resolution(horizontal) : 0.08 ~ 0.35 degree ( 5Hz ~ 30Hz )
    """
    # In the angle of FOV
    if h_fov[1] == 180.0 and h_fov[0] == -180.0 and v_fov[1] == 2.0 and v_fov[0] == -24.9:
        return points
    # Check in vertical angle of FOV
    if h_fov[1] == 180.0 and h_fov[0] == -180.0:
        return points[vertical_range_points(dist, z, v_fov)]
    # Check in horizontal angle of FOV
    elif v_fov[1] == 2.0 and v_fov[0] == -24.9:
        return points[horizontal_range_points(x, y, h_fov)]
    else:
        h_points = horizontal_range_points(x, y, h_fov)
        v_points = vertical_range_points(dist, z, v_fov)
        return points[np.logical_and(h_points, v_points)]

def points_to_panorama(points, vertical_resolution, horizontal_resolution, vertical_fov, horizontal_fov, depth=False):
    """
    points: numpy array for point clouds
    vertical resolution & horizontal resolution(degree) : rotation degree depending on rotation Hz
    vertical fov, horizontal fov : Depending on LiDAR sensor
    depth : If True, normalize distance value and covert to depth map
    """

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    dist = np.sqrt(x**2 + y**2 + z**2)

    # Project into 2D
    x_img = np.arctan2(-y, x) / (horizontal_resolution * (np.pi / 180))
    y_img = -(np.arctan2(z, dist) / (vertical_resolution * (np.pi / 180)))

    # Filter points according to FOV
    x_img = FOV_filtering(x_img, x, y, z, dist, horizontal_fov, vertical_fov)
    y_img = FOV_filtering(y_img, x, y, z, dist, horizontal_fov, vertical_fov)
    dist = FOV_filtering(dist, x, y, z, dist, horizontal_fov, vertical_fov)

    # Offset for points to have positive value
    x_offset = horizontal_fov[0] / horizontal_resolution
    x_img = np.trunc(x_img - x_offset).astype(np.int32)
    y_offset = vertical_fov[1] / vertical_resolution
    y_fine_tune = 1
    y_img = np.trunc(y_img + y_offset + y_fine_tune).astype(np.int32)

    # Normalize
    if depth == True:
        dist = normalize_depth(dist, min_val=0, max_val=120)
    else:
        dist = normalize_disparity(dist, min_val=0, max_val=120)

    # array to img
    x_size = int(np.ceil((horizontal_fov[1] - horizontal_fov[0]) / horizontal_resolution))
    y_size = int(np.ceil((vertical_fov[1] - vertical_fov[0]) / vertical_resolution))

    img = np.zeros([y_size + 1, x_size + 1], dtype=np.uint8)
    img[y_img, x_img] = dist

    return img
if __name__ == "__main__":

    bin_path = '/mnt/nas/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/0000000089.bin'
    velodyne_v_fov, velodyne_h_fov = (-24.9, 2.0), (-180, 180)

    points = load_point_clouds(bin_path)
    print(points.shape)

    panorama_img = points_to_panorama(points, vertical_resolution=0.42, horizontal_resolution=0.35, vertical_fov=velodyne_v_fov, horizontal_fov=velodyne_h_fov, depth=True)

    # Display panorama image
    plt.subplots(1, 1, figsize=(13, 3))
    plt.title( "Result of Vertical FOV ({} , {}) & Horizontal FOV ({} , {})".format(velodyne_v_fov[0], velodyne_v_fov[1], velodyne_h_fov[0], velodyne_h_fov[1]))
    plt.imshow(panorama_img, cmap='Greys')
    plt.axis('off')
    plt.imsave('Panorama_depth.png', panorama_img)
    plt.show()
    print(panorama_img.shape)

    # Save all Panorama Images in Video
    velodyne_points = sorted(glob.glob('/mnt/nas/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/*.bin'))

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    vid = cv2.VideoWriter('depth_panorama.avi', fourcc, 25.0, (1030, 66), False)

    for point in velodyne_points:
        velodyne_point = load_point_clouds(point)
        img = points_to_panorama(velodyne_point, vertical_resolution=0.42, horizontal_resolution=0.35, vertical_fov=velodyne_v_fov, horizontal_fov=velodyne_h_fov, depth=True)
        vid.write(img)

    print('video saved')
    vid.release()

