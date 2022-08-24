"""
Projecting point clouds into Image
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt

def load_point_clouds(bin_path):
    """
    Load point clouds from raw file
    .bin file to numpy array
    """
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4) # (x,y,z) coordinate + reflection
    return points[:,:3] # Return only x,y,z coordinate

def depth_color(val, min_distance=0, max_distance=120):
    """
    Print Color(HSV's H value) corresponding to distance(m)
    From Close(red) to Far(Blue)
    """
    np.clip(val, 0, max_distance, out=val)
    return (((val-min_distance)/(max_distance-min_distance)) * 120).astype(np.uint8)

def horizontal_range_points(m, n, fov):
    return np.logical_and(np.arctan2(n,m) > (-fov[1] * np.pi / 180), np.arctan2(n,m) < (-fov[0] * np.pi / 180))

def vertical_range_points(m, n, fov):
    return np.logical_and(np.arctan2(n, m) < (fov[1] * np.pi / 180), np.arctan2(n, m) > (fov[0] * np.pi / 180))

def in_range_points(points, size):
    """ extract in-range points """
    return np.logical_and(points > 0, points < size)

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

def filter_points(points, v_fov, h_fov):

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)

    # Setting according to camera's FOV
    if h_fov[0] < -90:
        h_fov = (-90,) + h_fov[1:]
    if h_fov[1] > 90:
        h_fov = h_fov[:1] + (90,)

    x_lim = FOV_filtering(x, x, y, z, dist, h_fov, v_fov)[:, None]
    y_lim = FOV_filtering(y, x, y, z, dist, h_fov, v_fov)[:, None]
    z_lim = FOV_filtering(z, x, y, z, dist, h_fov, v_fov)[:, None]

    # Stack arrays in sequence horizontally
    xyz_ = np.hstack((x_lim, y_lim, z_lim))
    xyz_ = xyz_.T

    # stack (1,n) arrays filled with the number 1
    one_mat = np.full((1, xyz_.shape[1]), 1)
    xyz_ = np.concatenate((xyz_, one_mat), axis=0)

    # need dist info for points color
    dist_lim = FOV_filtering(dist, x, y, z, dist, h_fov, v_fov)
    color = depth_color(dist_lim, 0, 70)

    return xyz_, color

def get_calibration_matrix_velodyne_to_cam(filepath):
    """
    Get calibration matrix converting Velodyne coordinates into Camera coordinates
    Rotatiton matrix : 3x3, Transition matrix : 3x3
    All information is provided with KITTI360 Dataset
    """
    # Get matrix
    with open(filepath, "r") as f:
        file = f.readlines()

        for line in file:
            (key, val) = line.split(':', 1)
            if key == 'R':
                R = np.fromstring(val, sep=' ')
                R = R.reshape(3, 3)
            if key == 'T':
                T = np.fromstring(val, sep=' ')
                T = T.reshape(3, 1)
    return R, T

def get_calibration_matrix_cam_to_Image(filepath, mode):
    """
    Assuming that the Images are rectified
    """
    with open(filepath, "r") as f:
        file = f.readlines()

        for line in file:
            (key, val) = line.split(':', 1)
            if key == ('P_rect_' + mode):
                P_ = np.fromstring(val, sep=' ')
                P_ = P_.reshape(3, 4)
                # erase 4th column ([0,0,0])
                P_ = P_[:3, :3]
    return P_

def velodyne_points_to_Image(points, v_fov, h_fov, vc_path, cc_path, mode='02'):
    """
    R_ : Rotation matrix(velodyne -> camera)
    T_ : Transition matrix(volodyne -> camera)
    RT_ : Rotation & Transition matrix(velodyne coordniate -> camera)
    P_ : Projection matrix(camera coordinage(3D) -> Image plane points(2D))
    xyz_v(points in velodyne coordinate) - 3D points corresponding to h, v FOV in the velodyne coordinates
    xyz_c(points in camera coordinate) - 3D points corresponding to h, v FOV in the camera coordinates
    c_ - color value(HSV's Hue) corresponding to distance(m)
    xy_i - points corresponding to h, v FOV in the image(pixel) coordinates before scale adjustment [s_1*x_1 , s_2*x_2 , .. ]
    ans  - points corresponding to h, v FOV in the image(pixel) coordinates  [x_1 , x_2 , .. ]
    """
    # Get matrix
    R_, T_ = get_calibration_matrix_velodyne_to_cam(vc_path)
    P_ = get_calibration_matrix_cam_to_Image(cc_path, mode)
    RT_ = np.concatenate((R_, T_), axis = 1)

    # Get Velodyne points in Camera's FOV
    xyz_v, c_ = filter_points(points, v_fov, h_fov)

    # Convert velodyne coordinate into camera coordinate
    for i in range(xyz_v.shape[1]):
        xyz_v[:3, i] = np.matmul(RT_, xyz_v[:, i])
    xyz_c = np.delete(xyz_v, 3, axis=0)

    # Convert Camera coordinate (X_c, Y_c, Z_c) into image(pixel) coordinate(x,y)
    for i in range(xyz_c.shape[1]):
        xyz_c[:,i] = np.matmul(P_, xyz_c[:,i])
    xy_i = xyz_c[::] / xyz_c[::][2]
    ans = np.delete(xy_i, 2, axis=0)

    width = 1408
    height = 376
    w_range = in_range_points(ans[0], width)
    h_range = in_range_points(ans[1], height)

    ans_x = ans[0][np.logical_and(w_range,h_range)][:,None].T
    ans_y = ans[1][np.logical_and(w_range,h_range)][:,None].T
    c_ = c_[np.logical_and(w_range,h_range)]

    ans = np.vstack((ans_x, ans_y))

    return ans, c_

def plot_projection(points, color, image, mode='rgb'):
    """
    Print converted velodyne points into camera Image
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Circle points in Image
    for i in range(points.shape[1]):
        cv2.circle(hsv_image, (np.int32(points[0][i]), np.int32(points[1][i])), 2, (int(color[i]), 255, 255), -1)

    if mode == 'rgb':
        return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
    elif mode == 'bgr':
        return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)


if __name__ == "__main__":

    bin_path = '/mnt/nas/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data/0000000000.bin'
    v2c_filepath = 'calibration/velo_to_cam.txt'
    c2c_filepath = 'calibration/cam_to_cam.txt'
    velodyne_v_fov, velodyne_h_fov = (-24.9, 2.0), (-180, 180)

    image_type = 'color'  # 'grayscale' or 'color' image
    mode = '00' if image_type == 'grayscale' else '02'  # image_00 = 'grayscale image' , image_02 = 'color image'

    image = cv2.imread('front.png')
    points = load_point_clouds(bin_path)
    print(points.shape)

    plt.subplots(1,1, figsize = (13,3) )
    plt.title("Original image")
    plt.imshow(image)

    ans, c_ = velodyne_points_to_Image(points, v_fov=(-24.9, 2.0), h_fov=(-45, 45), vc_path=v2c_filepath, cc_path=c2c_filepath, mode=mode)
    print(ans.shape)
    image = plot_projection(points=ans, color=c_, image=image)

    plt.subplots(1,1, figsize = (13,3))
    plt.title("Velodyne points to camera image Result")
    plt.imshow(image)
    plt.show()