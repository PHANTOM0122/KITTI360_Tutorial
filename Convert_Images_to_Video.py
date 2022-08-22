import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt

image_left = sorted(glob.glob('/mnt/nas/kitti360/KITTI-360/data_2d_raw/2013_05_28_drive_0000_sync/image_02/data_rgb/*.png'))
image_right = sorted(glob.glob('/mnt/nas/kitti360/KITTI-360/data_2d_raw/2013_05_28_drive_0000_sync/image_03/data_rgb/*.png'))

# print(len(image_left), len(image_right))
# print(image_left[0], image_right[0])

""" save panorama video """
fourcc = cv2.VideoWriter_fourcc(*'XVID')
vid = cv2.VideoWriter('pano_image.avi', fourcc, 25.0, (1400, 700), True)

for idx in range(int(len(image_left) / 20)):
    img_left = cv2.imread(image_left[idx])
    img_right = cv2.imread(image_right[idx])

    img_left = cv2.resize(img_left, (700, 700))
    img_right = cv2.resize(img_right, (700, 700))

    img = cv2.hconcat([img_left, img_right])
    # cv2.imshow("img_hconcat", img)
    # cv2.waitKey(0)
    # print(img.shape, img.dtype)
    print('Saved' + image_left[idx])
    vid.write(img)

print('video saved')
vid.release()
