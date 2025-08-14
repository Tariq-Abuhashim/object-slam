# script.py
# a pybind11 test script
import numpy as np
import open3d as o3d
import cv2

import torch
import mmcv
from mmcv.runner import load_checkpoint
#from mmdet3d.models import build_model
#from mmdet3d.apis import inference_detector, convert_SyncBN
#from mmdet3d.core.points import BasePoints, get_points_type

class MyClass:
    def my_method(self, image, point_cloud):
        print("Hello from Python!")
        print(image.shape)
        print(point_cloud.shape)
        # image
        cv_mat = cv2.normalize(image, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
        cv_mat = cv2.cvtColor(cv_mat, cv2.COLOR_RGB2BGR)
        cv2.imwrite('output_image.png', cv_mat)
        #cv2.imshow('Image', cv_mat)
        #cv2.waitKey(0)  # Wait for a key press before closing the image display
        #cv2.destroyAllWindows()  # Close the image display window
        # lidar
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud)
        o3d.visualization.draw_geometries([pcd])