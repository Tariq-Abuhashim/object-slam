
import numpy as np
import torch
import mmcv

from mmcv.runner import load_checkpoint
from mmdet3d.models import build_model
from mmdet3d.apis import inference_detector, convert_SyncBN
from mmdet3d.core.points import BasePoints, get_points_type

import matplotlib.pyplot as plt

# Configure logging for the module
import logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d (%(funcName)s) - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def get_detector3d(configs):
    logger.debug("Setting detector3D")
    return Detector3D(configs)

class Detector3D(object):
    def __init__(self, configs):
        # set device
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        config = configs.Detector3D.config_path
        checkpoint = configs.Detector3D.weight_path

        if isinstance(config, str):
            config = mmcv.Config.fromfile(config)
        elif not isinstance(config, mmcv.Config):
            raise TypeError('config must be a filename or Config object, '
                            f'but got {type(config)}')

        config.model.pretrained = None
        convert_SyncBN(config.model)
        config.model.train_cfg = None
        self.model = build_model(config.model, test_cfg=config.get('test_cfg'))

        if checkpoint is not None:
            checkpoint = load_checkpoint(self.model, checkpoint, map_location='cpu')
            if 'CLASSES' in checkpoint['meta']:
                self.model.CLASSES = checkpoint['meta']['CLASSES']
            else:
                self.model.CLASSES = config.class_names
            if 'PALETTE' in checkpoint['meta']:  # 3D Segmentor
                self.model.PALETTE = checkpoint['meta']['PALETTE']
        self.model.cfg = config  # save the config in the model for convenience
        self.model.to(device)
        self.model.eval()

    def rotation_matrix(self, rx, ry, rz):
        rx = np.radians(rx)
        ry = np.radians(ry)
        rz = np.radians(rz)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        R = np.dot(Rz, np.dot(Ry, Rx))
        return R

    def load_kitti_lidar_bin(self, path):
        pointcloud = np.fromfile(path, dtype=np.float32)
        return pointcloud.reshape(-1, 4)  # Each point in the point cloud has x, y, z coordinates and intensity

    def print_min_max_dimensions(self, point_cloud):
        max_dimensions = np.max(point_cloud, axis=0)
        min_dimensions = np.min(point_cloud, axis=0)
        print('X:[', min_dimensions[0], ',', max_dimensions[0], ']')
        print('Y:[', min_dimensions[1], ',', max_dimensions[1], ']')
        print('Z:[', min_dimensions[2], ',', max_dimensions[2], ']')

    def make_prediction(self, velo):
        # read in kitti data file .bin format
        if isinstance(velo, str): # for velo as filename
            velo = self.load_kitti_lidar_bin(velo); # from filename to numpy points (dont need this for kitti)
            #print(f"velo shape {velo[0].size}")

        #velo[:, 0]-=5
        #R = self.rotation_matrix(-90, 90, 0) #FIXME remove this to C++ .bin making file
        #velo[:, :3] = (R @ velo[:, :3].T).T #FIXME remove this to C++ .bin making file
        #self.print_min_max_dimensions(velo)

        # Choose a specific dimension to plot (e.g., the first dimension)
        #dim_to_plot = velo[:, -1]
        # Create a figure and axis
        #fig, ax = plt.subplots()
        # Plot the dimension
        #ax.plot(dim_to_plot)
        # Show the plot
        #plt.show()

		# numpy array to LiDARPoints
        points_class = get_points_type('LIDAR')
        pcd = points_class(velo, points_dim=velo.shape[-1], attribute_dims=None)
        predictions, data = inference_detector(self.model, pcd)
        # Car's label is 0 in KITTI dataset
        labels = predictions[0]["labels_3d"]
        scores = predictions[0]["scores_3d"]
        valid_mask = (labels == 0) & (scores > 0.0) # FIXME was 0.0
        boxes = predictions[0]["boxes_3d"].tensor
        #print(f"boxes shape {boxes.shape}")

        return boxes[valid_mask]

