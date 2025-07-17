
import numpy as np
import os
import cv2
import torch
from reconstruct.loss_utils import get_rays, get_time
from reconstruct.utils import ForceKeyErrorDict, read_calib_file, load_velo_scan, set_view
from reconstruct import get_detectors
from pyquaternion import Quaternion

try:
    import open3d as o3d
    from open3d import geometry
except ImportError:
    raise ImportError(
        'Please run "pip install open3d" to install open3d first.')


class FrameWithLiDAR:
    """
    A class to handle frame data with LiDAR and camera information for object detection.
    
    This class processes a single frame containing RGB image and LiDAR point cloud data,
    performs 2D and 3D object detection, and associates detections between modalities.
    """


    def __init__(self, sequence, frame_id=None, image=None, velo_pts=None, visualize=False):
        """
        Initialize frame with sequence properties or direct data.
        
        Args:
            sequence: KITTISequence object containing configuration and paths
            frame_id: Frame identifier for loading from disk
            image: Direct image data (numpy array)
            velo_pts: Direct LiDAR point cloud data (numpy array)
        """
        if sequence is None:
            raise ValueError("Sequence object is required")
            
        self.configs = sequence.configs
        self.visualize = visualize

        self._setup_paths_and_detectors(sequence, frame_id)
        self._setup_camera_params(sequence)
        self._setup_lidar_params()

        # Load image and LiDAR measurements
        self.frame_id = frame_id
        self._load_image_data(frame_id, image)
        self._load_lidar_data(frame_id, velo_pts)
        
        self.instances = []

        # Apply coordinate transformation to LiDAR points
        #self._transform_lidar_coordinates()


    def _setup_paths_and_detectors(self, sequence, frame_id):
        """Setup file paths and detector references."""
        if frame_id is not None:
            self.rgb_dir = sequence.rgb_dir
            self.velo_dir = sequence.velo_dir
            self.lbl2d_dir = sequence.lbl2d_dir
            self.lbl3d_dir = sequence.lbl3d_dir
        
        self.online = sequence.online
        self.detector_2d = sequence.detector_2d
        self.detector_3d = sequence.detector_3d
        #self.frame_fmt = getattr(sequence, 'frame_fmt', "%06d")


    def _setup_camera_params(self, sequence):
        """Setup camera intrinsic and extrinsic parameters."""
        self.K = sequence.K_cam
        self.invK = sequence.invK_cam
        self.T_cam_velo = sequence.T_cam_velo


    def _setup_lidar_params(self):
        """Setup LiDAR processing parameters."""
        self.max_lidar_pts = self.configs.num_lidar_max
        self.min_lidar_pts = self.configs.num_lidar_min
        self.min_mask_area = self.configs.min_mask_area
        self.sample_radius = 3.0
        self.match_thresh = 0.5
        self.max_non_surface_pixels = 200

     
    def _load_image_data(self, frame_id, image):
        """Load image data from file or direct input."""
        if frame_id is not None:
            # Use 06d format consistently - can be made configurable if needed
            #rgb_file = os.path.join(self.rgb_dir, self.frame_fmt % frame_id + ".png")
            rgb_file = os.path.join(self.rgb_dir, f"{frame_id:06d}.png") #TODO  06d or 010d
            if not os.path.exists(rgb_file):
                raise FileNotFoundError(f"RGB file not found: {rgb_file}")
            self.img_bgr = cv2.imread(rgb_file)
            if self.img_bgr is None:
                raise ValueError(f"Failed to load image: {rgb_file}")
        elif image is not None:
            self.img_bgr = image
        else:
            raise ValueError("Either frame_id or image must be provided")
        
        self.img_rgb = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2RGB)
        self.img_h, self.img_w, _ = self.img_rgb.shape


    def _load_lidar_data(self, frame_id, velo_pts):
        """Load LiDAR data from file or direct input."""
        if frame_id is not None:
            self.velo_file = os.path.join(self.velo_dir, f"{frame_id:06d}.bin") #TODO  06d or 010d
            #self.velo_file = os.path.join(self.velo_dir, self.frame_fmt % frame_id + ".bin")
            if not os.path.exists(self.velo_file):
                raise FileNotFoundError(f"Velodyne file not found: {self.velo_file}")
            self.velo_pts = load_velo_scan(self.velo_file)
        elif velo_pts is not None:
            self.velo_pts = velo_pts
        else:
            raise ValueError("Either frame_id or velo_pts must be provided")

  
    def _transform_lidar_coordinates(self):
        """Apply coordinate transformation to LiDAR points."""
        # Apply rotation transformation
        #R = self._create_rotation_matrix(-90, 0, 90)
        #R = self._create_rotation_matrix(-90, 85, 0)
        R = self._create_rotation_matrix(0, 0, 0) # kitti
        self.velo_pts[:, 0] -= 5  # Translation
        self.velo_pts[:, :3] = (R @ self.velo_pts[:, :3].T).T


    def _create_rotation_matrix(self, rx, ry, rz):
        """
        Create rotation matrix from Euler angles.
        
        Args:
            rx, ry, rz: Rotation angles in degrees
            
        Returns:
            3x3 rotation matrix
        """
        rx, ry, rz = np.radians([rx, ry, rz])
        
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
        return Rz @ Ry @ Rx


    def get_colored_pts(self):
        """
        Project LiDAR points to image and get corresponding colors.
    
        Returns:
            tuple: (points_in_camera_coords, colors_from_image)
            Both arrays will have the same number of points (N x 3)
        """
        velo_pts_cam = (self.T_cam_velo[:3, :3] @ self.velo_pts[:, :3].T).T + self.T_cam_velo[:3, 3]
        velo_pts_cam = velo_pts_cam[velo_pts_cam[:, 2] > 0]

        img_h, img_w, _ = self.img_rgb.shape

        uv_hom = (velo_pts_cam @ self.K.T)
        uv = uv_hom[:, :2] / uv_hom[:, 2:3]
        in_fov = (uv[:, 0] > 0) & (uv[:, 0] < self.img_w) & (uv[:, 1] > 0) & (uv[:, 1] < self.img_h)

        uv = uv[in_fov].astype(np.int32)
        colors = self.img_rgb[uv[:, 1], uv[:, 0]] / 255.
        return velo_pts_cam[in_fov], colors


    def pixels_sampler(self, bbox_2d, mask):
        """
        Sample pixels from non-surface areas within bounding box.
        
        Args:
            bbox_2d: 2D bounding box coordinates
            mask: Binary mask of the object
            
        Returns:
            Array of sampled pixel coordinates
        """
        alpha = int(self.configs.downsample_ratio)
        #expand_len = 5
        #max_w, max_h = self.img_w - 1, self.img_h - 1
        # Expand the crop such that it will not be too tight
        l, t, r, b = bbox_2d.astype(np.int32)
        l, t = max(0, l - 5), max(0, t - 5)
        r, b = min(self.img_w - 1, r + 5), min(self.img_h - 1, b + 5)
        # Sample pixels inside the 2d box
        hh, ww = np.meshgrid(
            np.linspace(t, b, max(1, int((b - t + 1) / alpha))).astype(np.int32),
            np.linspace(l, r, max(1, int((r - l + 1) / alpha))).astype(np.int32),
            indexing='ij'
        )
        sampled_pixels = np.stack([ww, hh], axis=-1).reshape(-1, 2)
        vv, uu = sampled_pixels[:, 1], sampled_pixels[:, 0]
        non_surface = ~mask[vv, uu]
        return sampled_pixels[non_surface]


    def get_labels(self):
        """
        Get 2D and 3D labels using detectors.
        
        Returns:
            tuple: (2D labels, 3D labels)
        """
        if not hasattr(self, 'velo_file'):
            raise ValueError("velo_file not available for label generation")
            
        labels_3d = self.detector_3d.make_prediction(self.velo_file).cpu().numpy()
        labels_2d = self.detector_2d.make_prediction(self.img_bgr)
        return labels_2d, labels_3d


    # TARIQ
    def build_vizbox(self, corners, rgb=[1, 0, 0]):
        """Build a open3d.geometry.LineSet to represent a cuboid

        Args:
           bbox3d (np.array): Point set in form (Nx3), following canonical format
           rgb (list of float): color of cuboid

        Returns:
           line_set (open3d.geometry.LineSet)        
        """
        lines = [
            [0, 1], [1, 2], [2, 3], [3, 0], # Lower square
            [4, 5], [5, 6], [6, 7], [7, 4], # Upper square
            [0, 4], [1, 5], [2, 6], [3, 7]  # Vertical lines
        ]
        colors = [rgb] * len(lines)
        colors[4] = [1 - rgb[0], 1 - rgb[1], 1 - rgb[2]] # Paint upper front bar in opposite color
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(corners),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        return line_set


    # TARIQ
    def create_3d_bbox(self, length, width, height, position, yaw):
        yaw = yaw #+ np.pi / 2 
        # Create rotation matrix
        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Create the vertices of the bounding box
        x_corners = length / 2
        y_corners = width / 2
        z_corners = height

        corners = np.array([
            [x_corners, y_corners, 0],
            [x_corners, -y_corners, 0],
            [-x_corners, -y_corners, 0],
            [-x_corners, y_corners, 0],
            [x_corners, y_corners, z_corners],
            [x_corners, -y_corners, z_corners],
            [-x_corners, -y_corners, z_corners],
            [-x_corners, y_corners, z_corners]
        ])

        # Rotate and translate vertices
        corners = np.dot(corners, R.T) + np.array(position).reshape((1, 3))
        return corners


    # TARIQ
    def transform_kitti_to_cuboid(self, width, height, length, location, rot_y):
        """
        Transform KITTI 3D box params into cuboid corners.
        Args:
            width: box width (X-axis)
            height: box height (Y-axis)
            length: box length (Z-axis)
            location: bottom center of box in camera coords
            rot_y: rotation around camera Y-axis
        Returns:
            corners_3d: (8, 3) array of 3D box corners
        """

        w = width
        h = height
        l = length

        # 8 corners in object coords
        x_corners = np.array([ w/2,  w/2, -w/2, -w/2,
                               w/2,  w/2, -w/2, -w/2])
        y_corners = np.array([ 0,    -h,   -h,    0,
                               0,    -h,   -h,    0])
        z_corners = np.array([ l/2,  l/2,  l/2,  l/2,
                              -l/2, -l/2, -l/2, -l/2])

        # rotation around Y-axis
        R = np.array([
            [ np.cos(rot_y), 0, np.sin(rot_y)],
            [ 0,             1, 0           ],
            [-np.sin(rot_y), 0, np.cos(rot_y)]
        ])
        corners_3d = R @ np.vstack((x_corners, y_corners, z_corners))
        corners_3d = corners_3d + np.array(location).reshape(3, 1)
        return corners_3d.T


    # TARIQ
    def visualize_3d(self, cuboids, velo_pts_cam, colors):
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        for cuboid in cuboids:
            vis.add_geometry(cuboid)

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(velo_pts_cam.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))
        vis.add_geometry(cloud)

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        vis.add_geometry(frame)

        set_view(vis, dist=20, theta=0.0)
        vis.run()
        vis.destroy_window()


    def _process_detections_3d(self):
        """Run 3D detector and return array of detections."""
        t1 = get_time()
        # get lidar points here
        if self.online:
            #if hasattr(self, "velo_file"):  # FIXME suggested replacement
            if self.frame_id is not None:   
                print(f"velo_file = {self.velo_file}")
                detections = self.detector_3d.make_prediction(self.velo_file).cpu().numpy()
            else:
                #print(self.velo_pts)
                detections = self.detector_3d.make_prediction(self.velo_pts).cpu().numpy()
        else:
            #label_path = os.path.join(self.lbl3d_dir, self.frame_fmt % self.frame_id + ".lbl") # FIXME suggested replacement
            label_path_3d = os.path.join(self.lbl3d_dir,  "%06d.lbl" % self.frame_id)
            detections = torch.load(label_path_3d)
        t2 = get_time()
        print("3D detector took %.3f seconds" % (t2 - t1))
        
        if detections.ndim == 1:
            detections = detections[None, :]
            
        # Sort by depth
        detections = detections[np.argsort(detections[:, 0])]
        return detections


    def transform_points(self, points, T):
        """
        Transform Nx3 points with a 4x4 matrix T, on GPU.
        """
        points_homo = torch.cat([points[:, :3], torch.ones((points.shape[0], 1), device=points.device)], dim=1)
        points_trans = points_homo @ T.T
        return points_trans[:, :3]


    def _create_instance_from_3d_box_cpu(self, det_3d):
        """
        Create instance dict and optionally LineSet visualization from a 3D detection.
        det: [x, y, z, w, l, h, heading]
        On CPU
        """
        # FIXME mmdet3d 1.0.0rc6 -det_3d[6]+np.pi/2 , mmdet3d 0.18.1 det_3d[6]
        center, size, heading = det_3d[:3], det_3d[3:6], -det_3d[6]+np.pi/2
        
        # Get SE(3) transformation matrix from trans and theta
        T_velo_obj = np.array([[np.cos(heading),  0, -np.sin(heading), center[0]],
                               [-np.sin(heading), 0, -np.cos(heading), center[1]],
                               [0, 1, 0, center[2] + size[2] / 2],
                               [0, 0, 0, 1]]).astype(np.float32)
        
        # Filter out points that are too far away from car centroid, with radius 3.0 meters
        radius = self.sample_radius
        x, y, z = center
        nearby = ( (self.velo_pts[:, 0] > x - radius) & (self.velo_pts[:, 0] < x + radius) &
                   (self.velo_pts[:, 1] > y - radius) & (self.velo_pts[:, 1] < y + radius) &
                   (self.velo_pts[:, 2] > z - radius) & (self.velo_pts[:, 2] < z + radius) )
        points_nearby = self.velo_pts[nearby]
        T_obj_velo = np.linalg.inv(T_velo_obj)
        points_obj = (points_nearby[:, None, :3] * T_obj_velo[:3, :3]).sum(-1) + T_obj_velo[:3, 3]
        
        # Further filter out the points that are outside the 3D bounding box
        # size = [w, l, h]  # for mmdet3d 1.0.0rc6
        # or
        # size = [l, w, h]  # for mmdet3d 0.18.1
        l,w,h = list(size/2.0)
        #w *= 1.1 #
        #l *= 1.1
        in_box_mask = ( (points_obj[:, 0] > -w) & (points_obj[:, 0] < w) &
                        (points_obj[:, 1] > -h) & (points_obj[:, 1] < h) &
                        (points_obj[:, 2] > -l) & (points_obj[:, 2] < l) )
        pts_surface_velo = points_nearby[in_box_mask]
        if pts_surface_velo.shape[0] == 0:
            return None, None  # or return a minimal default instance
            
        # Sample from all the depth measurement
        N = pts_surface_velo.shape[0]
        if N > self.max_lidar_pts:
            sample_ind = np.linspace(0, N-1, self.max_lidar_pts).astype(np.int32)
            pts_surface_velo = pts_surface_velo[sample_ind, :]
        
        # Transform to camera frame
        points_cam = (pts_surface_velo[:, None, :3] * self.T_cam_velo[:3, :3]).sum(-1) + self.T_cam_velo[:3, 3]
        
        T_cam_obj = self.T_cam_velo @ T_velo_obj
        T_cam_obj[:3, :3] *= l  # FIXME we should not scale rotation
        
        # Initialize detected instance
        instance = ForceKeyErrorDict()
        instance.T_cam_obj = T_cam_obj
        instance.scale = size
        instance.surface_points = points_cam.astype(np.float32)
        instance.num_surface_points = points_cam.shape[0]
        instance.is_front = T_cam_obj[2, 3] > 0.0 # FIXME set to True to see all the scan objects
        instance.rays = None
        
        # Get the box
        line_set = None # FIXME is this okay?
        if self.visualize: # was if show_3d
            R = self.T_cam_velo[:3, :3]
            tvec = self.T_cam_velo[:3, 3]
            t = R @ center + tvec
            w1,l1,h1 = list(size)
            corners = self.transform_kitti_to_cuboid(w1, h1, l1, t, heading)
            line_set = self.build_vizbox(corners, [0, 0, 1])
        
        return instance, line_set


    def _create_instance_from_3d_box_gpu(self, det_3d):
        """
        Create instance dict and optionally LineSet visualization from a 3D detection.
        det: [x, y, z, w, l, h, heading]
        On GPU
        """
        # FIXME mmdet3d 1.0.0rc6 -det_3d[6]+np.pi/2 , mmdet3d 0.18.1 det_3d[6]
        center, size, heading = det_3d[:3], det_3d[3:6], -det_3d[6]+np.pi/2
        
        # Get SE(3) transformation matrix from trans and theta
        T_velo_obj = np.array([[np.cos(heading),  0, -np.sin(heading), center[0]],
                               [-np.sin(heading), 0, -np.cos(heading), center[1]],
                               [0, 1, 0, center[2] + size[2] / 2],
                               [0, 0, 0, 1]]).astype(np.float32)
        
        # move to GPU
        T_velo_obj_torch = torch.tensor(T_velo_obj, device="cuda")
        T_obj_velo_torch = torch.linalg.inv(T_velo_obj_torch)
        size_torch = torch.tensor(size, device="cuda")
        center_torch = torch.tensor(center, device="cuda", dtype=torch.float32)
        
        # Filter out points that are too far away from car centroid, with radius 3.0 meters
        radius = self.sample_radius
        lower = center_torch - radius
        upper = center_torch + radius
          
        # GPU-version using torch : This is orders of magnitude faster for millions of points.
        nearby = ( (self.velo_pts_torch[:, 0] >= lower[0]) &
                   (self.velo_pts_torch[:, 0] <= upper[0]) &
                   (self.velo_pts_torch[:, 1] >= lower[1]) &
                   (self.velo_pts_torch[:, 1] <= upper[1]) &
                   (self.velo_pts_torch[:, 2] >= lower[2]) &
                   (self.velo_pts_torch[:, 2] <= upper[2]) )
        points_nearby = self.velo_pts_torch[nearby] #FIXME
        points_obj = self.transform_points(points_nearby, T_obj_velo_torch) #GPU
        
        # Further filter out the points that are outside the 3D bounding box
        # size = [w, l, h]  # for mmdet3d 1.0.0rc6
        # or
        # size = [l, w, h]  # for mmdet3d 0.18.1
        l,w,h = size_torch / 2.0
        #w *= 1.1
        #l *= 1.1
        in_box_mask = ( (points_obj[:, 0] > -w) & (points_obj[:, 0] < w) &
                        (points_obj[:, 1] > -h) & (points_obj[:, 1] < h) &
                        (points_obj[:, 2] > -l) & (points_obj[:, 2] < l) )
        pts_surface_velo = points_nearby[in_box_mask]
        if pts_surface_velo.shape[0] == 0:
            return None, None  # or return a minimal default instance
            
        # Sample from all the depth measurement
        N = pts_surface_velo.shape[0]
        if N > self.max_lidar_pts:
            #sample_ind = torch.linspace(0, N-1, self.max_lidar_pts, device="cuda").long()
            sample_ind = torch.randperm(N, device="cuda")[:self.max_lidar_pts]
            pts_surface_velo = pts_surface_velo[sample_ind]
        
        # Transform to camera frame
        T_cam_velo_torch = torch.tensor(self.T_cam_velo, device="cuda", dtype=torch.float32)
        points_cam = self.transform_points(pts_surface_velo, T_cam_velo_torch)
        points_cam = points_cam.cpu().numpy()
        
        T_cam_obj = self.T_cam_velo @ T_velo_obj
        T_cam_obj[:3, :3] *= l.cpu().numpy() # FIXME we should not scale rotation
        
        # Initialize detected instance
        instance = ForceKeyErrorDict()
        instance.T_cam_obj = T_cam_obj
        instance.scale = size
        instance.surface_points = points_cam.astype(np.float32)
        instance.num_surface_points = points_cam.shape[0]
        instance.is_front = T_cam_obj[2, 3] > 0.0 # FIXME set to True to see all the scan objects
        instance.rays = None
        
        # Get the box
        line_set = None # FIXME is this okay?
        if self.visualize: # was if show_3d
            R = self.T_cam_velo[:3, :3]
            tvec = self.T_cam_velo[:3, 3]
            t = R @ center + tvec
            w1,l1,h1 = list(size)
            corners = self.transform_kitti_to_cuboid(w1, h1, l1, t, heading)
            line_set = self.build_vizbox(corners, [0, 0, 1])
        
        return instance, line_set

       
    def visualize_3d(self, cuboids, velo_pts_cam, colors):
        """Visualize the point cloud"""
        
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis_ctr = vis.get_view_control()
        
        for cuboid in cuboids:
            # if instance.is_front:  # FIXME where is this condition acheived now?
            vis.add_geometry(cuboid)
        
        scene_pcd = o3d.geometry.PointCloud()
        scene_pcd.points = o3d.utility.Vector3dVector(velo_pts_cam.astype(np.float32))
        scene_pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))
        vis.add_geometry(scene_pcd)
        
        # Create a coordinate frame
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(frame)
        
        # must be put after adding geometries
        set_view(vis, dist=20, theta=0.)
        vis.run()
        vis.destroy_window()


    def _process_detections_2d(self):
        """Run 2D detector and return array of detections."""
        t3 = get_time()
        print(f"[_process_detections_2d] Image shape: {self.img_bgr.shape}")
        if self.online:
            det_2d = self.detector_2d.make_prediction(self.img_bgr)
            if self.visualize:
                self.detector_2d.visualize_result(self.img_bgr, "maskrcnn_debug.png")
        else:
            #label_path = os.path.join(self.lbl2d_dir, self.frame_fmt % self.frame_id + ".lbl") #FIXME proposed change
            label_path2d = os.path.join(self.lbl2d_dir, "%06d.lbl" % self.frame_id)
            det_2d = torch.load(label_path2d)
        t4 = get_time()
        print("2D detctor took %.3f seconds" % (t4 - t3))
        return det_2d


    def _associate_detections_2d_3d(self, det_2d):
        """Match 2D masks to 3D surface points."""
        masks_2d = det_2d["pred_masks"]
        bboxes_2d = det_2d["pred_boxes"]
        
        # Occlusion masks
        img_h, img_w, _ = self.img_rgb.shape
        occ_mask = np.zeros((img_h, img_w), dtype=bool)
        prev_mask = None
        
        for instance in self.instances:
            if not instance.is_front:
                continue
            
            # Project LiDAR points to image plane
            surface_points = instance.surface_points
            pixels_homo = (surface_points[:, None, :] * self.K).sum(-1)
            pixels_uv = (pixels_homo[:, :2] / pixels_homo[:, 2, None])
            in_fov = ( (pixels_uv[:, 0] > 0) & (pixels_uv[:, 0] < img_w) &
                       (pixels_uv[:, 1] > 0) & (pixels_uv[:, 1] < img_h) )
            pixels_coord = pixels_uv[in_fov].astype(np.int32)
            if pixels_coord.shape[0] == 0:
                continue
                
            # Check all the n 2D masks, and see how many projected points are inside them
            points_in_masks = [masks_2d[n, pixels_coord[:, 1], pixels_coord[:, 0]] for n in range(masks_2d.shape[0])]
            num_matches = np.array([points_in_mask[points_in_mask].shape[0] for points_in_mask in points_in_masks])
            max_num_matchess = num_matches.max()
            
            if max_num_matchess > pixels_coord.shape[0] * self.match_thresh: # 0.5
                n = np.argmax(num_matches)
                mask = masks_2d[n]
                bbox = bboxes_2d[n]
                instance.mask = mask
                instance.bbox = bbox
                
                if mask.sum() > self.min_mask_area:
                    # Sample non-surface pixels
                    non_surface_pixels = self.pixels_sampler(bbox, mask)
                    if non_surface_pixels.shape[0] > self.max_non_surface_pixels:
                        sample_ind = np.linspace(
                              0, non_surface_pixels.shape[0]-1,
                              self.max_non_surface_pixels
                        ).astype(np.int32)
                        non_surface_pixels = non_surface_pixels[sample_ind]
                        
                    pixels_inside_bb = np.concatenate([pixels_uv, non_surface_pixels], axis=0)
                    # rays contains all, but depth should only contain foreground
                    instance.rays = get_rays(pixels_inside_bb, self.invK).astype(np.float32)
                    instance.depth = surface_points[:, 2].astype(np.float32)
                    
                # Create occlusion mask
                if prev_mask is not None:
                    occ_mask = occ_mask | prev_mask
                instance.occ_mask = occ_mask
                prev_mask = mask
        
        
    def get_detections(self):
        """Main detection pipeline."""
        if self.velo_pts is None:
            return
        
        # Convert to GPU tensor (FIXME only apply this if sorting points on GPU)
        # in case you have reflectance or intensity as 4th channel (use :3)
        #self.velo_pts_torch = torch.tensor( self.velo_pts[:, :3], dtype=torch.float32, device="cuda")
            
        # Run 3D detection
        detections_3d = self._process_detections_3d()

        # sort according to depth order
        t1 = get_time()
        depth_order = np.argsort(detections_3d[:, 0])  # x, y, z, w, l, h, heading 
        detections_3d = detections_3d[depth_order, :]
        cuboids = []
        for det in detections_3d:
            # cpu is faster than gpu, unless there are millions of points
            instance, cuboid = self._create_instance_from_3d_box_cpu(det)
            if instance:
                self.instances.append(instance)
            if self.visualize:
                cuboids.append(cuboid)
        t2 = get_time()
        print("Cropping, masking and transforming lidar points took %.3f seconds" % (t2 - t1))
        
        # Visualize point cloud and boxes
        if self.visualize and cuboids:
            velo_pts_cam, colors = self.get_colored_pts()
            self.visualize_3d(cuboids, velo_pts_cam, colors)

        # Get 2D Detection and associate with 3D instances
        det_2d = self._process_detections_2d()
        
        # Associate 2D and 3D detections
        if det_2d is not None:
            self._associate_detections_2d_3d(det_2d)
        

    def get_2d_detections(self):
        """Run 2D detection ONLY and populate self.instances with mask, bbox, label, and score."""
        # Get 2D Detection and associate with 3D instances
        det_2d = self._process_detections_2d()
        
        # Associate 2D and 3D detections
        if det_2d is None:
            print("2D detector returned None.")
            return

        masks_2d = det_2d.get("pred_masks")
        bboxes_2d = det_2d.get("pred_boxes")
        labels_2d = det_2d.get("pred_labels")
        scores_2d = det_2d.get("pred_scores")

        # If no 2D detections, return right away
        if masks_2d is None or masks_2d.shape[0] == 0:
            print("No 2D detections found.")
            return

        # Initialize detected instance
        # use the zip() function to iterate over two or more lists
        for mask, bbox, label, score in zip(masks_2d, bboxes_2d, labels_2d, scores_2d):
            instance = ForceKeyErrorDict()
            instance.mask = mask  # instance["mask"]
            instance.bbox = bbox  # instance["bbox"]
            instance.label = label  # instance["label"]
            instance.score = score  # instance["score"]
            self.instances.append(instance)


class KITIISequence:
    def __init__(self, data_dir, configs):
        self.root_dir = data_dir
        self.configs = configs
        
		# The latter strings shouldn't start with a slash. 
		# If they start with a slash, then they're considered an "absolute path" and everything before them is discarded.
		# kitti_raw : "2011_09_30_drive_0018_sync/image_02/data/" + "2011_09_30_drive_0018_sync/velodyne_points/data/"
		# kitti odometry 07 : "image_2/" + "velodyne/"
        # mission system extracts : data_dir "/home/mrt/data/output/" + "png/" + "bin/"
        self.rgb_dir = os.path.join(data_dir, "image_2/") #TODO
        self.velo_dir = os.path.join(data_dir, "velodyne/") #TODO
        self.calib_file = os.path.join(data_dir, "calib.txt")
        print(f"[KITIISequence] Root: {self.root_dir}")
        print(f"[KITIISequence] RGB: {self.rgb_dir}")
        print(f"[KITIISequence] Velodyne: {self.velo_dir}")
        print(f"[KITIISequence] Calib: {self.calib_file}")
        
        # Load calibration and detection modules
        self._load_calib()
        self.detector_2d, self.detector_3d = get_detectors(self.configs)
        
        # Support offline (pre-labeled) or online detection
        self.online = configs.detect_online
        self.lbl2d_dir = configs.path_label_2d
        self.lbl3d_dir = configs.path_label_3d
        if not self.online:
            assert self.lbl2d_dir is not None, print("2D label path required in offline mode")
            assert self.lbl3d_dir is not None, print("3D label path required in offline mode")
        
        self.num_frames = len(os.listdir(self.rgb_dir))
        self.current_frame = None
        self.detections_in_current_frame = None
        
        print("KITTI Sequence initialization complete.")


    def _load_calib(self):
        """Load and compute intrinsic and extrinsic calibration parameters."""
        # Load the calibration file
        filedata = read_calib_file(self.calib_file)

        # Load projection matrix P_cam2_cam0, and compute perspective instrinsics K of cam2
        P_cam2_cam0 = np.reshape(filedata['P2'], (3, 4))
        self.K_cam = P_cam2_cam0[0:3, 0:3].astype(np.float32)
        self.invK_cam = np.linalg.inv(self.K_cam).astype(np.float32)

        # Load the transfomration from T_cam0_velo, and compute the transformation T_cam2_velo
        T_cam0_velo, T_cam2_cam0 = np.eye(4), np.eye(4)
        T_cam0_velo[:3, :] = np.reshape(filedata['Tr'], (3, 4))
        T_cam2_cam0[0, 3] = P_cam2_cam0[0, 3] / P_cam2_cam0[0, 0]
        self.T_cam_velo = T_cam2_cam0.dot(T_cam0_velo).astype(np.float32)


    def get_frame_by_id(self, frame_id):
        """Load a frame by index and run detections."""
        frame = FrameWithLiDAR(self, frame_id, None, None, True)
        frame.get_detections()
        self.current_frame = frame
        self.detections_in_current_frame = frame.instances
        return frame.instances


    def get_frame(self, image, velo_pts):
        """Use external image and lidar points to generate detections."""

        # inspect and visualize input data
        if self.visualize:
            # image input
            cv_mat = cv2.normalize(image, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
            cv_mat = cv2.cvtColor(cv_mat, cv2.COLOR_RGB2BGR)
            cv2.imwrite('output_image.png', cv_mat)
            cv2.imshow('Image', cv_mat)
            cv2.waitKey(0)  # Wait for a key press before closing the image display
            cv2.destroyAllWindows()  # Close the image display window

            # lidar input
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(velo_pts[:, 0:3])
            o3d.visualization.draw_geometries([pcd])

        frame = FrameWithLiDAR(self, None, image, velo_pts)
        #frame.get_2d_detections() # FIXME, this should be get_detections() to include 3d detections
        frame.get_detections() # returns 2d+3d detections
        self.current_frame = frame
        self.detections_in_current_frame = frame.instances
        return frame.instances


    def get_labels_and_save(self):
        """Run detection and save labels for all frames (offline mode)."""
        os.makedirs(self.lbl2d_dir, exist_ok=True)
        os.makedirs(self.lbl3d_dir, exist_ok=True)

        for frame_id in range(self.num_frames):
            frame = FrameWithLiDAR(self, frame_id)
            labels_2d, labels_3d = frame.get_labels()
		    # kitti_raw : "%010d.lbl"
		    # kitti odometry 07 : "%06d.lbl"
            # mission system extracts : "%010d.lbl"
            save_2d_path = os.path.join(self.lbl2d_dir, f"{frame_id:06d}.lbl") #TODO 06d or 010d
            save_3d_path = os.path.join(self.lbl3d_dir, f"{frame_id:06d}.lbl") #TODO
            torch.save(labels_2d, save_2d_path) 
            torch.save(labels_3d, save_3d_path)
            print(f"Saved labels for frame {frame_id}")
            
