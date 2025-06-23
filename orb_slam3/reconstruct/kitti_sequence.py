
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
    def __init__(self, sequence=None, frame_id=None, image=None, velo_pts=None):
        # Load sequence properties
        self.configs = sequence.configs
        if frame_id is not None:
            self.rgb_dir = sequence.rgb_dir
            self.velo_dir = sequence.velo_dir
            self.lbl2d_dir = sequence.lbl2d_dir
            self.lbl3d_dir = sequence.lbl3d_dir
        self.K = sequence.K_cam
        self.invK = sequence.invK_cam
        self.T_cam_velo = sequence.T_cam_velo
        self.online = sequence.online
        self.detector_2d = sequence.detector_2d
        self.detector_3d = sequence.detector_3d
        self.max_lidar_pts = self.configs.num_lidar_max
        self.min_lidar_pts = self.configs.num_lidar_min
        self.min_mask_area = self.configs.min_mask_area

        # Load image and LiDAR measurements
        self.frame_id = frame_id
        if frame_id is not None:
		    # kitti_raw : "%010d.lbl"
		    # kitti odometry 07 : "%06d.lbl"
            # mission system extracts : "%010d.lbl"
            rgb_file = os.path.join(self.rgb_dir, "{:010d}".format(frame_id) + ".png") #TODO
            self.velo_file = os.path.join(self.velo_dir, "{:010d}".format(frame_id) + ".bin") #TODO
            #print(rgb_file)
            #print(self.velo_file)
            self.img_bgr = cv2.imread(rgb_file)
        if image is not None:
            self.img_bgr = image
        self.img_rgb = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2RGB)
        self.img_h, self.img_w, _ = self.img_rgb.shape
        if frame_id is not None:
            self.velo_pts = load_velo_scan(self.velo_file)
        if velo_pts is not None:
            self.velo_pts = velo_pts

        self.instances = []

        #TODO move this to C++ class
        #R = self.rotation_matrix(-90, 0, 90)
        R = self.rotation_matrix(-90, 85, 0)
        #R = self.rotation_matrix(0, 0, 0) # kitti
        self.velo_pts[:, 0]-=5
        self.velo_pts[:, :3] = (R @ self.velo_pts[:, :3].T).T

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

    def get_colored_pts(self):
        velo_pts_cam = self.velo_pts[:, :3]
        # Project points to image
        #velo_pts_cam = (self.velo_pts[:, None, :3] * self.T_cam_velo[:3, :3]).sum(-1) + self.T_cam_velo[:3, 3]
        #velo_pts_cam = velo_pts_cam[(velo_pts_cam[:, 2] > 0), :] # FIXME

        img_h, img_w, _ = self.img_rgb.shape
        uv_hom = (velo_pts_cam[:, None, :] * self.K).sum(-1)
        uv = uv_hom[:, :2] / uv_hom[:, 2, None]
        in_fov = (uv[:, 0] > 0) & (uv[:, 0] < img_w) & (uv[:, 1] > 0) & (uv[:, 1] < img_h)
        uv = uv[in_fov].astype(np.int32)
        #velo_pts_cam_fov = velo_pts_cam[in_fov].astype(np.float32) # FIXME
        velo_pts_cam_fov = velo_pts_cam.astype(np.float32)
        colors_fov = self.img_rgb[uv[:, 1], uv[:, 0], :] / 255.
        return velo_pts_cam_fov, colors_fov

    def pixels_sampler(self, bbox_2d, mask):
        alpha = int(self.configs.downsample_ratio)
        expand_len = 5
        max_w, max_h = self.img_w - 1, self.img_h - 1
        # Expand the crop such that it will not be too tight
        l, t, r, b = list(bbox_2d.astype(np.int32))
        l = l - 5 if l > expand_len else 0
        t = t - 5 if t > expand_len else 0
        r = r + 5 if r < max_w - expand_len else max_w
        b = b + 5 if b < max_h - expand_len else max_h
        # Sample pixels inside the 2d box
        crop_H, crop_W = b - t + 1, r - l + 1
        hh = np.linspace(t, b, int(crop_H / alpha)).astype(np.int32)
        ww = np.linspace(l, r, int(crop_W / alpha)).astype(np.int32)
        crop_h, crop_w = hh.shape[0], ww.shape[0]
        hh = hh[:, None].repeat(crop_w, axis=1)
        ww = ww[None, :].repeat(crop_h, axis=0)
        sampled_pixels = np.concatenate([hh[:, :, None], ww[:, :, None]], axis=-1).reshape(-1, 2)
        vv, uu = sampled_pixels[:, 0], sampled_pixels[:, 1]
        non_surface = ~mask[vv, uu]
        sampled_pixels_non_surface = np.concatenate([uu[non_surface, None], vv[non_surface, None]], axis=-1)
        return sampled_pixels_non_surface

    def get_labels(self):
        labels_3d = self.detector_3d.make_prediction(self.velo_file).cpu().numpy()
        labels_2d = self.detector_2d.make_prediction(self.img_bgr)
        return labels_2d, labels_3d

    # TARIQ
    def build_vizbox(self, bbox3d, rgb=[1, 0, 0]):
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
        colors = [rgb for i in range(len(lines))]
        colors[4] = [1 - rgb[0], 1 - rgb[1], 1 - rgb[2]] # Paint upper front bar in opposite color
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(bbox3d),
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
        Generate a cuboid from KITTI label parameters
        Args:
                width: width of the bounding box
                height: height of the bounding box
                length: length of the bounding box
                location: location of the bottom of the (Y=0) bounding box
                rot_y: Rotation along Y
        """
        # Build box from KITTI3D label at origin. The car sits on the ground at Y=0
        # NOTE: Invert Y-axis because OpenCV goes down
        w, h, l = width, height, length
        front = np.asarray([[-w / 2, -h, l / 2], [+w / 2, -h, l / 2], [+w / 2, +0, l / 2], [-w / 2, +0, l / 2]])
        back = np.copy(front)
        back[:, 2] *= -1
        local_box = np.vstack((front, back))

        # Rotate and translate into global space
        # 'A car which is facing along the X-axis of the camera coordinate system
        # corresponds to rotation_y=0'. This is why we add PI/2 here...
        angle = rot_y + np.pi / 2 # FIXME mmdet3d 0.18.1 you add PI/2
        rot = Quaternion(axis=[0, 1, 0], radians=angle).rotation_matrix

        # angles for rotation around x, y, and z
        #angle_x = np.radians(60)
        #angle_y = np.radians(0)
        #angle_z = np.radians(0)
        # create individual quaternions for each rotation
        #quat_x = Quaternion(axis=[1, 0, 0], radians=angle_x)
        #quat_y = Quaternion(axis=[0, 1, 0], radians=angle_y)
        #quat_z = Quaternion(axis=[0, 0, 1], radians=angle_z)
        # combine rotations
        #rot = (quat_x * quat_y * quat_z).rotation_matrix

        return (rot @ local_box.T).T + location # FIXME

    def get_detections(self):
        # Get 3D Detection first
        t1 = get_time()
        # get lidar points here
        if self.online:
            if self.frame_id is not None:
                detections_3d = self.detector_3d.make_prediction(self.velo_file).cpu().numpy()
            else:
                #print(self.velo_pts)
                detections_3d = self.detector_3d.make_prediction(self.velo_pts).cpu().numpy()
        else:
            label_path_3d = os.path.join(self.lbl3d_dir,  "%06d.lbl" % self.frame_id)
            detections_3d = torch.load(label_path_3d)
        t2 = get_time()
        #print("3D detector takes %f seconds" % (t2 - t1))

        ############ Visualize results
        show_2d = True # FIXME
        show_3d = False # FIXME
        if show_3d:
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            vis_ctr = vis.get_view_control()
        ############

        # sort according to depth order
        depth_order = np.argsort(detections_3d[:, 0])  # x, y, z, w, l, h, heading 
        detections_3d = detections_3d[depth_order, :]
        for n in range(detections_3d.shape[0]):
            det_3d = detections_3d[n, :]
            trans, size, theta = det_3d[:3], det_3d[3:6], det_3d[6]   # FIXME mmdet3d 1.0.0rc6 -det_3d[6]+np.pi/2 , mmdet3d 0.18.1 det_3d[6]
            # Get SE(3) transformation matrix from trans and theta
            T_velo_obj = np.array([[np.cos(theta), 0, -np.sin(theta), trans[0]],
                                   [-np.sin(theta), 0, -np.cos(theta), trans[1]],
                                   [0, 1, 0, trans[2] + size[2] / 2],
                                   [0, 0, 0, 1]]).astype(np.float32)
            T_obj_velo = np.linalg.inv(T_velo_obj)
            x, y, z = list(trans)
            # Filter out points that are too far away from car centroid, with radius 3.0 meters
            r = 3.0
            nearby = (self.velo_pts[:, 0] > x - r) & (self.velo_pts[:, 0] < x + r) & \
                     (self.velo_pts[:, 1] > y - r) & (self.velo_pts[:, 1] < y + r) & \
                     (self.velo_pts[:, 2] > z - r) & (self.velo_pts[:, 2] < z + r)
            points_nearby = self.velo_pts[nearby]
            points_obj = (points_nearby[:, None, :3] * T_obj_velo[:3, :3]).sum(-1) + T_obj_velo[:3, 3]
           
            ######## Get the boxes
            if show_3d:
                #print(self.T_cam_velo)
                #t = (trans * self.T_cam_velo[:3, :3]).sum(-1) + self.T_cam_velo[:3, 3]
                t = T_velo_obj[:3, 3]
                #w1,l1,h1 = list(size)
                l1,w1,h1 = list(size)
          #      corners = self.transform_kitti_to_cuboid(w1, h1, l1, t, theta)
                corners = self.create_3d_bbox(l1, w1, h1, trans, theta)
                line_set = self.build_vizbox(corners, [0, 0, 1])
                #center = det_3d[0:3]
                #dim = det_3d[3:6]
                #yaw = np.zeros(3)
                #rot_axis = 2
                #yaw[rot_axis] = -det_3d[i,6]
                #rot_mat = geometry.get_rotation_matrix_from_xyz(yaw)
                #center[rot_axis] += dim[rot_axis] / 2  # lidar_bottom: bottom center to gravity center
                #box3d = geometry.OrientedBoundingBox(center, rot_mat, dim)
                #line_set = geometry.LineSet.create_from_oriented_bounding_box(box3d)
                #bbox_color = (0, 1, 0)
                #line_set.paint_uniform_color(bbox_color)
            #########

			# Further filter out the points that are outside the 3D bounding box
            l,w,h = list(size / 2) # FIXME w,l,h for mmdet3d 0.18.1, l,w,h for mmdet3d 1.0.0rc6
            w *= 1.1 # 
            l *= 1.1
            on_surface = (points_obj[:, 0] > -w) & (points_obj[:, 0] < w) & \
                         (points_obj[:, 1] > -h) & (points_obj[:, 1] < h) & \
                         (points_obj[:, 2] > -l) & (points_obj[:, 2] < l)
            pts_surface_velo = points_nearby[on_surface]
            # Sample from all the depth measurement
            N = pts_surface_velo.shape[0]
            if N > self.max_lidar_pts:
                sample_ind = np.linspace(0, N-1, self.max_lidar_pts).astype(np.int32)
                pts_surface_velo = pts_surface_velo[sample_ind, :]
            pts_surface_cam = (pts_surface_velo[:, None, :3] * self.T_cam_velo[:3, :3]).sum(-1) + self.T_cam_velo[:3, 3]
            T_cam_obj = self.T_cam_velo @ T_velo_obj
            T_cam_obj[:3, :3] *= l   # FIXME why is scaling orientation is better ?

            # Initialize detected instance
            instance = ForceKeyErrorDict()
            instance.T_cam_obj = T_cam_obj
            instance.scale = size
            instance.surface_points = pts_surface_cam.astype(np.float32)
            instance.num_surface_points = pts_surface_cam.shape[0]
            instance.is_front = True #T_cam_obj[2, 3] > 0.0 # FIXME
            instance.rays = None

            self.instances += [instance]

            ######### - TARIQ
            if show_3d:
                if instance.is_front:
                    vis.add_geometry(line_set)
            #########

        ######## Add LiDAR point cloud
        # must be put after adding geometries
        if show_3d:
            # Visualize the point cloud
            velo_pts_cam, colors = self.get_colored_pts()
            velo_pts_cam = velo_pts_cam.astype(np.float32)
            scene_pcd = o3d.geometry.PointCloud()
            scene_pcd.points = o3d.utility.Vector3dVector(velo_pts_cam)
            vis.add_geometry(scene_pcd)
            # Create a coordinate frame
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
            vis.add_geometry(frame)
            # must be put after adding geometries
            set_view(vis, dist=20, theta=0.)
            vis.run()
            vis.destroy_window()
        #########

        # Get 2D Detection and associate with 3D instances
        t3 = get_time()
        print(self.img_bgr.shape)
        if self.online:
            det_2d = self.detector_2d.make_prediction(self.img_bgr)
            if show_2d:
                self.detector_2d.visualize_result(self.img_bgr, "sdfsdf.png") # FIXME: TARIQ added this line
        else:
            label_path2d = os.path.join(self.lbl2d_dir, "%06d.lbl" % self.frame_id)
            det_2d = torch.load(label_path2d)
        t4 = get_time()
        print("2D detctor takes %f seconds" % (t4 - t3))

        img_h, img_w, _ = self.img_rgb.shape
        masks_2d = det_2d["pred_masks"]
        bboxes_2d = det_2d["pred_boxes"]

        # If no 2D detections, return right away
        if masks_2d.shape[0] == 0:
            print('no detection')
            return

        # Occlusion masks
        occ_mask = np.full([img_h, img_w], False, dtype=np.bool)
        prev_mask = None
        for instance in self.instances:
            if not instance.is_front:
                continue
            # Project LiDAR points to image plane
            surface_points = instance.surface_points
            pixels_homo = (surface_points[:, None, :] * self.K).sum(-1)
            pixels_uv = (pixels_homo[:, :2] / pixels_homo[:, 2, None])
            in_fov = (pixels_uv[:, 0] > 0) & (pixels_uv[:, 0] < img_w) & \
                     (pixels_uv[:, 1] > 0) & (pixels_uv[:, 1] < img_h)
            pixels_coord = pixels_uv[in_fov].astype(np.int32)
            # Check all the n 2D masks, and see how many projected points are inside them
            points_in_masks = [masks_2d[n, pixels_coord[:, 1], pixels_coord[:, 0]] for n in range(masks_2d.shape[0])]
            num_matches = np.array([points_in_mask[points_in_mask].shape[0] for points_in_mask in points_in_masks])
            max_num_matchess = num_matches.max()

            if max_num_matchess > pixels_coord.shape[0] * 0.5:
                n = np.argmax(num_matches)
                instance.mask = masks_2d[n, ...]
                instance.bbox = bboxes_2d[n, ...]

                if instance.mask[instance.mask].shape[0] > self.min_mask_area:
                    # Sample non-surface pixels
                    non_surface_pixels = self.pixels_sampler(instance.bbox, instance.mask)
                    if non_surface_pixels.shape[0] > 200:
                        sample_ind = np.linspace(0, non_surface_pixels.shape[0]-1, 200).astype(np.int32)
                        non_surface_pixels = non_surface_pixels[sample_ind, :]

                    pixels_inside_bb = np.concatenate([pixels_uv, non_surface_pixels], axis=0)
                    # rays contains all, but depth should only contain foreground
                    instance.rays = get_rays(pixels_inside_bb, self.invK).astype(np.float32)
                    instance.depth = surface_points[:, 2].astype(np.float32)

                # Create occlusion mask
                if prev_mask is not None:
                    occ_mask = occ_mask | prev_mask
                instance.occ_mask = occ_mask
                prev_mask = masks_2d[n, ...]


    # for only 2d detections
    def get_2d_detections(self):
        t3 = get_time()
        if self.online:
            det_2d = self.detector_2d.make_prediction(self.img_bgr)
            if True:
                self.detector_2d.visualize_result(self.img_bgr, "sdfsdf.png") # FIXME: TARIQ added this line
        else:
            label_path2d = os.path.join(self.lbl2d_dir, "%06d.lbl" % self.frame_id)
            det_2d = torch.load(label_path2d)
        t4 = get_time()
        print("2D detctor takes %f seconds" % (t4 - t3))

        img_h, img_w, _ = self.img_rgb.shape
        masks_2d = det_2d["pred_masks"]
        bboxes_2d = det_2d["pred_boxes"]
        labels_2d = det_2d["pred_labels"]
        scores_2d = det_2d["pred_scores"]

        # If no 2D detections, return right away
        if masks_2d.shape[0] == 0:
            print('no detection')
            return
        #else:
        #    print(det_2d)

        # Initialize detected instance
        for mask, bbox, label, score in zip(masks_2d, bboxes_2d, labels_2d, scores_2d): # use the zip() function to iterate over two lists
            instance = ForceKeyErrorDict()
            instance.mask = mask  # instance["mask"]
            instance.bbox = bbox  # instance["bbox"]
            instance.label = label  # instance["label"]
            instance.score = score  # instance["score"]
            self.instances.append(instance)


class KITIISequence:
    def __init__(self, data_dir, configs):
        self.root_dir = data_dir
		# The latter strings shouldn't start with a slash. If they start with a slash, then they're considered an "absolute path" and everything before them is discarded.
		# kitti_raw : "2011_09_30_drive_0018_sync/image_02/data/" + "2011_09_30_drive_0018_sync/velodyne_points/data/"
		# kitti odometry 07 : "image_2/" + "velodyne/"
        # mission system extracts : data_dir "/home/mrt/data/output/" + "png/" + "bin/"
        self.rgb_dir = os.path.join(data_dir, "/") #TODO
        self.velo_dir = os.path.join(data_dir, "/") #TODO
        self.calib_file = os.path.join(data_dir, "calib.txt")
        print(self.root_dir)
        print(self.rgb_dir)
        print(self.velo_dir)
        print(self.calib_file)
        self.load_calib()
        self.num_frames = len(os.listdir(self.rgb_dir))
        self.configs = configs
        self.online = self.configs.detect_online
        # Pre-stored label path
        self.lbl2d_dir = self.configs.path_label_2d
        self.lbl3d_dir = self.configs.path_label_3d
        if not self.online:
            assert self.lbl2d_dir is not None, print()
            assert self.lbl3d_dir is not None, print()
        # Detectors
        self.detector_2d, self.detector_3d = get_detectors(self.configs)
        self.current_frame = None
        self.detections_in_current_frame = None
        print("DONE !!!")

    def load_calib(self):
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
        self.current_frame = FrameWithLiDAR(self, frame_id)
        self.current_frame.get_detections()
        self.detections_in_current_frame = self.current_frame.instances
        return self.detections_in_current_frame

    def get_frame(self, image, velo_pts):
        #print("Hello from Python!")
        #print(image.shape)
        #print(velo_pts.shape)

        # image
        #cv_mat = cv2.normalize(image, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
        #cv_mat = cv2.cvtColor(cv_mat, cv2.COLOR_RGB2BGR)
        #cv2.imwrite('output_image.png', cv_mat)
        #cv2.imshow('Image', cv_mat)
        #cv2.waitKey(0)  # Wait for a key press before closing the image display
        #cv2.destroyAllWindows()  # Close the image display window

        # lidar
        #pcd = o3d.geometry.PointCloud()
        #pcd.points = o3d.utility.Vector3dVector(velo_pts[:, 0:3])
        #o3d.visualization.draw_geometries([pcd])

        self.current_frame = FrameWithLiDAR(self, None, image, velo_pts)
        self.current_frame.get_2d_detections() # FIXME, this should be get_detections() to include 3d detections
        self.detections_in_current_frame = self.current_frame.instances
        return self.detections_in_current_frame

    def get_labels_and_save(self):
        if not os.path.exists(self.lbl2d_dir):
            os.makedirs(self.lbl2d_dir)
        if not os.path.exists(self.lbl3d_dir):
            os.makedirs(self.lbl3d_dir)

        for frame_id in range(0, self.num_frames):
            frame = FrameWithLiDAR(self, frame_id)
            labels_2d, labels_3d = frame.get_labels()
		    # kitti_raw : "%010d.lbl"
		    # kitti odometry 07 : "%06d.lbl"
            # mission system extracts : "%010d.lbl"
            torch.save(labels_2d, os.path.join(self.lbl2d_dir, "%010d.lbl" % frame_id)) #TODO
            torch.save(labels_3d, os.path.join(self.lbl3d_dir, "%010d.lbl" % frame_id)) #TODO
            print("Finished saving frame %d" % frame_id)
