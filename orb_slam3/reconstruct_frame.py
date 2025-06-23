
from reconstruct.utils import color_table, set_view, get_configs, get_decoder
from reconstruct.loss_utils import get_time
from reconstruct.kitti_sequence_ import KITIISequence
from reconstruct.optimizer import Optimizer, MeshExtractor

import open3d as o3d
import argparse


def config_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, required=True, help='path to config file')
    parser.add_argument('-d', '--sequence_dir', type=str, required=True, help='path to kitti sequence')
    parser.add_argument('-i', '--frame_id', type=int, required=True, help='frame id')
    return parser


# 2D and 3D detection and data association
if __name__ == "__main__":
    parser = config_parser()
    args = parser.parse_args()

    configs = get_configs(args.config)
    decoder = get_decoder(configs)
    kitti_seq = KITIISequence(args.sequence_dir, configs)
    optimizer = Optimizer(decoder, configs)
    detections = kitti_seq.get_frame_by_id(args.frame_id)

    # start reconstruction
    objects_recon = []
    start = get_time()
    for det in detections:
        # No observed rays, possibly not in fov
        if det.rays is None:
            continue
        print("%d depth samples on the car, %d rays in total" % (det.num_surface_points, det.rays.shape[0]))
        obj = optimizer.reconstruct_object(det.T_cam_obj, det.surface_points, det.rays, det.depth)
        # in case reconstruction fails
        if obj.code is None:
            continue
        objects_recon += [obj]
    end = get_time()
    print("Reconstructed %d objects in the scene, time elapsed: %f seconds" % (len(objects_recon), end - start))

    # Visualize results
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis_ctr = vis.get_view_control()

    # Add LiDAR point cloud
    velo_pts, colors = kitti_seq.current_frame.get_colored_pts()
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(velo_pts)
    scene_pcd.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(scene_pcd)

    mesh_extractor = MeshExtractor(decoder, voxels_dim=64)
    for i, obj in enumerate(objects_recon):
        mesh = mesh_extractor.extract_mesh_from_code(obj.code)
        mesh_o3d = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(mesh.vertices), o3d.utility.Vector3iVector(mesh.faces))
        mesh_o3d.compute_vertex_normals()
        mesh_o3d.paint_uniform_color(color_table[i])
        # Transform mesh from object to world coordinate
        mesh_o3d.transform(obj.t_cam_obj)
        vis.add_geometry(mesh_o3d)

    # must be put after adding geometries
    set_view(vis, dist=20, theta=0.)
    vis.run()
    vis.destroy_window()
