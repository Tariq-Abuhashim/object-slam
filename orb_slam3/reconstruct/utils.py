import torch
import numpy as np
import json
from addict import Dict
import plyfile
import skimage.measure as measure
from deep_sdf.workspace import config_decoder

# Color table used for visualization
COLOR_TABLE = [
    [230. / 255., 0., 0.],
    [60. / 255., 180. / 255., 75. / 255.],
    [0., 0., 255. / 255.],
    [1., 0., 1.],
    [1., 165. / 255., 0.],
    [128. / 255., 0., 128. / 255.],
    [0., 1., 1.],
    [210. / 255., 245. / 255., 60. / 255.],
    [250. / 255., 190. / 255., 190. / 255.],
    [0., 128. / 255., 128. / 255.]
]

def check_cuda():
    try:
        import torch
        if not torch.cuda.is_available():
            print("Warning: CUDA not available.")
    except Exception as e:
        print("Torch init error:", e)

def set_view(vis, dist=100., theta=np.pi / 6.):
    """Set the camera view in the Open3D visualizer."""
    vis_ctr = vis.get_view_control()
    cam = vis_ctr.convert_to_pinhole_camera_parameters()
    T = np.array([
        [1., 0., 0., 0.],
        [0., np.cos(theta), -np.sin(theta), 0.],
        [0., np.sin(theta), np.cos(theta), dist],
        [0., 0., 0., 1.]
    ])
    cam.extrinsic = T
    vis_ctr.convert_from_pinhole_camera_parameters(cam)

def read_calib_file(filepath):
    """Read and parse a KITTI calibration file into a dictionary."""
    data = {}
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip() == "":
                break
            key, value = line.split(':', 1)
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass
    return data

def load_velo_scan(file):
    """Load and parse a Velodyne binary file (KITTI-style)."""
    scan = np.fromfile(file, dtype=np.float32).reshape((-1, 4))
    return scan

class ForceKeyErrorDict(Dict):
    """Custom dictionary that throws KeyError for missing keys."""
    def __missing__(self, key):
        raise KeyError(key)

def get_configs(cfg_file):
    with open(cfg_file) as f:
        cfg_dict = json.load(f)
    return ForceKeyErrorDict(**cfg_dict)

def get_decoder(configs):
    return config_decoder(configs.DeepSDF_DIR)

def create_voxel_grid(vol_dim=128):
    voxel_origin = [-1, -1, -1]
    voxel_size = 2.0 / (vol_dim - 1)

    overall_index = torch.arange(0, vol_dim ** 3, 1, dtype=torch.long)
    values = torch.zeros(vol_dim ** 3, 3)

    values[:, 2] = overall_index % vol_dim
    values[:, 1] = torch.div(overall_index, vol_dim, rounding_mode='trunc') % vol_dim
    values[:, 0] = torch.div(torch.div(overall_index, vol_dim, rounding_mode='trunc'), vol_dim, rounding_mode='trunc') % vol_dim

    values = values * voxel_size + torch.tensor(voxel_origin, dtype=torch.float32)

    return values

def convert_sdf_voxels_to_mesh(sdf_tensor):
    """
    Convert SDF samples (as a 3D tensor) to mesh using marching cubes.
    """
    sdf_np = sdf_tensor.cpu().detach().numpy()
    voxels_dim = sdf_np.shape[0]
    voxel_size = 2.0 / (voxels_dim - 1)

    verts, faces, normals, values = measure.marching_cubes(
        sdf_np, level=0.0, spacing=[voxel_size] * 3
    )

    origin = np.array([-1., -1., -1.])
    verts += origin

    return verts, faces

def write_mesh_to_ply(vertices, faces, ply_filename_out):
    """Write mesh data (vertices and faces) to a .ply file."""
    verts_tuple = np.array(
        [tuple(v) for v in vertices],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
    )

    faces_tuple = np.array(
        [([tuple(f)],) for f in faces],
        dtype=[("vertex_indices", "i4", (3,))]
    )

    el_verts = plyfile.PlyElement.describe(verts_tuple, "vertex")
    el_faces = plyfile.PlyElement.describe(faces_tuple, "face")
    plyfile.PlyData([el_verts, el_faces]).write(ply_filename_out)

