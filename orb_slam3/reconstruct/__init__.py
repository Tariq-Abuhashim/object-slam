
# set CUDA_LAUNCH_BLOCKING environment variable in your script before importing any CUDA-dependent modules 
import os
os.environ['CUDA_LAUNCH_BLOCKING'] = "1"

def get_detectors(configs):
    if configs.detect_online:
        print("importing reconstruct.detector2d")
        from reconstruct.detector2d import get_detector2d
        if configs.data_type == "KITTI":
            print("importing reconstruct.detector3d")
            from reconstruct.detector3d import get_detector3d
            return get_detector2d(configs), get_detector3d(configs)
        else:
            return get_detector2d(configs)
    else:
        if configs.data_type == "KITTI":
            return None, None
        else:
            return None
    print('Finished')

def get_sequence(data_dir, configs):
    if configs.data_type == "KITTI":
        from .kitti_sequence_ import KITIISequence  # .kitti_sequence_ is the original source, .kitti_sequence is for Volcan 
        return KITIISequence(data_dir, configs)
    # We use a single class for Redwood and Freiburg sequence
    if configs.data_type == "Redwood" or configs.data_type == "Freiburg":
        from .mono_sequence import MonoSequence
        return MonoSequence(data_dir, configs)
