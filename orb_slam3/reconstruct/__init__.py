import os
import logging

# Set CUDA and PyTorch environment variables before any CUDA usage
os.environ['CUDA_LAUNCH_BLOCKING'] = "1"
os.environ['PYTORCH_NO_CUDA_MEMORY_CACHING'] = "1"

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d (%(funcName)s) - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# Internal imports (organized alphabetically)
#from . import detector2d
#from . import detector3d
#from . import kitti_sequence
#from . import loss
#from . import loss_utils
#from . import mono_sequence
#from . import optimizer
#from . import script
#from . import utils


def get_detectors(configs):
    """Initializes and returns the appropriate 2D/3D detectors based on config."""
    if configs.detect_online:
        logger.info("Importing reconstruct.detector2d")
        from reconstruct.detector2d import get_detector2d
        logger.info("Successfully imported detector2d")
        if configs.data_type == "KITTI":
            logger.info("Importing reconstruct.detector3d")
            from reconstruct.detector3d import get_detector3d
            logger.info("Successfully imported detector3d")
            return get_detector2d(configs), get_detector3d(configs)
        else:
            return get_detector2d(configs)
    else:
        if configs.data_type == "KITTI":
            return None, None
        else:
            return None
    logger.info('Finished initialisation ')


def get_sequence(data_dir, configs):
    """Returns the appropriate sequence loader based on dataset type."""
    if configs.data_type == "KITTI":
        from .kitti_sequence import KITIISequence
        logger.info("Loaded KITTI sequence handler")
        return KITIISequence(data_dir, configs)

    if configs.data_type in ["Redwood", "Freiburg"]:
        from .mono_sequence import MonoSequence
        logger.info(f"Loaded MonoSequence handler for {configs.data_type}")
        return MonoSequence(data_dir, configs)

    logger.warning(f"Unknown data_type: {configs.data_type}")
    return None


