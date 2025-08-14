
import warnings
import cv2
import torch
import numpy as np
import mmcv
from mmcv.runner import load_checkpoint
from mmdet.models import build_detector
from mmdet.core import get_classes
from mmdet.apis import inference_detector

# Configure logging for the module
import logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d (%(funcName)s) - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

#object_class_table = {"cars": [2], "chairs": [56, 57]}
object_class_table = {"Person": [0], "cars": [2], "Motorcycle": [3], "Bus": [5], "Truck": [7], "chairs": [56, 57]}

def get_detector2d(configs):
    logger.debug('Setting detector2D')
    return Detector2D(configs)

class Detector2D(object):
    def __init__(self, configs):
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        config = configs.Detector2D.config_path
        checkpoint = configs.Detector2D.weight_path
        if isinstance(config, str):
            config = mmcv.Config.fromfile(config)
        elif not isinstance(config, mmcv.Config):
            raise TypeError('config must be a filename or Config object, '
                            f'but got {type(config)}')
        config.model.pretrained = None
        config.model.train_cfg = None
        self.model = build_detector(config.model, test_cfg=config.get('test_cfg'))
        if checkpoint is not None:
            checkpoint = load_checkpoint(self.model, checkpoint, map_location='cpu')
            if 'CLASSES' in checkpoint.get('meta', {}):
                self.model.CLASSES = checkpoint['meta']['CLASSES']
            else:
                warnings.simplefilter('once')
                warnings.warn('Class names are not saved in the checkpoint\'s '
                              'meta data, use COCO classes by default.')
                self.model.CLASSES = get_classes('coco')
        self.model.cfg = config  # save the config in the model for convenience
        self.model.to(device)
        self.model.eval()
        self.min_bb_area = configs.min_bb_area
        self.predictions = None

    def make_prediction(self, image, object_classes=["cars", "Person", "Bus", "Truck", "Motorcycle"]):
        #assert object_class == "chairs" or object_class == "cars"
        self.predictions = inference_detector(self.model, image)

        labels = []
        boxes = []
        masks = []
        for obj_class in object_classes:
            #assert obj_class in ["chairs", "cars"], "Object class should be 'chairs' or 'cars'"
            #class_boxes = [self.predictions[0][o] for o in object_class_table[obj_class]]
            class_boxes_and_labels = [(self.predictions[0][o], o) for o in object_class_table[obj_class]]
            class_boxes, class_labels = zip(*class_boxes_and_labels) # the results are tuples
            # Convert tuples to lists
            class_boxes = list(class_boxes) # list of arrays
            class_labels = list(class_labels) # list of integers
            # Skip if no boxes were found for this class
            if len(class_boxes) > 0 and len(class_boxes[0]) > 0:
                #for bbox, label in zip(class_boxes, class_labels):
                #    logger.debug(f"Box: {bbox}, Label: {label}")
                # Transform list of arrays to a single array (NumPy array)
                class_boxes = np.concatenate(class_boxes, axis=0)
                class_labels = class_labels * len(class_boxes)
                boxes.append(class_boxes)
                labels.append(class_labels)
            else:
                class_boxes = np.zeros((0, 0, 5))
                class_labels = []

            class_masks = []
            n_det = 0
            for o in object_class_table[obj_class]:
                class_masks += self.predictions[1][o]
                n_det += len(self.predictions[1][o])
            # In case there is no detections
            if n_det == 0:
                class_masks = np.zeros((0, 0, 0))
            else:
                class_masks = np.stack(class_masks, axis=0)
                masks.append(class_masks)
        
            #logger.debug("Labels: ", class_labels)
            #logger.debug("Boxes shape: ", class_boxes.shape)
            #logger.debug("Masks shape: ", class_masks.shape)
            assert class_boxes.shape[0] == class_masks.shape[0], "Number of boxes and masks do not match"
            assert class_boxes.shape[0] == len(class_labels), "Number of boxes and labels do not match"

        self.height, self.width = image.shape[0], image.shape[1]

        # Flatten lists of arrays into a single array for each of boxes and masks
        if boxes:
            boxes = np.concatenate(boxes, axis=0)
        if masks:
            masks = np.concatenate(masks, axis=0)
        if labels:
            labels = np.concatenate(labels, axis=0)
        return self.get_valid_detections(labels, boxes, masks)

    def visualize_result(self, image, filename):
        self.model.show_result(image, self.predictions, out_file=filename, score_thr=0.7)

    def get_valid_detections(self, labels, boxes, masks):
        if len(boxes) == 0 or len(masks) == 0:
            return {"pred_labels": np.array([]), "pred_boxes": np.array([]), "pred_masks": np.array([]), "pred_scores": np.array([])}
        boxes = np.array(boxes) # Python doesn't allow to use a tuple to index a list.
        masks = np.array(masks) # valid_mask is tuple, while boxes and masks are lists
        labels = np.array(labels)
        # hence, first need to convert boxes and masks to numpy arrays
        # Remove those on the margin
        #print(boxes)
        #   0----2
        # 1
        # |
        # 3
        cond1 = (boxes[:, 0] >= 30) & (boxes[:, 1] > 30) & (boxes[:, 2] < self.width-30) & (boxes[:, 3] < self.height-30) # FIXME image boarders
        boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        # Remove those with too small bounding boxes
        cond2 = (boxes_area > self.min_bb_area)
        scores = boxes[:, -1]
        cond3 = (scores >= 0.80)

        valid_mask = (cond2 & cond3)
        valid_instances = {"pred_labels": labels[valid_mask],
                           "pred_boxes": boxes[valid_mask, :4],
                           "pred_masks": masks[valid_mask, ...],
                           "pred_scores": boxes[valid_mask, -1]}

        return valid_instances

    @staticmethod
    def save_masks(masks):
        mask_imgs = masks.cpu().numpy()
        n = mask_imgs.shape[0]
        for i in range(n):
            cv2.imwrite("mask_%d.png" % i, mask_imgs[i, ...].astype(np.float32) * 255.)
