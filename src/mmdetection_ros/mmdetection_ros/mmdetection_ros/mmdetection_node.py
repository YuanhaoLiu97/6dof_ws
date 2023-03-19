
import os, logging, sys
from typing import List, Union

from attr import s

import rclpy
import rclpy.qos as qos
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

import torch
import mmcv
import numpy as np

from mmdet.apis import inference_detector, init_detector
from mmdet.core import INSTANCE_OFFSET

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from mmdetection_msgs.srv import TaskRequest
from mmdetection_msgs.msg import (
    Detection,
    Detections
)


class MmDetectionNode(Node):

    def __init__(self, logger=None) -> None:
        super().__init__("mmdetection_node")

        # define logger
        self.logger = logger if logger else self.get_logger()
        self.logger.info("Logger initialized")

        # params
        self.declare_parameter(
            "config", os.path.join
            (get_package_share_directory("mmdetection_bringup"),
             "config",
             "solo/solov2_r50_fpn_3x_coco.py"))
        self.declare_parameter(
            "weights", os.path.join
            (get_package_share_directory("mmdetection_bringup"),
             "config",
             "solo/solov2_r50_fpn_3x_coco.pth"))
        self.declare_parameter("device", "cpu")
        self.declare_parameter("threshold", 0.3)
        self.declare_parameter("enable", True)

        self.declare_parameter("label", "cylinder")
        self.declare_parameter("score", 0.75)
        self.declare_parameter("count", 1)

        self.request_dict ={}
        

        config = self.get_parameter(
            "config").get_parameter_value().string_value
        weights = self.get_parameter(
            "weights").get_parameter_value().string_value
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.label = self.get_parameter(
            'label').get_parameter_value().string_value
        self.score = self.get_parameter(
            'score').get_parameter_value().double_value
        self.count = self.get_parameter(
            'count').get_parameter_value().integer_value


        self.model = init_detector(config, weights, device)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.cv_bridge = CvBridge()

        # topcis
        self._pub = self.create_publisher(
            Detections, "detections", 10)
        self._sub = self.create_subscription(
            Image, "/rvc/color_image", self.image_cb,
            10
        )

        # services
        self._srv = self.create_service(TaskRequest, "TaskRequest", self.enable_cb)

    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.label =  req.label
        self.count =  req.count
        self.score =  req.score
        self.enable = req.enable
        res.success = True
        return res

    def parse_results(self, result: Union[list, dict, tuple]) -> List[np.ndarray]:

        # panoptic
        if "pan_results" in result:
            pan_results = result["pan_results"]
            ids = np.unique(pan_results)[::-1]
            legal_indices = ids != self.model.num_classes
            ids = ids[legal_indices]
            labels = np.array(
                [id % INSTANCE_OFFSET for id in ids], dtype=np.int64)
            masks = (pan_results[None] == ids[:, None, None])
            return [labels, None, masks]

        # object detection
        if isinstance(result, tuple):
            bbox_result, mask_result = result
            if isinstance(mask_result, tuple):
                mask_result = mask_result[0]  # ms rcnn
        else:
            bbox_result, mask_result = result, None
        bboxes = np.vstack(bbox_result)

        labels = [
            np.full(bbox.shape[0], i, dtype=np.int32)
            for i, bbox in enumerate(bbox_result)
        ]
        labels = np.concatenate(labels)

        # instance segmentation
        masks = None
        if mask_result is not None and len(labels) > 0:  # non empty
            masks = mmcv.concat_list(mask_result)
            if isinstance(masks[0], torch.Tensor):
                masks = torch.stack(masks, dim=0).detach().cpu().numpy()
            else:
                masks = np.stack(masks, axis=0)

        return [labels, bboxes, masks]

    def image_cb(self, msg: Image) -> None:
        self.logger.info("image segmentation callback is called.")
        if self.enable:

            detections_msg = Detections()
            detections_msg.header.frame_id = msg.header.frame_id
            detections_msg.header.stamp = msg.header.stamp  # self.get_clock().now().to_msg()

            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            result = inference_detector(self.model, cv_image)

            labels, bboxes, masks = self.parse_results(result)
            
            if not labels is None:

                detections = [labels]
                bboxes_idx = 1
                masks_idx = 2

                if not bboxes is None:
                    detections.append(bboxes)
                else: 
                    masks_idx = 1

                if not masks is None:
                    detections.append(masks)

                for detection in zip(*detections):

                    if bboxes is None or detection[bboxes_idx][4] > self.threshold:
                        d_msg = Detection()
                        d_msg.label_id = int(detection[0])

                        if detection[0] < len(self.model.CLASSES):
                            d_msg.label = self.model.CLASSES[detection[0]]

                        if not bboxes is None:
                            d_msg.score = float(detection[bboxes_idx][4])

                        if not bboxes is None:
                            d_msg.box.center_x = float((
                                detection[bboxes_idx][0] + detection[bboxes_idx][2]) / 2)
                            d_msg.box.center_y = float((
                                detection[bboxes_idx][1] + detection[bboxes_idx][3]) / 2)
                            d_msg.box.size_x = float(
                                detection[bboxes_idx][2] - detection[bboxes_idx][0])
                            d_msg.box.size_y = float(
                                detection[bboxes_idx][3] - detection[bboxes_idx][1])

                        if not masks is None:
                            d_msg.mask.height = len(detection[masks_idx])
                            d_msg.mask.width = len(detection[masks_idx][0])
                            d_msg.mask.data = np.concatenate(
                                detection[masks_idx]).tolist()

                        detections_msg.detections.append(d_msg)

            # filter with label, score, and count
            self.logger.info("len of detections before filter {}".format(len(detections_msg.detections)))
            detections_msg_temp = Detections()
            detections_msg_temp.header = detections_msg.header
            n = 0
            for d in detections_msg.detections:
                self.logger.info("NO.{} label {}, score {}".format(n, d.label, d.score))
                if d.label == self.label and d.score >= self.score and n < self.count:
                    detections_msg_temp.detections.append(d)
                    n += 1
            self.logger.info("len of detections after filter {}".format(len(detections_msg_temp.detections)))
            self._pub.publish(detections_msg_temp)


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler = logging.StreamHandler(sys.stdout)
    logger.addHandler(handler)
    logger.info('node mmdetection is starting')
    
    rclpy.init()
    node = MmDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
