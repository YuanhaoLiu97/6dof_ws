import sys

import rclpy
import logging
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Transform
from my_interfaces.msg import RegistrationHomos
from mmdetection_msgs.msg import Detections


import message_filters

from pc_registration.pcRegistration_utils import pcRegistration

class node_pcRegistration(Node):

    def __init__(self, logger=None):
        super().__init__('node_pcRegistration')
        # define logger
        self.logger = logger if logger else self.get_logger(__name__)
        self.logger.info("Logger initialized")

        # define a publisher to publishing result of pcRegistration, as Topic '/registration/homo' of Transform
        self.publisher_ = self.create_publisher(RegistrationHomos, '/registrationhomos', 10)
        self.logger.info("Topic created {}".format('/registrationhomos'))
        
        # define massage filter 
        # self.obj_pc_sub = message_filters.Subscriber(self, PointCloud2, '/obj/point_cloud')
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, "/rvc/point_cloud")
        self.detections_sub = message_filters.Subscriber(self, Detections, "/detections")
        self.logger.info("Topic subscribed {} & {}".format("/rvc/point_cloud", "/rvc/color_image"))

        # define sync policy
        # ts = message_filters.TimeSynchronizer([self.obj_sub, self.pc_sub, self.detections_sub], 10)
        ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.detections_sub], 10, slop=1)
        ts.registerCallback(self.registration_callback)
        self.logger.info("Message filter for registration created.")

    def registration_callback(self, pc_sub, detections_sub):
        
        # self.get_logger().info(' request\na: %d b: %d' % (request.a, request.b))
        # todo:
        self.logger.info('registration callback function called.')
        registration_homos = pcRegistration(pc_sub, detections_sub, self.logger)

        # publish homo as topic '/registration/homo' of Transform
        self.publisher_.publish(registration_homos)
        self.logger.info('{} published'.format('/registrationhomos'))
        return 

def main(args=None):
    
    # define logger
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler = logging.StreamHandler(sys.stdout)
    logger.addHandler(handler)
    # file_handler = logging.FileHandler('{}.log'.format(__name__))
    # file_handler.setFormatter(formatter)
    # file_handler.setLevel(logging.DEBUG)
    # logger.addHandler(file_handler)
    logger.info('node pcregistration is starting')




    rclpy.init(args=args)

    node = node_pcRegistration(logger)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()