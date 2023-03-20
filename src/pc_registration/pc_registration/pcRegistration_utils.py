import os, copy
import numpy as np
from math import sqrt
import open3d as o3d

from geometry_msgs.msg import Transform
from my_interfaces.msg import RegistrationHomos
from pc_registration.readpoints import read_points
from pc_registration.o3d_utils import *



def pcRegistration(pc_sub, detections_sub, logger):
    # define homos msgs
    homos_msg = RegistrationHomos() 
    
    # point cloud to np.array
    pc_np_source = np.array(list(read_points(pc_sub)))[:,0:3]
    print(pc_np_source.shape)
    # read detection result
    # mask
    for detection in detections_msg.detections:
        mask = None
        if detection.mask.height > 0 and detection.mask.width > 0:
            mask = np.array(detection.mask.data)
            mask = mask.reshape(detection.mask.height,
                                detection.mask.width)
            mask = mask.astype(bool)
        
            # todo: pcsegmentation
            pc_np_seg_source = pcsegmentation(pc_np_source, mask)
        
        voxel_size = 0.05  # means 5cm for this dataset
        # read from Demo as an example
        # demo_icp_pcds = o3d.data.DemoICPPointClouds()
        # source = o3d.io.read_point_cloud('rs.pcd')
        # target = o3d.io.read_point_cloud('rs2.pcd')
        # read point clouds as source and target



        source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_np_seg_source))
        target = o3d.io.read_point_cloud("output1.pcd") 
        target.estimate_normals()
        # down sampling both clouds and extract fpfh features
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

        # o3d.visualization.draw_geometries([source_down, target_down], window_name='pre-registration')
        # global registration with RANSAC
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    voxel_size)
        # visualization result of GR
        # draw_registration_result(source_down, target_down, result_ransac.transformation)

        print(result_ransac.transformation)
        distance_threshold = voxel_size * 0.4
        result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                        voxel_size, result_ransac.transformation, distance_threshold)

        # visualization result of ICP
        # draw_registration_result(source, target, result_icp.transformation)

        print(result_icp.transformation)
        # define msg in topic Transform 
        tf = Transform()
        # translation with x,y,z in double(float64)
        tf.translation.x = result_icp.transformation[0,3]
        tf.translation.y = result_icp.transformation[1,3]
        tf.translation.z = result_icp.transformation[2,3]
        # rotation in quaternion in 
        quat = rotation_matrix_to_quaternion(result_icp.transformation[0:3,0:3])
        tf.rotation.x = quat[0]
        tf.rotation.y = quat[1]
        tf.rotation.z = quat[2]
        tf.rotation.w = quat[3]

    homos_msg.data.append(tf)
    
    return homos_msg


def rotation_matrix_to_quaternion(R):
    m11, m12, m13, m21, m22, m23, m31, m32, m33 = R.ravel()
    w = sqrt(1 + m11 + m22 + m33) / 2
    x = (m23 - m32) / (4 * w)
    y = (m31 - m13) / (4 * w)
    z = (m12 - m21) / (4 * w)
    q_norm = sqrt(w*w + x*x + y*y + z*z)
    return np.array([w, x, y, z]) / q_norm


def pcsegmentation(pc_np_source, mask):
    h,w = mask.shape
    pc_np_seg_source = pc_np_source.reshape((h,w,-1))[mask]
    return pc_np_seg_source