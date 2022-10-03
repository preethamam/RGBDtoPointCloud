import open3d as o3d
import numpy as np
import json
import os

from PIL import Image
from numpy import asarray

# ----------------------------------------------------------------------------------------------------------
# Inputs
# ----------------------------------------------------------------------------------------------------------
# Read color image
color = Image.open("redwood_847.png")

# Read depth image
depth = Image.open("redwood_847d.png")

# Depth scale (constant) to convert mm to m vice-versa
depth_scale = 1000

# JSON file of camera intrinsics
json_file = 'camera.json'

# Output file name
pcd_filename = 'output_py.pcd'

# Convert to nnumpy array
color = asarray(color)
depth = asarray(depth)

# ----------------------------------------------------------------------------------------------------------
# RGBD to pointcloud class
# ----------------------------------------------------------------------------------------------------------
class RGBD2Pointcloud:
    # Constructor
    def __init__(self, json_filename, color_image, depth_image, depth_scale = 1000):  
        self.json_filename          = json_filename
        self.depth_scale            = depth_scale
        self.color_image            = np.reshape(np.array(color_image), (-1,3), order='C')
        self.depth_image            = np.reshape(np.array(depth_image), -1, order='C')       
    
    # Camera intrinsics
    def __get_camera_intrincs(self):
        camera_intrinsic = json.load(open(json_file))
        cx = camera_intrinsic['cx']
        cy = camera_intrinsic['cy']
        fx = camera_intrinsic['fx']
        fy = camera_intrinsic['fy']
        width  = camera_intrinsic['width']
        height = camera_intrinsic['height']

        return cx, cy, fx, fy, width, height
    
    # XYZ and RGB data
    def xyz_rgb(self):
        cx, cy, fx, fy, width, height = self.__get_camera_intrincs()

        xx = np.arange(0, width)
        yy = np.arange(0, height)
        xv, yv = np.meshgrid(xx, yy)
              
        u = np.reshape(xv, -1, order='C')
        v = np.reshape(yv, -1, order='C')

        z = self.depth_image / self.depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        xyzRGB = np.concatenate((np.vstack((x, y, z)).T, 
                                 self.color_image), axis=1)
        return xyzRGB

    # Write the point cloud data
    def write2file(self, xyzRGB, file_name):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyzRGB[:,0:3])
        pcd.colors = o3d.utility.Vector3dVector(xyzRGB[:,3:6] / 255.0)
        o3d.io.write_point_cloud(file_name, pcd)


# ----------------------------------------------------------------------------------------------------------        
# Convert RGBD to point cloud and write
# ----------------------------------------------------------------------------------------------------------
# Object files
pcd = RGBD2Pointcloud(json_file, color, depth, depth_scale)

# Get the XYZ annd RGB files
xyzRGB = pcd.xyz_rgb()

# Write to point cloud file
pcd.write2file(xyzRGB, pcd_filename)

