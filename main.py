from PIL import Image
from numpy import asarray
from constants import *

from rgbd2pointcloud import RGBD2Pointcloud


if __name__ == "__main__":

    # Inputs
    # Read color image
    color = Image.open("redwood_847.png")

    # Read depth image
    depth = Image.open("redwood_847d.png")

    # Convert to nnumpy array
    color = asarray(color)
    depth = asarray(depth)

    # Convert RGBD to point cloud and write
    # Object files
    pcd = RGBD2Pointcloud(JSON_FILE, color, depth, DEPTH_SCALE)

    # Get the XYZ annd RGB files
    xyzRGB = pcd.xyz_rgb()

    # Write to point cloud file
    pcd.write2file(xyzRGB, PCD_FILENAME)
