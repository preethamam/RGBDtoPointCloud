import json
import numpy as np
import open3d as o3d

# ----------------------------------------------------------------------------------------------------------
# RGBD to pointcloud class
# ----------------------------------------------------------------------------------------------------------
class RGBD2Pointcloud:
    # Constructor
    def __init__(
        self,
        json_filename: str,
        color_image: np.uint8,
        depth_image: np.uint16,
        depth_scale: int = 1000,
    ) -> None:
        """Init xyz

        Args:
            json_filename (str): _description_
            color_image (np.uint8): _description_
            depth_image (np.uint8): _description_
            depth_scale (int, optional): _description_. Defaults to 1000.
        """
        self.json_filename = json_filename
        self.depth_scale = depth_scale
        self.color_image = np.reshape(np.array(color_image), (-1, 3), order="C")
        self.depth_image = np.reshape(np.array(depth_image), -1, order="C")

    # Camera intrinsics
    def __get_camera_intrincs(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        camera_intrinsic = json.load(open(self.json_filename))
        cx = camera_intrinsic["cx"]
        cy = camera_intrinsic["cy"]
        fx = camera_intrinsic["fx"]
        fy = camera_intrinsic["fy"]
        width = camera_intrinsic["width"]
        height = camera_intrinsic["height"]

        return cx, cy, fx, fy, width, height

    # XYZ and RGB data
    def xyz_rgb(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        cx, cy, fx, fy, width, height = self.__get_camera_intrincs()

        xx = np.arange(0, width)
        yy = np.arange(0, height)
        xv, yv = np.meshgrid(xx, yy)

        u = np.reshape(xv, -1, order="C")
        v = np.reshape(yv, -1, order="C")

        z = self.depth_image / self.depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        xyzRGB = np.concatenate((np.vstack((x, y, z)).T, self.color_image), axis=1)
        return xyzRGB

    # Write the point cloud data
    @staticmethod
    def write2file(xyzRGB: np.float32, file_name: str):
        """_summary_

        Args:
            xyzRGB (_type_): _description_
            file_name (_type_): _description_
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyzRGB[:, 0:3])
        pcd.colors = o3d.utility.Vector3dVector(xyzRGB[:, 3:6] / 255.0)
        o3d.io.write_point_cloud(file_name, pcd)
