import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
group =np.array([[0., -1., 0.0, 0.],[1., 0., 0., 0.],[0., 0., 1., 0.],[0., 0., 0., 1.]])

color = o3d.io.read_image("RGB1.png")
depth = o3d.io.read_image("Depth1.png")
rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth)
pcd = o3d.io.read_point_cloud("PointCloud/9.ply")
# 将图像上下翻转
# 3. __init__(self: open3d.camera.PinholeCameraIntrinsic, width: int, height: int, fx: float, fy: float, cx: float, cy: float) -> None
#pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


o3d.visualization.draw_geometries([pcd])
