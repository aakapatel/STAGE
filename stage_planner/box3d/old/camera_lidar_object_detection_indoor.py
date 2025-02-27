#! /usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO
from sklearn.cluster import KMeans
from typing import List
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
import time

#ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import message_filters
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

MAX_RANGE = 10
UNDISTORT = False
FRAME = "shafter4/camera_aligned_depth_to_color_frame"
# OOI = [0,1,2,3,5,6]
OOI = [13,24,25,26,39,40,41,42,43,44,45,56,57,58,60,61,68,69,71,73,74]

NAMES = {13:0,24:1,25:2,26:3,39:4,40:5,41:6,42:7,43:8,44:9,45:10,56:11,57:12,58:13,60:14,61:15,68:16,69:17,71:18,73:19,74:20}
# NAMES = {
#   0: 0,
#   1: 1,
#   2: 2,
#   3: 3,
#   5: 4,
#   6: 5}

class yolo_lidar:
	def __init__(self):

		self.model = YOLO("yolov8n-seg.pt")

		self.yolo_pub = rospy.Publisher("/yolo/color/image_raw", Image, queue_size = 1)
		self.yolo_seg_pub = rospy.Publisher("/yolo/segmentation_mask/image_raw", Image, queue_size = 1)

		self.yolo_pcl_pub = rospy.Publisher("/yolo/points", PointCloud2, queue_size = 1)

		# self.yolo_pub = rospy.Publisher("/thermal/yolo/image_raw", Image, queue_size = 1)
		# self.marker_pub = rospy.Publisher("/thermal/yolo/markers", MarkerArray, queue_size = 1)
		self.depth_ol_pub = rospy.Publisher("/lidar/aligned_range_to_image/image_overlay", Image, queue_size = 1)
		self.bbox_pub = rospy.Publisher("/lidar/aligned_bounding_boxes", MarkerArray, queue_size = 1)
		# self.global_seg_pub = rospy.Publisher("/lidar/segmented_map", PointCloud2, queue_size = 1)

		self.bridge = CvBridge()

		######################## CAMERA INFO TOPIC ############################
		#self.intrinsics_sub = rospy.Subscriber("/spot/camera/color/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)
		self.intrinsics_sub = rospy.Subscriber("/shafter4/camera/depth/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)
		
		###################### REAL SENSE CAMERA #########################	
		self.image_sub = message_filters.Subscriber("/shafter4/camera/color/image_raw", Image)
		self.points_sub = message_filters.Subscriber("/shafter4/camera/depth/color/points",PointCloud2)
		#self.odometry_sub = message_filters.Subscriber("/dlo/odom_node/odom", Odometry)
		self.height = 720
		self.width = 1280
		self.camera_matrix = np.asarray([635.2752075195312/2.0, 0.0, 652.32861328125/2.0, 0.0, 634.6666259765625/2.0, 358.4922180175781/2.0, 0.0, 0.0, 1.0]).reshape((3, 3))
		self.dist_coeffs = np.asarray([-0.05529356002807617, 0.06882049143314362, -0.00044326853821985424, 0.0005935364752076566, -0.021897781640291214])
		self.tvec = np.asarray([0.00, 0.00, 0.00])
		self.rvec,_ = cv2.Rodrigues(np.asarray([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]))

		######################## SPOT SIMULATION ##########################
		# self.image_sub = message_filters.Subscriber("/d435/color/image_raw", Image)
		# self.points_sub = message_filters.Subscriber("/velodyne_points",PointCloud2)
		# self.height = 720
		# self.width = 1280
		# self.camera_matrix = np.asarray([695.9951171875, 0.0, 640.0, 0.0, 695.9951171875, 360.0, 0.0, 0.0, 1.0]).reshape((3, 3))
		# self.dist_coeffs = np.zeros((5, 1), np.float32)
		# self.tvec = np.asarray([0.00,0.00,0.00])
		# self.rvec,_ = cv2.Rodrigues(np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]]))

		######################## FLIR BOSON THERMAL CAMERA ##########################
		# self.image_sub = message_filters.Subscriber("/flir_boson/image_raw", Image)
		# self.points_sub = message_filters.Subscriber("/shafter2/ouster/points",PointCloud2)
		# self.height = 512
		# self.width = 640
		# self.camera_matrix = np.asarray([767.942728, 0.000000, 317.244746, 0.000000, 767.068789, 231.644928, 0.000000, 0.000000, 1.000000]).reshape((3, 3))
		# self.dist_coeffs = np.asarray([-0.504042, 0.268940, 0.003454, -0.000748, 0.000000])
		# self.tvec = np.asarray([0.00,0.50,0.85])
		# self.rvec,_ = cv2.Rodrigues(np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]]))

		######################## KITTY DATASET ########################
		# self.image_sub = message_filters.Subscriber("/kitti/camera_color_left/image", Image)
		# self.points_sub = message_filters.Subscriber("/kitti/velo/pointcloud", PointCloud2)
		# self.odometry_sub = message_filters.Subscriber("/kitti/dlo/odom_node/odom", Odometry)
		# self.glboal_map_sub = message_filters.Subscriber("/kitti/dlo/map_node/map", PointCloud2)

		# self.height = 376
		# self.width = 1241
		# self.camera_matrix = np.asarray([718.856, 0.0, 607.1928, 0.0, 718.856, 185.2157, 0.0, 0.0, 1.0]).reshape((3, 3))
		# # self.camera_matrix = np.asarray([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).reshape((3, 3))
		# # self.camera_matrix = np.asarray([718.856, 0.0, 607.1928, 45.38225, 0.0, 718.856, 185.2157, -0.1130887, 0.0, 0.0, 1.0, 0.003779761]).reshape((3, 4))
		# self.dist_coeffs = np.zeros((5, 1), np.float32)
		# # self.dist_coeffs = np.asarray([718.856, 0.0, 607.1928, 45.38225, 0.0, 718.856, 185.2157, -0.1130887, 0.0, 0.0, 1.0, 0.003779761])
		# #self.tvec = np.zeros((3, 1), np.float32)
		# self.tvec = np.asarray([-4.069766e-03, -7.631618e-02, -2.717806e-01])
		# self.rvec,_ = cv2.Rodrigues(np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]]))
		# # self.rvec = np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]])

		#############
		# self.camera_matrix = np.zeros((3, 3), np.float32)
		# self.dist_coeffs = np.zeros((5, 1), np.float32)
		# self.rvec = np.zeros((3, 1), np.float32)
		# self.tvec = np.zeros((3, 1), np.float32)
		############

		#self.cmap = plt.cm.viridis
		self.cmap = plt.cm.jet
		self.cmaplist = [self.cmap(i) for i in range(self.cmap.N)]

		self.ts = message_filters.ApproximateTimeSynchronizer([self.points_sub, self.image_sub], queue_size=1, slop=0.5)
		#self.ts = message_filters.TimeSynchronizer([self.points_sub, self.image_sub], queue_size=1)
		self.ts.registerCallback(self.callback)
		#print("Initialized")

	def callback_intrinsics(self,data):
		self.height = data.height
		self.width = data.width
		self.camera_matrix = np.asarray(data.K).reshape((3, 3))
		self.dist_coeffs = np.asarray(data.D)
		print(self.camera_matrix, self.dist_coeffs)
		self.intrinsics_sub.unregister()

	def callback(self, pcl_data, img_data):
		#print("Callback")
		frame = self.bridge.imgmsg_to_cv2(img_data, desired_encoding=img_data.encoding)

		points_xyz = []
		for point in pcl2.read_points(pcl_data, field_names=('x', 'y', 'z'), skip_nans=True):
			points_xyz.append(list(point))

		print(len(points_xyz))

		pixels, _ = cv2.projectPoints(np.asarray(points_xyz), self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)

		print(*pixels)

		if UNDISTORT:
			newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (self.width, self.height), 1, (self.width, self.height))
			frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, newcameramatrix)

		results = self.model.predict(frame, conf=0.50, retina_masks=True, classes = OOI) 
		self.yolo_pub.publish(self.bridge.cv2_to_imgmsg(results[0].plot(), 'rgb8'))
		# boxes = results[0].boxes.cpu().numpy().data

		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		depth_ol = frame.copy()

		#depth_img = np.zeros((self.height,self.width, 1), dtype = np.float32)
		segMask = np.zeros((self.height,self.width, 1), dtype = np.uint8)
		instanceMask = np.zeros((self.height,self.width, 1), dtype = np.uint8)

		if results[0].masks is not None:
			masks = results[0].masks.cpu().numpy().data
			for idxMask, mask in enumerate(masks):
				#print(*mask.shape)

				#Reduce overlap
				#mask = mask.transpose(1, 2, 0).squeeze()
				mask = cv2.resize(mask, (self.width, self.height))
				mask = np.uint8(mask)
				kernel = np.ones((35, 35), np.uint8)
				mask = cv2.erode(mask, kernel) 

				#print(results[0].boxes[idxMask].cpu().numpy().data[0][5])

				#segMask = cv2.add(segMask, mask*int(results[0].boxes[idxMask].cpu().numpy().data[0][5] + 1)) 
				segMask = cv2.add(segMask, mask*(NAMES[results[0].boxes[idxMask].cpu().numpy().data[0][5]] + 1)) 
				instanceMask = cv2.add(instanceMask, mask*(idxMask + 1)) 
			
		self.yolo_seg_pub.publish(self.bridge.cv2_to_imgmsg(segMask, 'mono8'))
					
		points_yolo = []

		# print(len(pixels))

		for idxPixel, pixel in enumerate(pixels):
			if pixel[0][0]<self.width and pixel[0][1]<self.height and pixel[0][0]>=0 and pixel[0][1]>=0:# and segMask[int(pixel[0][1]),int(pixel[0][0])] > 0:
			#if segMask[int(pixel[0][1]),int(pixel[0][0])] > 0:

				depth = int(np.linalg.norm(points_xyz[idxPixel])*255/MAX_RANGE)
				if depth > 255: depth = 255

				color_range = self.cmaplist[depth]
				depth_ol = cv2.circle(depth_ol, tuple([int(pixel[0][0]), int(pixel[0][1])]), 1, (color_range[0]*255, color_range[1]*255, color_range[2]*255), -1)

				#depth_img[int(pixel[0][1]),int(pixel[0][0])] = np.linalg.norm(points_xyz[idxPixel])

				points_yolo.append([points_xyz[idxPixel][0], points_xyz[idxPixel][1], points_xyz[idxPixel][2], instanceMask[int(pixel[0][1]),int(pixel[0][0])], segMask[int(pixel[0][1]),int(pixel[0][0])]])

			#else: points_yolo.append([points_xyz[idxPixel][0], points_xyz[idxPixel][1], points_xyz[idxPixel][2], 0, 0])

			header = Header()
			header.frame_id = FRAME
			header.stamp = rospy.Time.now()

			fields = [PointField('x', 0, PointField.FLOAT32, 1),
				PointField('y', 4, PointField.FLOAT32, 1),
				PointField('z', 8, PointField.FLOAT32, 1),
				PointField('instance', 12, PointField.FLOAT32, 1),
				PointField('label', 16, PointField.FLOAT32, 1)]
			
			# if len(points_yolo) > 0: 
			point_cloud = pcl2.create_cloud(header, fields, points_yolo)
			self.yolo_pcl_pub.publish(point_cloud)

			self.depth_ol_pub.publish(self.bridge.cv2_to_imgmsg(depth_ol, 'bgr8'))


	
if __name__ == '__main__':
	rospy.init_node('lidar_camera_aligment', anonymous=True)
	yt = yolo_lidar()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down") 
