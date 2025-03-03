#! /usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import message_filters
import matplotlib.pyplot as plt
from ultralytics import YOLO
import open3d as o3d
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import time

# ####################### ELLI CORE ############################

import sys
import os
sys.path.append(os.path.abspath(".."))  # Adds the parent directory to the path

from box3d_core import *

# #####################################################

FRAME = "world"
KERNEL = 5
UNDISTORT = False
SEGMENTED = True

# REMAP = {1:0,2:1,25:2,26:3,39:4,40:5,41:6,42:7,43:8,44:9,45:10,56:11,57:12,58:13,60:14,61:15,68:16,69:17,71:18,73:19,74:20} 
# OOI = [0,1]

# #Indoors COCO
# REMAP = {13:0,24:1,25:2,26:3,39:4,40:5,41:6,42:7,43:8,44:9,45:10,56:11,57:12,58:13,60:14,61:15,68:16,69:17,71:18,73:19,74:20} 
# OOI = [13,24,25,26,39,40,41,42,43,44,45,56,57,58,60,61,68,69,71,73,74]

#Outdoors COCO
# REMAP = {0:0,1:1,2:2,3:3,5:4,6:5} 
# OOI = [0,1,2,3,5,6]

# SubT artifacts
# REMAP = {0:0,1:1,2:2,3:3,4:4,5:5} 
# OOI = [0,1,2,3,4,5]

class yolo_lidar:
	def __init__(self):

		if SEGMENTED: self.model = YOLO("yolo11m-seg") #Segmentation
		else: self.model = YOLO("/home/godzillapc/mario/vision_utils/box3d/weights/subt.pt") #No segmentation

		self.yolo_pub = rospy.Publisher("/yolo/color/image_raw", Image, queue_size = 1)
		self.yolo_seg_pub = rospy.Publisher("/yolo/segmentation_mask/image_raw", Image, queue_size = 1)
		self.yolo_inst_pub = rospy.Publisher("/yolo/instance_mask/image_raw", Image, queue_size = 1)
		self.yolo_pcl_pub = rospy.Publisher("/yolo/points", PointCloud2, queue_size = 1)
		self.bbox_pub = rospy.Publisher("/yolo/aligned_bounding_boxes", MarkerArray, queue_size = 1)

		self.bridge = CvBridge()

		######################## CAMERA INFO TOPIC ############################
		self.intrinsics_sub = rospy.Subscriber("/hummingbird/camera/color/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)
		# self.intrinsics_sub = rospy.Subscriber("/spot/camera/color/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)

		
		###################### REAL SENSE CAMERA #########################	
		self.image_sub = message_filters.Subscriber("/hummingbird/camera/color/image_raw", Image)
		self.points_sub = message_filters.Subscriber("/hummingbird/ouster/points",PointCloud2)
		self.odometry_sub = message_filters.Subscriber("/hummingbird/ground_truth/odometry", Odometry)


		# self.image_sub = message_filters.Subscriber("/spot/camera/color/image_raw_throttle", Image)
		# self.points_sub = message_filters.Subscriber("/spot/velodyne_points",PointCloud2)
		# self.odometry_sub = message_filters.Subscriber("/spot/odometry/imu", Odometry)
		self.height = 720
		self.width = 1280
		self.camera_matrix = np.asarray([424.2752075195312, 0.0, 320.32861328125, 0.0, 424.6666259765625, 240.4922180175781, 0.0, 0.0, 1.0]).reshape((3, 3))
		self.dist_coeffs = np.asarray([-0.05529356002807617, 0.06882049143314362, -0.00044326853821985424, 0.0005935364752076566, -0.021897781640291214])
		# self.tvec = np.asarray([0.060,0.018,0.072])
		self.tvec = np.asarray([0.2,0.0,-0.01])
		self.rvec,_ = cv2.Rodrigues(np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]]))

		#self.cmap = plt.cm.viridis
		self.cmap = plt.cm.jet
		self.cmaplist = [self.cmap(i) for i in range(self.cmap.N)]

		self.ts = message_filters.ApproximateTimeSynchronizer([self.points_sub, self.image_sub, self.odometry_sub], queue_size=1, slop=0.05)
		# self.ts = message_filters.TimeSynchronizer([self.points_sub, self.image_sub, self.odometry_sub], queue_size=1)
		self.ts.registerCallback(self.callback)
		#print("Initialized")
		self.global_bboxes = []
		self.colors = []

	def callback_intrinsics(self,data):
		self.height = data.height
		self.width = data.width
		self.camera_matrix = np.asarray(data.K).reshape((3, 3))
		self.dist_coeffs = np.asarray(data.D)
		print(self.camera_matrix, self.dist_coeffs)
		self.intrinsics_sub.unregister()

	def callback(self, pcl_data, img_data, odom_data):
		#print("Callback")
		frame = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")

		points = []
		for point in pcl2.read_points(pcl_data, field_names=('x', 'y', 'z'), skip_nans=True):
			if np.linalg.norm(point) > 0 and point[0]>self.tvec[0]:
				points.append(list(point))

		pixels, _ = cv2.projectPoints(np.asarray(points), self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)

		# Undistorting
		if UNDISTORT:
			newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (self.width, self.height), 1, (self.width, self.height))
			frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, newcameramatrix)

		# frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		results = self.model.predict(frame, retina_masks=True) 
		self.yolo_pub.publish(self.bridge.cv2_to_imgmsg(results[0].plot(), 'bgr8'))

		segMask = np.zeros((self.height,self.width), dtype = np.uint8)
		instMask = np.zeros((self.height,self.width), dtype = np.uint8)

		if results[0].masks is not None:
			masks = results[0].masks.cpu().numpy().data
			for idxMask, mask in enumerate(masks):

				mask = cv2.resize(mask, (self.width, self.height))
				mask = np.uint8(mask)
				kernel = np.ones((KERNEL, KERNEL), np.uint8)
				mask = cv2.erode(mask, kernel) 

				#segMask = cv2.add(segMask, mask*int(results[0].boxes[idxMask].cpu().numpy().data[0][5] + 1)) 
				# segMask = cv2.add(segMask, mask * (REMAP[results[0].boxes[idxMask].cpu().numpy().data[0][5]] + 1)) 
				segMask = cv2.add(segMask, mask * (int(results[0].boxes[idxMask].cpu().numpy().data[0][5]) + 1)) 
				instMask = cv2.add(instMask, mask * (idxMask + 1))

		else: 
			# Divide the bbox into 9 sectors and use the central sector as the mask
			for idxBox, box in enumerate(results[0].boxes):
				box_data = box.cpu().numpy().data[0]
				x1, y1, x2, y2 = int(box_data[0]), int(box_data[1]), int(box_data[2]), int(box_data[3])
				# Calculate the center of the bbox
				center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
				# Calculate the width and height of the bbox
				width, height = x2 - x1, y2 - y1
				# Calculate the size of each sector
				sector_width, sector_height = width // 3, height // 3

				# Calculate the top-left corner of the central sector
				central_sector_x1, central_sector_y1 = center_x - sector_width // 2, center_y - sector_height // 2
				central_sector_x2, central_sector_y2 = center_x + sector_width // 2, center_y + sector_height // 2

				# Create a mask for the central sector
				central_sector_mask = np.zeros((self.height, self.width), dtype=np.uint8)
				central_sector_mask[central_sector_y1:central_sector_y2, central_sector_x1:central_sector_x2] = 1

				# segMask = cv2.add(segMask, central_sector_mask * (REMAP[results[0].boxes[idxBox].cpu().numpy().data[0][5]] + 1))
				segMask = cv2.add(segMask, central_sector_mask * (int(results[0].boxes[idxBox].cpu().numpy().data[0][5]) + 1))
				instMask = cv2.add(instMask, central_sector_mask * (idxBox + 1))
			
		self.yolo_seg_pub.publish(self.bridge.cv2_to_imgmsg(segMask, 'mono8'))
		self.yolo_inst_pub.publish(self.bridge.cv2_to_imgmsg(instMask, 'mono8'))
					
		points_yolo = []

		#if results[0].masks is not None:
		for idxPixel, pixel in enumerate(pixels):
			if pixel[0][0]<self.width and pixel[0][1]<self.height and pixel[0][0]>=0 and pixel[0][1]>=0 and points[idxPixel][0]>self.tvec[0]:
				# Ensure that we are appending 5 items: x, y, z, instance, and label
				instance_id = instMask[int(pixel[0][1]), int(pixel[0][0])]
				label_id = segMask[int(pixel[0][1]), int(pixel[0][0])]
				points_yolo.append([points[idxPixel][0], points[idxPixel][1], points[idxPixel][2], instance_id, label_id])

		print(f"Number of yolo points is: {len(points_yolo)}")
		# print(points_yolo)
		## Cluster the points based within an instance using euclidean distance in order to filter misclassified points
		##  Try removing walls and road
		# try:
		
		## Transform points_yolo based on odometry pose
		points = np.asarray(points_yolo)[:,:3]
		pose = odom_data.pose.pose
		## Get rotation matrix from quaternion
		rotation = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
		## Make the transformation matrix
		transformation_matrix = np.eye(4)
		transformation_matrix[:3,:3] = rotation
		transformation_matrix[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
		## Make pointcloud homogeneous
		points = np.hstack((points, np.ones((len(points),1))))
		## Apply the transformation
		points = np.dot(transformation_matrix, np.transpose(points)).T
		points_yolo = np.hstack((points[:,:3], np.asarray(points_yolo)[:,3:]))

		points_yolo, bboxes, classes = clusterInstances(np.asarray(points_yolo))
		print(f"Number of points: {len(points_yolo)}, Number of BBOX: {len(bboxes)}")

		for bbox in bboxes:
			# Calculate volume of the bounding box
			extent = bbox.get_extent()
			volume = extent[0] * extent[1] * extent[2]
			
			# Only add boxes with volume greater than threshold (e.g., 0.001 cubic meters)
			if volume > 0.004:  # Adjust threshold as needed
				self.global_bboxes.append(bbox)
			# self.global_bboxes.append(bbox)

		# ## Transform the bounding boxes based on odometry pose
		# for bbox in bboxes:
		# 	bbox = bbox.get_oriented_bounding_box()
		# 	## Get the box center in homogeneous coordinates
		# 	center = np.hstack((bbox.center, 1))
		# 	## Apply the transformation
		# 	center = np.dot(transformation_matrix, center)
		# 	## Update the bbxo components to match the new orientation
		# 	bbox.center = center[:3]
		# 	## Rotate the bbox.R
		# 	bbox.R = np.dot(rotation, bbox.R)
		# 	## Append the bbox to the global list
		# 	self.global_bboxes.append(bbox)
		
		## Keep a color for each bbox for instance coloring
		#self.colors += [np.random.rand(3) for _ in range(len(bboxes))]
		## Keep a color for each bbox for class coloring
		# self.colors += [np.array(self.cmaplist[int(classes[i])])[:3] for i in range(len(bboxes))]

		# Publish the bounding boxes. Make a marker array for each bbox in bboxes
		# marker_array = publishBboxes(self.global_bboxes, color=(1.0, 1.0, 0.0))
		# self.bbox_pub.publish(marker_array)

		toc = time.time()
		## Merge bounding boxes
		self.global_bboxes = mergeBboxes(self.global_bboxes)
		tic = time.time()
		print(f"Time to merge bboxes: {tic-toc}")

		# print(self.global_bboxes)

		## Publish the bounding boxes. Make a marker array for each bbox in bboxes
		maeker_array = self.publishBboxes(self.global_bboxes, time=odom_data.header.stamp)
		self.bbox_pub.publish(maeker_array)

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

		print("BOX3D!")
			
		# except:
		# 	pass

	def publishBboxes(self, bboxes:List[o3d.geometry.AxisAlignedBoundingBox], color=None, time=None):
		## Publish the bounding boxes. Make a marker array for each bbox in bboxes
		markerArray = MarkerArray()
		for idx, bbox in enumerate(bboxes):
			marker = Marker()
			marker.header.frame_id = FRAME
			marker.header.stamp = rospy.Time.now() if time is None else time
			marker.id = idx
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.pose.position.x = bbox.get_center()[0]
			marker.pose.position.y = bbox.get_center()[1]
			marker.pose.position.z = bbox.get_center()[2]
			# marker.pose.orientation.x = R.from_matrix([bbox.R]).as_quat()[0][0]
			# marker.pose.orientation.y = R.from_matrix([bbox.R]).as_quat()[0][1]
			# marker.pose.orientation.z = R.from_matrix([bbox.R]).as_quat()[0][2]
			# marker.pose.orientation.w = R.from_matrix([bbox.R]).as_quat()[0][3]
			marker.scale.x = (bbox.get_max_bound()[0] - bbox.get_min_bound()[0])*1
			marker.scale.y = (bbox.get_max_bound()[1] - bbox.get_min_bound()[1])*1
			marker.scale.z = (bbox.get_max_bound()[2] - bbox.get_min_bound()[2])*1
			marker.color.a = 0.35#35
			## Get random color for every bbox
			if color is None:
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.0
			else:
				marker.color.r = color[idx][0]
				marker.color.g = color[idx][1]
				marker.color.b = color[idx][2]
			markerArray.markers.append(marker)
		# self.bbox_pub.publish(markerArray)
		print(f"Marker Ready, number of markers: {len(markerArray.markers)}")

		return markerArray

if __name__ == '__main__':
	rospy.init_node('lidar_camera_aligment', anonymous=True)
	yt = yolo_lidar()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down") 
