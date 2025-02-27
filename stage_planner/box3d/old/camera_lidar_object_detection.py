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
from sklearn.cluster import KMeans
import numpy as np
from typing import List
import open3d as o3d
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sklearn.cluster import DBSCAN
import time

MAX_RANGE = 35
UNDISTORT = False
FRAME = "world"
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
		self.depth_pub = rospy.Publisher("/lidar/aligned_range_to_image/image_raw", Image, queue_size = 1)
		self.reflect_ol_pub = rospy.Publisher("/lidar/aligned_reflectivity_to_image/image_overlay", Image, queue_size = 1)
		self.reflect_pub = rospy.Publisher("/lidar/aligned_reflectivity_to_image/image_raw", Image, queue_size = 1)
		self.bbox_pub = rospy.Publisher("/lidar/aligned_bounding_boxes", MarkerArray, queue_size = 1)
		# self.global_seg_pub = rospy.Publisher("/lidar/segmented_map", PointCloud2, queue_size = 1)

		self.bridge = CvBridge()

		######################## CAMERA INFO TOPIC ############################
		#self.intrinsics_sub = rospy.Subscriber("/spot/camera/color/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)
		#self.intrinsics_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.callback_intrinsics, queue_size=1)
		
		###################### REAL SENSE CAMERA #########################	
		# self.image_sub = message_filters.Subscriber("/spot/camera/color/image_raw", Image)
		# self.points_sub = message_filters.Subscriber("/spot/velodyne_points",PointCloud2)
		# self.odometry_sub = message_filters.Subscriber("/dlo/odom_node/odom", Odometry)
		# self.height = 720
		# self.width = 1280
		# self.camera_matrix = np.asarray([635.2752075195312, 0.0, 652.32861328125, 0.0, 634.6666259765625, 358.4922180175781, 0.0, 0.0, 1.0]).reshape((3, 3))
		# self.dist_coeffs = np.asarray([-0.05529356002807617, 0.06882049143314362, -0.00044326853821985424, 0.0005935364752076566, -0.021897781640291214])
		# self.tvec = np.asarray([0.10,0.00,-0.10])
		# self.rvec,_ = cv2.Rodrigues(np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]]))

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
		self.image_sub = message_filters.Subscriber("/kitti/camera_color_left/image", Image)
		self.points_sub = message_filters.Subscriber("/kitti/velo/pointcloud", PointCloud2)
		self.odometry_sub = message_filters.Subscriber("/kitti/dlo/odom_node/odom", Odometry)
		self.glboal_map_sub = message_filters.Subscriber("/kitti/dlo/map_node/map", PointCloud2)

		self.height = 376
		self.width = 1241
		self.camera_matrix = np.asarray([718.856, 0.0, 607.1928, 0.0, 718.856, 185.2157, 0.0, 0.0, 1.0]).reshape((3, 3))
		# self.camera_matrix = np.asarray([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).reshape((3, 3))
		# self.camera_matrix = np.asarray([718.856, 0.0, 607.1928, 45.38225, 0.0, 718.856, 185.2157, -0.1130887, 0.0, 0.0, 1.0, 0.003779761]).reshape((3, 4))
		self.dist_coeffs = np.zeros((5, 1), np.float32)
		# self.dist_coeffs = np.asarray([718.856, 0.0, 607.1928, 45.38225, 0.0, 718.856, 185.2157, -0.1130887, 0.0, 0.0, 1.0, 0.003779761])
		#self.tvec = np.zeros((3, 1), np.float32)
		self.tvec = 
		
		 

		# self.rvec = np.asarray([[0.0,0.0,-1.0],[1.0,0.0,0.0],[0.0,1.0,0.0]])

		#############
		# self.camera_matrix = np.zeros((3, 3), np.float32)
		# self.dist_coeffs = np.zeros((5, 1), np.float32)
		# self.rvec = np.zeros((3, 1), np.float32)
		# self.tvec = np.zeros((3, 1), np.float32)
		############

		#self.cmap = plt.cm.viridis
		self.cmap = plt.cm.jet
		self.cmaplist = [self.cmap(i) for i in range(self.cmap.N)]

		#self.ts = message_filters.ApproximateTimeSynchronizer([self.points_sub, self.image_sub], queue_size=1, slop=0.5)
		self.ts = message_filters.TimeSynchronizer([self.points_sub, self.image_sub, self.odometry_sub], queue_size=1)
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

		points_i = []
		points_xyz = []
		for point in pcl2.read_points(pcl_data, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True):
			if sum(list(point[:-1])) != 0.0:# and point[0]>self.tvec[0]:
				points_i.append(point[-1])
				points_xyz.append(list(point[:-1]))
				#print(list(point[:-1]))
				#pointXYZ = np.array([[[point[:-1]]]], np.float32)

		## Remove points within 5 meteres radius
		points_xyz = [point for point in points_xyz if np.linalg.norm(point) > 0]

		pixels, _ = cv2.projectPoints(np.asarray(points_xyz), self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)
		# p_mat = np.asarray([718.856, 0.0, 607.1928, 45.38225, 0.0, 718.856, 185.2157, -0.1130887, 0.0, 0.0, 1.0, 0.003779761]).reshape((3,4))
		# pixels = self.projectPoints(np.asarray(points_xyz), rvec = self.rvec, tvec = self.tvec, proyection_matrix = p_mat)

		#print(pixels)

		if UNDISTORT:
			newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (self.width, self.height), 1, (self.width, self.height))
			frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, newcameramatrix)

		depth_ol = frame.copy()
		reflect_ol = frame.copy()

		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		results = self.model.predict(frame, conf=0.50, retina_masks=True, classes = OOI) 
		self.yolo_pub.publish(self.bridge.cv2_to_imgmsg(results[0].plot(), 'rgb8'))
		# boxes = results[0].boxes.cpu().numpy().data

		depth_img = np.zeros((self.height,self.width, 1), dtype = np.float32)
		reflect_img = np.zeros((self.height,self.width, 1), dtype = np.uint8)

		segMask = np.zeros((self.height,self.width, 1), dtype = np.uint8)
		instanceMask = np.zeros((self.height,self.width, 1), dtype = np.uint8)

		if results[0].masks is not None:
			masks = results[0].masks.cpu().numpy().data
			for idxMask, mask in enumerate(masks):
				#print(*mask.shape)

				#mask = mask.transpose(1, 2, 0).squeeze()
				mask = cv2.resize(mask, (self.width, self.height))
				mask = np.uint8(mask)
				kernel = np.ones((35, 35), np.uint8)
				mask = cv2.erode(mask, kernel) 

				#print(results[0].boxes[idxMask].cpu().numpy().data[0][5])

				#segMask = cv2.add(segMask, mask*int(results[0].boxes[idxMask].cpu().numpy().data[0][5] + 1)) 
				segMask = cv2.add(segMask, mask*(NAMES[results[0].boxes[idxMask].cpu().numpy().data[0][5]] + 1)) 
				instanceMask = cv2.add(instanceMask, mask*(idxMask + 1)) 

				#Reduce overlap
				#kernel = np.ones((3, 3), np.uint8)
				#mask = cv2.erode(mask, kernel) 
			
		self.yolo_seg_pub.publish(self.bridge.cv2_to_imgmsg(segMask, 'mono8'))
					
		points_yolo = []

		#img = np.zeros((self.height,self.width,3), dtype=np.uint8)
		for idxPixel, pixel in enumerate(pixels):
			#print(pixel[0][0])
			if pixel[0][0]<self.width and pixel[0][1]<self.height and pixel[0][0]>=0 and pixel[0][1]>=0 and points_xyz[idxPixel][0]>self.tvec[0]:
				#print(*pixel, points_xyz[idxPixel])
				#img[int(pixel[0]),int(pixel[1])] = (255,255,255)
				#print(tuple(int(pixel[0])))
				depth = int(np.linalg.norm(points_xyz[idxPixel])*255/MAX_RANGE)
				if depth > 255:
					depth = 255

				reflect = int(points_i[idxPixel])
				#print(depth)
				#frame = cv2.circle(frame, tuple([int(pixel[0][0]),int(pixel[0][1])]), 1, (255,0,depth), -1)
				color_range = self.cmaplist[depth]
				depth_ol = cv2.circle(depth_ol, tuple([int(pixel[0][0]), int(pixel[0][1])]), 1, (color_range[0]*255, color_range[1]*255, color_range[2]*255), -1)

				color_reflect = self.cmaplist[reflect]
				reflect_ol = cv2.circle(reflect_ol, tuple([int(pixel[0][0]), int(pixel[0][1])]), 1, (color_reflect[0]*255, color_reflect[1]*255, color_reflect[2]*255), -1)

				#depth_ol[int(pixel[0][1]),int(pixel[0][0])] = [color[0]*255, color[1]*255, color[2]*255]
				depth_img[int(pixel[0][1]),int(pixel[0][0])] = np.linalg.norm(points_xyz[idxPixel])
				reflect_img[int(pixel[0][1]),int(pixel[0][0])] = int(points_i[idxPixel])

				points_yolo.append([points_xyz[idxPixel][0], points_xyz[idxPixel][1], points_xyz[idxPixel][2], instanceMask[int(pixel[0][1]),int(pixel[0][0])], segMask[int(pixel[0][1]),int(pixel[0][0])]])

			else: points_yolo.append([points_xyz[idxPixel][0], points_xyz[idxPixel][1], points_xyz[idxPixel][2], 0, 0])

		## Cluster the points based within an instance using euclidean distance in order to filter misclassified points
		##  Try removing walls and road
		try:
			points_yolo, bboxes, classes = self.clusterInstances(np.asarray(points_yolo))
			
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

			## Transform the bounding boxes based on odometry pose
			for bbox in bboxes:
				bbox = bbox.get_oriented_bounding_box()
				## Get the box center in homogeneous coordinates
				center = np.hstack((bbox.center, 1))
				## Apply the transformation
				center = np.dot(transformation_matrix, center)
				## Update the bbxo components to match the new orientation
				bbox.center = center[:3]
				## Rotate the bbox.R
				bbox.R = np.dot(rotation, bbox.R)
				## Append the bbox to the global list
				self.global_bboxes.append(bbox)
			
			## Keep a color for each bbox for instance coloring
			#self.colors += [np.random.rand(3) for _ in range(len(bboxes))]
			## Keep a color for each bbox for class coloring
			self.colors += [np.array(self.cmaplist[classes[i]])[:3] for i in range(len(bboxes))]

			## Publish the bounding boxes. Make a marker array for each bbox in bboxes
			# self.publish_bboxes(self.global_bboxes, color=(1.0, 1.0, 0.0))
			
			toc = time.time()
			## Merge bounding boxes
			self.global_bboxes = self.mergeBboxes(self.global_bboxes)
			tic = time.time()
			print(f"Time to merge bboxes: {tic-toc}")

			## Publish the bounding boxes. Make a marker array for each bbox in bboxes
			self.publish_bboxes(self.global_bboxes, self.colors)

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
			self.reflect_ol_pub.publish(self.bridge.cv2_to_imgmsg(reflect_ol, 'bgr8'))

			self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_img, 'passthrough'))
			self.reflect_pub.publish(self.bridge.cv2_to_imgmsg(reflect_img, 'passthrough'))
			
		except:
			pass


	def publish_bboxes(self, bboxes:List[o3d.geometry.OrientedBoundingBox], color=None) -> None:
		## Publish the bounding boxes. Make a marker array for each bbox in bboxes
		markerArray = MarkerArray()
		for idx, bbox in enumerate(bboxes):
			marker = Marker()
			marker.header.frame_id = FRAME
			marker.header.stamp = rospy.Time.now()
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
			marker.scale.x = bbox.get_max_bound()[0] - bbox.get_min_bound()[0]
			marker.scale.y = bbox.get_max_bound()[1] - bbox.get_min_bound()[1]
			marker.scale.z = bbox.get_max_bound()[2] - bbox.get_min_bound()[2]
			marker.color.a = 0.7#35
			## Get random color for every bbox
			if color is None:
				marker.color.r = np.random.rand()
				marker.color.g = np.random.rand()
				marker.color.b = np.random.rand()
			else:
				marker.color.r = color[idx][0]
				marker.color.g = color[idx][1]
				marker.color.b = color[idx][2]
			markerArray.markers.append(marker)
		self.bbox_pub.publish(markerArray)

	def clusterInstances(self, points:np.ndarray) -> np.ndarray:
		""" Cluster the points based within an instance using euclidean distance in order to filter misclassified points
			Try removing walls and road
		"""
		## Get points from each label
		buses = points[points[:,4] == 1]
		trucks = points[points[:,4] == 2]
		cars = points[points[:,4] == 3]
		motorcycles = points[points[:,4] == 4]
		bicycles = points[points[:,4] == 5]
		bboxs = []
		centers = []
		classes = []
		## Get car from the instance by clustering and removing missclassified points
		if len(cars) > 0:
			## Check how many different instances are in the pointcloud
			num_instances = len(np.unique(cars[:,3]))
			print(f"Number of instances: {num_instances}")
			## For every instance, cluster the points
			for instance in range(num_instances):
				## Get the points of the instance
				instance_points = cars[cars[:,3] == instance][:,:3]
				## If the instance has more than 5 points, cluster the points
				if len(instance_points) > 10:
					## Cluster the points with DBSCAN
					cluster = DBSCAN(eps=0.2, min_samples=10, algorithm="auto", n_jobs=-1).fit(instance_points)
					## Get the center of each cluster
					unique_labels = set(cluster.labels_)
					centers = []
					for k in unique_labels:
						class_member_mask = (cluster.labels_ == k)
						centers.append(np.mean(instance_points[class_member_mask], axis=0))
					## Keep only the points from the cluster with the closest center to the origin
					centers = np.asarray(centers)
					## Get the closest cluster center to the origin
					closest_center = np.argmin(np.linalg.norm(centers, axis=1))
					## Get the mask of the points from the closest cluster
					class_member_mask = (cluster.labels_ == closest_center)														
					## Get the points from the closest cluster
					instance_points = instance_points[class_member_mask]
					if len(instance_points) > 4:
						## Get the bounding box
						bbox = self.getBoundingBox(instance_points)
						bboxs.append(bbox)
						classes.append(3)
						## Put points back in the cars array
						cars = cars[cars[:,3] != instance]
						cars = np.vstack((cars, np.hstack((instance_points, np.ones((len(instance_points),1))*3, np.ones((len(instance_points),1))*3))))
					else:
						cars = cars[cars[:,3] != instance]
			## Get the points from the other labels
			buses = points[points[:,4] == 1]
			trucks = points[points[:,4] == 2]
			motorcycles = points[points[:,4] == 4]
			bicycles = points[points[:,4] == 5]
			## Get the points from the other labels
			# points = np.vstack((buses, trucks, motorcycles, bicycles, cars))
			points = np.vstack((buses, trucks, motorcycles, bicycles, cars))
		## Get pedestrian from the instance by clustering and removing missclassified points
		if len(bicycles) > 0:
			## Check how many different instances are in the pointcloud
			num_instances = len(np.unique(bicycles[:,3]))
			print(f"Number of instances: {num_instances}")
			## For every instance, cluster the points
			for instance in range(num_instances):
				## Get the points of the instance
				instance_points = bicycles[bicycles[:,3] == instance][:,:3]
				## If the instance has more than 5 points, cluster the points
				if len(instance_points) > 10:
					## Cluster the points with DBSCAN
					cluster = DBSCAN(eps=0.2, min_samples=10, algorithm="auto", n_jobs=-1).fit(instance_points)
					## Get the center of each cluster
					unique_labels = set(cluster.labels_)
					centers = []
					for k in unique_labels:
						class_member_mask = (cluster.labels_ == k)
						centers.append(np.mean(instance_points[class_member_mask], axis=0))
					## Keep only the points from the cluster with the closest center to the origin
					centers = np.asarray(centers)
					## Get the closest cluster center to the origin
					closest_center = np.argmin(np.linalg.norm(centers, axis=1))
					## Get the mask of the points from the closest cluster
					class_member_mask = (cluster.labels_ == closest_center)														
					## Get the points from the closest cluster
					instance_points = instance_points[class_member_mask]
					if len(instance_points) > 4:
						## Get the bounding box
						bbox = self.getBoundingBox(instance_points)
						bboxs.append(bbox)
						classes.append(5)
						## Put points back in the cars array
						bicycles = bicycles[bicycles[:,3] != instance]
						bicycles = np.vstack((bicycles, np.hstack((instance_points, np.ones((len(instance_points),1))*5, np.ones((len(instance_points),1))*5))))
					else:
						bicycles = bicycles[bicycles[:,3] != instance]
		return cars, bboxs, classes

	def getBoundingBox(self, points:np.ndarray) -> o3d.geometry.AxisAlignedBoundingBox:
		""" Get the bounding box of the points using open3d
		"""
		## Create the pointcloud
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points)
		## Get the axis aligned bounding box
		bbox = pcd.get_axis_aligned_bounding_box()
		return bbox

	def mergeBboxes(self, bboxes:List[o3d.geometry.OrientedBoundingBox]) -> List[o3d.geometry.OrientedBoundingBox]:
		""" Check if boxes are overlapping and get all the points from the boxes. Then create the bbox of the merged points
		"""
		merged_bboxes = []
		processed = set()
		for idx1, bbox1 in enumerate(bboxes):
			if idx1 in processed:
				continue
			merged_bbox = bbox1
			overlapping_indices = [idx1]
			for idx2, bbox2 in enumerate(bboxes[idx1 + 1:]):
				if idx2 + idx1 + 1 in processed:
					continue	
				if self.isOverlapping(merged_bbox, bbox2):
					merged_bbox = self.mergeBoxes(merged_bbox, bbox2)
					overlapping_indices.append(idx2 + idx1 + 1)	
			merged_bboxes.append(merged_bbox)
			processed.update(overlapping_indices)
		return merged_bboxes
	
	def mergeBoxes(self, bbox1:o3d.geometry.OrientedBoundingBox, bbox2:o3d.geometry.OrientedBoundingBox) -> o3d.geometry.OrientedBoundingBox:
		""" Merge two bounding boxes
		"""
		## Get the points from the boxes
		points1 = np.asarray(bbox1.get_box_points())
		points2 = np.asarray(bbox2.get_box_points())
		## Merge the points
		points = np.vstack((points1, points2))
		## Get the bounding box of the merged points
		bbox = self.getBoundingBox(points)
		return bbox
	
	def isOverlapping(self, bbox1:o3d.geometry.OrientedBoundingBox, bbox2:o3d.geometry.OrientedBoundingBox) -> bool:
		""" Check if two bounding boxes are overlapping.
		"""
		## Get the points from the boxes
		points1 = np.asarray(bbox1.get_box_points())
		points2 = np.asarray(bbox2.get_box_points())
		## Get the minimum and maximum points from the boxes
		min1 = np.min(points1, axis=0)
		max1 = np.max(points1, axis=0)
		min2 = np.min(points2, axis=0)
		max2 = np.max(points2, axis=0)
		## Check if the boxes are overlapping
		overlapping = (min1[0] <= max2[0] and max1[0] >= min2[0]) and (min1[1] <= max2[1] and max1[1] >= min2[1]) and (min1[2] <= max2[2] and max1[2] >= min2[2])
		return overlapping

	def projectPoints(self, points3D, rvec, tvec, proyection_matrix):
		transformation_matrix = np.asarray([[rvec[0][0], rvec[0][1], rvec[0][2], tvec[0][0]],
									        [rvec[1][0], rvec[1][1], rvec[1][2], tvec[1][0]],
											[rvec[2][0], rvec[2][1], rvec[2][2], tvec[2][0]],
											[0.0, 0.0, 0.0, 1.0]])
		#print(transformation_matrix)
		points2D = []
		for point3D in points3D:
			point2D = np.dot(np.dot(proyection_matrix, transformation_matrix),np.transpose([point3D[0], point3D[1], point3D[2], 1.0]))
			points2D.append([[int(point2D[1]), int(point2D[0])]])
		#print(points2D)
		return points2D
	

if __name__ == '__main__':
	rospy.init_node('lidar_camera_aligment', anonymous=True)
	yt = yolo_lidar()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down") 
