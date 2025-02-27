import numpy as np
from typing import List
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN

FRAME = "world"

def clusterInstances(points:np.ndarray) -> np.ndarray:
    """ Cluster the points based within an instance using euclidean distance in order to filter misclassified points
    """
    ## Initialize lists to hold the final points, bounding boxes, and classes
    final_points = []
    bboxs = []
    classes = []
    
    ## Loop through each unique class in the points
    for class_label in np.unique(points[:,4]):
        if class_label == 0:  # Omit if class is 0
            continue
        class_points = points[points[:,4] == class_label]
        num_instances = len(np.unique(class_points[:,3]))
        print(f"Number of instances for class {class_label}: {num_instances}")

        for instance in range(num_instances):
            instance_points = class_points[class_points[:,3] == instance+1][:,:3]
            print(f"Instance {instance+1} for class {class_label} has {len(instance_points)} points.")
            if len(instance_points) > 10:
                cluster = DBSCAN(eps=0.2, min_samples=10, algorithm="auto", n_jobs=-1).fit(instance_points)
                unique_labels = set(cluster.labels_)
                centers = []
                for k in unique_labels:
                    class_member_mask = (cluster.labels_ == k)
                    centers.append(np.mean(instance_points[class_member_mask], axis=0))
                centers = np.asarray(centers)
                closest_center = np.argmin(np.linalg.norm(centers, axis=1))
                class_member_mask = (cluster.labels_ == closest_center)
                instance_points = instance_points[class_member_mask]
                if len(instance_points) > 4:
                    bbox = getBoundingBox(instance_points)
                    bboxs.append(bbox)
                    classes.append(class_label)
                    # Add instance id and class id to each point in instance_points
                    instance_points_with_id = np.hstack((instance_points, np.full((len(instance_points), 2), [instance+1, class_label])))
                    final_points.append(instance_points_with_id)
                    print(f"Instance {instance+1} for class {class_label} has {len(instance_points)} points and a bounding box.")
                else:
                    class_points = class_points[class_points[:,3] != instance]
                    print(f"Instance {instance+1} for class {class_label} has less than 4 points and is being omitted.")
    
    ## Stack all the final points together
    if final_points:  # Check if final_points is not empty
        final_points = np.vstack(final_points)
    else:
        final_points = np.empty((0, 5))  # Return an empty array with the correct shape if no points were added
        
    return final_points, bboxs, classes

def getBoundingBox(points:np.ndarray) -> o3d.geometry.AxisAlignedBoundingBox:
    """ Get the bounding box of the points using open3d
    """
    ## Create the pointcloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    ## Get the axis aligned bounding box
    bbox = pcd.get_axis_aligned_bounding_box()
    return bbox

def mergeBboxes(bboxes:List[o3d.geometry.AxisAlignedBoundingBox]) -> List[o3d.geometry.AxisAlignedBoundingBox]:
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
            if isOverlapping(merged_bbox, bbox2):
                merged_bbox = mergeBoxes(merged_bbox, bbox2)
                overlapping_indices.append(idx2 + idx1 + 1)	
        merged_bboxes.append(merged_bbox)
        processed.update(overlapping_indices)
    return merged_bboxes

def mergeBoxes(bbox1:o3d.geometry.AxisAlignedBoundingBox, bbox2:o3d.geometry.AxisAlignedBoundingBox) -> o3d.geometry.AxisAlignedBoundingBox:
    """ Merge two bounding boxes
    """
    ## Get the points from the boxes
    points1 = np.asarray(bbox1.get_box_points())
    points2 = np.asarray(bbox2.get_box_points())
    ## Merge the points
    points = np.vstack((points1, points2))
    ## Get the bounding box of the merged points
    bbox = getBoundingBox(points)
    return bbox

def isOverlapping(bbox1:o3d.geometry.AxisAlignedBoundingBox, bbox2:o3d.geometry.AxisAlignedBoundingBox) -> bool:
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

def projectPoints(points3D, rvec, tvec, proyection_matrix):
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