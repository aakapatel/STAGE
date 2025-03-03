import rospy
from visualization_msgs.msg import MarkerArray, Marker
from stage_planner.msg import Graph  # Corrected import path for Graph message

import numpy as np
from collections import deque
from geometry_msgs.msg import Point

class GraphNode:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/hummingbird/semantic_graph', Graph, self.graph_callback)
        # self.subscriber = rospy.Subscriber('/spot/semantic_graph', Graph, self.graph_callback)

        self.publisher = rospy.Publisher('/graph/areas', MarkerArray, queue_size=30)
        # self.graph_data = []  # List to store graph data
        # self.adjacency_matrix = []  # Adjacency matrix to store edges
        # self.vertex_coordinates = []  # List to store vertex coordinates
        # self.is_semantic_list = []  # List to store is_semantic attributes
        self.marker_array = MarkerArray()

    def graph_callback(self, msg):

        # Check if the existing marker array is empty
        # if not self.marker_array.markers:  # Check if the marker array is empty
        # self.delete_marker_array(self.marker_array)  # Call the delete function if not empty
            # self.marker_array = MarkerArray()

        self.graph_data = []  # List to store graph data
        self.adjacency_matrix = []  # Adjacency matrix to store edges
        self.vertex_coordinates = []  # List to store vertex coordinates
        self.is_semantic_list = []  # List to store is_semantic attributes

        # Initialize lists for vertex coordinates and is_semantic attributes
        num_vertices = len(msg.vertices)  # Moved this line up to avoid using uninitialized variable
        rospy.loginfo(f"Received {num_vertices} vertices from the graph message.")
        self.vertex_coordinates = [None] * num_vertices
        self.is_semantic_list = [False] * num_vertices
        
        for vertex in msg.vertices:
            vertex_id = vertex.id  # Assuming vertex has an 'id' attribute
            self.vertex_coordinates[vertex_id] = (vertex.pose.position.x, vertex.pose.position.y, vertex.pose.position.z)  # Save 3D coordinates
            self.is_semantic_list[vertex_id] = vertex.is_semantic  # Save is_semantic attribute

        # rospy.loginfo(f"Received {self.is_semantic_list} vertices from the graph message.")

        # Convert vertex_coordinates to a NumPy array
        self.vertex_coordinates = np.array(self.vertex_coordinates)

        # Create or update the adjacency matrix
        self.adjacency_matrix = np.zeros((num_vertices, num_vertices), dtype=int)  # Initialize with zeros
        
        for edge in msg.edges:
            source_id = edge.source_id
            target_id = edge.target_id
            self.adjacency_matrix[source_id, target_id] = 1  # Mark the edge in the adjacency matrix
            self.adjacency_matrix[target_id, source_id] = 1  # Mark the edge in the adjacency matrix

        print(len(self.vertex_coordinates))
        print(len(msg.edges))

        # print(self.adjacency_matrix)

        rospy.loginfo("Adjacency matrix created. Propagating weights.")
        # Propagate
        weight_matrix = self.propagate_weights_adaptive(self.vertex_coordinates,self.is_semantic_list,self.adjacency_matrix)
        # print(*weight_matrix)
        cluster_centroids, object_cluster_matrix = self.cluster_objects(self.vertex_coordinates,self.is_semantic_list,self.adjacency_matrix, weight_matrix, z_threshold=5)
        rospy.loginfo(f"Clustered objects into {len(cluster_centroids)} clusters.")

        self.delete_marker_array(self.marker_array) 
        self.marker_array = self.create_marker_array(self.vertex_coordinates, cluster_centroids, object_cluster_matrix, self.adjacency_matrix)
        
        # Process the graph message and populate the marker_array
        self.publisher.publish(self.marker_array)
        rospy.loginfo("Published marker array.")

    def propagate_weights_adaptive(self, nodes, object_nodes, adjacency_matrix):
        """
        Propagates weights from the object nodes throughout the graph.
        
        Args:
            nodes (list or np.ndarray): List of existing node coordinates.
            object_nodes (list of bool): List of booleans indicating which nodes are objects.
            adjacency_matrix (np.ndarray): Adjacency matrix for the entire graph.
        
        Returns:
            np.ndarray: Weight matrix with propagated weights along the edges.
        """
        
        # Combine nodes in the order [nodes] to match the adjacency matrix.
        num_nodes = len(nodes)
        
        # Check that the adjacency matrix matches the expected shape.
        assert adjacency_matrix.shape == (num_nodes, num_nodes), "Invalid adjacency matrix shape"
        
        # Initialize the weight matrix and a tracker for maximum propagation value per node.
        weight_matrix = np.zeros_like(adjacency_matrix, dtype=int)
        max_prop = np.full(num_nodes, -1, dtype=int)
        
        # Set up a queue for BFS: (node_index, current_propagation_value)
        queue = deque()
        
        # Identify object indices based on the boolean list
        object_indices = [i for i, is_object in enumerate(object_nodes) if is_object]
        
        # Calculate the edge propagation value for each object node based on the distance to its 3 closest object nodes.
        for idx in object_indices:
            # Find the distances to all other nodes using BFS
            distances = np.full(num_nodes, np.inf)  # Initialize distances with infinity
            distances[idx] = 0  # Distance to itself is 0
            queue_bfs = deque([idx])
            
            while queue_bfs:
                current = queue_bfs.popleft()
                for neighbor in np.where(adjacency_matrix[current] == 1)[0]:
                    if distances[neighbor] == np.inf:  # Not visited
                        distances[neighbor] = distances[current] + 1
                        queue_bfs.append(neighbor)

            # Find the 3 closest object nodes based on the calculated distances
            closest_neighbors = np.argsort(distances[object_indices])[:3]  # Sort only among object nodes
            closest_neighbors = [object_indices[i] for i in closest_neighbors]  # Get global indices
            
            # Calculate the edge propagation value based on the mean distance - min distance of the closest neighbors
            if closest_neighbors:
                min_distance = distances[closest_neighbors[0]]
                total_distance = sum(distances[neighbor] for neighbor in closest_neighbors if distances[neighbor] < np.inf)
                mean_distance = total_distance / len(closest_neighbors)
                edge_propagation = int(mean_distance - min_distance)
            else:
                edge_propagation = 0

            # print(total_distance)
                    
            # # Calculate the edge propagation value based on the closest neighbors
            # edge_propagation = np.inf
            # for neighbor in closest_neighbors:
            #     if distances[neighbor] < np.inf:  # Ensure the neighbor is reachable
            #         edge_propagation = min(edge_propagation, int(distances[neighbor] / 2 + 2))

            # print(edge_propagation)

            # rospy.loginfo(f"Edge propagation value for node {idx}: {edge_propagation}")
            
            if edge_propagation > max_prop[idx]:
                max_prop[idx] = edge_propagation
                queue.append((idx, edge_propagation))
        
        # Propagate weights using BFS.
        while queue:
            node, current_prop = queue.popleft()
            
            # Retrieve all neighbors of the current node from the adjacency matrix.
            neighbors = np.where(adjacency_matrix[node] == 1)[0]
            
            for neighbor in neighbors:
                # Update the edge weight if the current propagation value is higher than the existing one.
                if current_prop > weight_matrix[node][neighbor]:
                    weight_matrix[node][neighbor] = current_prop
                    weight_matrix[neighbor][node] = current_prop  # Ensure the graph is undirected.
                
                # Compute the propagation for the next step.
                new_prop = current_prop - 1
                if new_prop > 0 and new_prop > max_prop[neighbor]:
                    max_prop[neighbor] = new_prop
                    queue.append((neighbor, new_prop))
        
        return weight_matrix
    
    def cluster_objects(self, nodes, object_nodes, adjacency_matrix, weight_matrix, z_threshold):
        """
        Correctly clusters object nodes based on connectivity through edges with weight > 0.
        
        Args:
            nodes (list or np.ndarray): List of existing node coordinates.
            object_nodes (list of bool): List of booleans indicating which nodes are objects.
            adjacency_matrix (np.ndarray): Adjacency matrix for the entire graph.
        """
        num_places = len(nodes) - sum(object_nodes)  # Calculate number of places based on object_nodes
        num_objects = sum(object_nodes)  # Count the number of objects
        total_nodes = len(nodes)  # Total nodes is now just the length of nodes

        # Validate input shapes
        # assert adjacency_matrix.shape == (total_nodes, total_nodes), "Adjacency matrix shape mismatch"
        # assert weight_matrix.shape == (total_nodes, total_nodes), "Weight matrix shape mismatch"

        # Debugging: Log the adjacency and weight matrices
        rospy.loginfo(f"Adjacency Matrix: {adjacency_matrix}")
        rospy.loginfo(f"Weight Matrix: {weight_matrix}")

        # Track visited status for all nodes
        visited_all = np.zeros(total_nodes, dtype=bool)

        # Track clustered object nodes
        object_indices = [i for i in range(total_nodes) if object_nodes[i]]  # List of indices for object nodes
        num_objects = len(object_indices)  # Count the number of objects
        object_visited = np.zeros(num_objects, dtype=bool)
        clusters = []

        for start_global in object_indices:
            local_idx = object_indices.index(start_global)  # Get the local index based on object_indices
            if object_visited[local_idx]:  # Ensure local_idx is valid
                continue

            queue = deque([start_global])
            cluster = []
            visited_all[start_global] = True

            while queue:
                current_global = queue.popleft()

                # Add object nodes to the cluster
                if object_nodes[current_global]:
                    local_idx = object_indices.index(current_global)  # Get the local index based on object_indices
                    if not object_visited[local_idx]:
                        object_visited[local_idx] = True
                        cluster.append(local_idx)

                # Find valid neighbors (adjacent + weight > 0)
                neighbors = np.where(
                    (adjacency_matrix[current_global] == 1) & 
                    (weight_matrix[current_global] > 0)
                )[0]

                for neighbor in neighbors:
                    if not visited_all[neighbor]:
                        visited_all[neighbor] = True
                        queue.append(neighbor)

            if cluster:
                clusters.append(cluster)

        # Log the identified clusters
        rospy.loginfo(f"Identified clusters: {clusters}")

        # Compute cluster centroids with Z adjustment
        cluster_centroids = []
        for cluster in clusters:
            # Use global indices to access the nodes
            global_indices = [object_indices[local_idx] for local_idx in cluster]
            centroid = np.mean(nodes[global_indices], axis=0)  # Ensure nodes is a NumPy array
            centroid[2] = z_threshold  # Adjust Z coordinate
            cluster_centroids.append(centroid.tolist())

        # Build object-cluster adjacency matrix
        object_cluster_matrix = np.zeros((len(nodes), len(clusters)), dtype=int)  # Adjusted shape
        for cluster_idx, cluster in enumerate(clusters):
            for local_idx in cluster:
                global_idx = object_indices[local_idx]  # Get the global index from the local index
                object_cluster_matrix[global_idx, cluster_idx] = 1  # Connect clusters to their objects

        return cluster_centroids, object_cluster_matrix

    def create_marker_array(self, vertex_coordinates, cluster_centroids, object_cluster_matrix, adjacency_matrix):
        marker_array = MarkerArray()
        
        # Create markers for cluster centroids
        for idx, centroid in enumerate(cluster_centroids):
            # Check if the cluster has more than one node or if it has one node with connections
            if sum(object_cluster_matrix[:, idx]) > 1 or np.any(adjacency_matrix[object_cluster_matrix[:, idx] == 1, :].sum(axis=0) > 0):
                marker = Marker()
                marker.header.frame_id = "world"  # Set the frame of reference
                marker.header.stamp = rospy.Time.now()
                marker.ns = "clusters"
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = centroid[0]
                marker.pose.position.y = centroid[1]
                marker.pose.position.z = centroid[2]
                marker.scale.x = 1.5  # Size of the cube
                marker.scale.y = 1.5
                marker.scale.z = 1.5
                marker.color.r = 1.0  # Orange color
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0  # Fully opaque
                marker_array.markers.append(marker)

                # Create line markers from centroid to its respective objects
                for obj_idx in range(len(vertex_coordinates)):
                    if object_cluster_matrix[obj_idx, idx] == 1:  # If the object belongs to this cluster
                        line_marker = Marker()
                        line_marker.header.frame_id = "world"
                        line_marker.header.stamp = rospy.Time.now()
                        line_marker.ns = "lines"
                        line_marker.id = len(marker_array.markers) + obj_idx  # Unique ID
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        line_marker.pose.orientation.w = 1.0  # No rotation
                        line_marker.scale.x = 0.2  # Line thickness
                        line_marker.color.r = 1.0  # Orange color for lines
                        line_marker.color.g = 0.5
                        line_marker.color.b = 0.0
                        line_marker.color.a = 1.0

                        # Add points for the line
                        line_marker.points.append(Point(centroid[0], centroid[1], centroid[2]))  # Centroid position
                        line_marker.points.append(Point(vertex_coordinates[obj_idx][0], vertex_coordinates[obj_idx][1], vertex_coordinates[obj_idx][2]))  # Object position

                        marker_array.markers.append(line_marker)

        # Create a purple square marker at the centroid position
        purple_square_marker = Marker()
        purple_square_marker.header.frame_id = "world"
        purple_square_marker.header.stamp = rospy.Time.now()
        purple_square_marker.ns = "purple_squares"
        purple_square_marker.id = len(marker_array.markers)  # Unique ID
        purple_square_marker.type = Marker.CUBE
        purple_square_marker.action = Marker.ADD
        purple_square_marker.pose.position.x = np.mean([centroid[0] for centroid in cluster_centroids])
        purple_square_marker.pose.position.y = np.mean([centroid[1] for centroid in cluster_centroids])
        purple_square_marker.pose.position.z = np.mean([centroid[2] for centroid in cluster_centroids]) + 10
        purple_square_marker.scale.x = 2.5  # Size of the cube
        purple_square_marker.scale.y = 2.5
        purple_square_marker.scale.z = 2.5
        purple_square_marker.color.r = 0.5  # Purple color
        purple_square_marker.color.g = 0.0
        purple_square_marker.color.b = 0.5
        purple_square_marker.color.a = 1.0  # Fully opaque
        marker_array.markers.append(purple_square_marker)

        # Create edges from the purple square to the clusters
        for idx, centroid in enumerate(cluster_centroids):
            if sum(object_cluster_matrix[:, idx]) > 1 or np.any(adjacency_matrix[object_cluster_matrix[:, idx] == 1, :].sum(axis=0) > 0):
                edge_marker = Marker()
                edge_marker.header.frame_id = "world"
                edge_marker.header.stamp = rospy.Time.now()
                edge_marker.ns = "edges"
                edge_marker.id = len(marker_array.markers) + idx  # Unique ID
                edge_marker.type = Marker.LINE_STRIP
                edge_marker.action = Marker.ADD
                edge_marker.pose.orientation.w = 1.0  # No rotation
                edge_marker.scale.x = 0.3  # Line thickness
                edge_marker.color.r = 0.5  
                edge_marker.color.g = 0.0
                edge_marker.color.b = 0.5
                edge_marker.color.a = 0.70

                # Add points for the edge
                edge_marker.points.append(Point(np.mean([centroid[0] for centroid in cluster_centroids]), np.mean([centroid[1] for centroid in cluster_centroids]), np.mean([centroid[2] for centroid in cluster_centroids]) + 10))  # Centroid position
                edge_marker.points.append(Point(centroid[0], centroid[1], centroid[2])) # Object position

                marker_array.markers.append(edge_marker)

        return marker_array

    def delete_marker_array(self, marker_array):
        """
        Republish the given marker array with the action set to delete.
        
        Args:
            marker_array (MarkerArray): The marker array to delete.
        """
        delete_marker_array = MarkerArray()
        for marker in marker_array.markers:
            delete_marker = Marker()
            delete_marker.header = marker.header  # Keep the same header
            delete_marker.ns = marker.ns
            delete_marker.id = marker.id
            delete_marker.action = Marker.DELETE  # Set action to delete
            delete_marker_array.markers.append(delete_marker)

        # Publish the delete marker array
        self.publisher.publish(delete_marker_array)
        rospy.loginfo("Published delete marker array.")

if __name__ == '__main__':
    rospy.init_node('graph_node')
    node = GraphNode()
    rospy.spin()
