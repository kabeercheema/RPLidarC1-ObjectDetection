import numpy as np
from sklearn.cluster import DBSCAN
import math
import itertools
import rtmaps.base_component
import rtmaps.types
from rtmaps.base_component import BaseComponent

class rtmaps_python(BaseComponent):
    """
    RTMaps Python bridge for lidar clustering and width calculation.
    Outputs:
      - xy_points: all valid (x, y) detections.
      - cluster_array: centroids of each detected cluster.
      - width_bounds_array: for each cluster, [pt1_x, pt1_y, pt2_x, pt2_y, width]
    """
    def __init__(self):
        # Required RTMaps base class initialization
        BaseComponent.__init__(self)

    def Dynamic(self):
        # Lidar input: flat array [angle0, dist0, angle1, dist1, ...]
        self.add_input("lidar", rtmaps.types.FLOAT64)

        # Output 1: All valid xy points as [x, y, x, y, ...]
        self.add_output("xy_points", rtmaps.types.FLOAT64, 4096)
        # Output 2: Cluster centroids [centroid_x, centroid_y, ...]
        self.add_output("cluster_array", rtmaps.types.FLOAT64, 128)
        # Output 3: For each cluster: [pt1_x, pt1_y, pt2_x, pt2_y, width]
        #         (width is distance between pt1 and pt2)
        self.add_output("width_bounds_array", rtmaps.types.FLOAT64, 320)  # 64 clusters * 5 floats

        # User properties for clustering
        self.add_property("max_distance_mm", 12000)     # Max valid distance (mm)
        self.add_property("dbscan_eps_mm", 200)         # DBSCAN epsilon (mm)
        self.add_property("dbscan_min_samples", 3)      # DBSCAN min samples

    def Birth(self):
        # Nothing needed for startup
        pass

    def Core(self):
        # ---- 1. Get lidar data and check validity ----
        in_data = np.array(self.inputs["lidar"].ioelt.data)
        # Must be even length: angle/dist pairs
        if len(in_data) % 2 != 0:
            return

        # ---- 2. Convert to Cartesian and filter ----
        max_dist = float(self.get_property("max_distance_mm"))
        points = []
        for i in range(0, len(in_data), 2):
            angle = in_data[i]
            dist = in_data[i + 1]
            # Reject invalid distances and those "behind" (angle 91–269)
            if dist <= 0 or dist > max_dist or math.isnan(dist) or (angle > 90 and angle < 270):
                continue
            rad = math.radians(-angle + 90)  # Change sign to fit common lidar conventions
            x = dist * math.cos(rad)
            y = dist * math.sin(rad)
            points.append([x, y])
        points_np = np.array(points, dtype=np.float64)  # (N, 2)

        # Output all points
        self.outputs["xy_points"].write(points_np)

        # ---- 3. Cluster using DBSCAN ----
        centroids = []         # Store cluster centroids
        width_bounds = []      # For each cluster: [pt1_x, pt1_y, pt2_x, pt2_y, width]

        # Only cluster if enough points
        if points_np.shape[0] >= self.get_property("dbscan_min_samples"):
            clustering = DBSCAN(
                eps=float(self.get_property("dbscan_eps_mm")),
                min_samples=int(self.get_property("dbscan_min_samples"))
            ).fit(points_np)
            labels = clustering.labels_

            # Iterate over each cluster label (except noise)
            for lbl in set(labels):
                if lbl == -1:
                    continue  # -1 is noise
                cluster_pts = points_np[labels == lbl]
                centroid = np.mean(cluster_pts, axis=0)
                centroids.append(centroid)

                # Find two points in cluster farthest apart (width endpoints)
                max_width = 0
                pt1 = pt2 = None
                if len(cluster_pts) >= 2:
                    for a, b in itertools.combinations(cluster_pts, 2):
                        dist = np.linalg.norm(a - b)
                        if dist > max_width:
                            max_width = dist
                            pt1, pt2 = a, b
                # If only one point, endpoints are the point itself
                if pt1 is None or pt2 is None:
                    pt1 = pt2 = cluster_pts[0]
                    max_width = 0.0
                # Store endpoints and width
                width_bounds.append([pt1[0], pt1[1], pt2[0], pt2[1], max_width])
        else:
            centroids = []
            width_bounds = []

        # ---- 4. Output clusters and width bounds ----
        # Output centroids as flat array
        cluster_array_np = np.array(centroids, dtype=np.float64).flatten()
        self.outputs["cluster_array"].write(cluster_array_np)

        # Output width bounds: [pt1_x, pt1_y, pt2_x, pt2_y, width, ...]
        width_bounds_np = np.array(width_bounds, dtype=np.float64).flatten()
        self.outputs["width_bounds_array"].write(width_bounds_np)

    def Death(self):
        # Nothing to clean up on exit
        pass
