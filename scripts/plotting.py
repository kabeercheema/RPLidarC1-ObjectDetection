import numpy as np
import rtmaps.base_component
import rtmaps.types
from rtmaps.base_component import BaseComponent
import cv2
import time

class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        self.img_w = 1200   # Image width in pixels
        self.img_h = 800    # Plotting area height (excluding margin)
        self.margin_bottom = 60  # Space for x-axis labels
        # Set world (real) coordinate system (mm)
        self.xmin = -10000
        self.xmax = 10000
        self.ymin = 0
        self.ymax = 10000
        # For buffering last valid frame
        self.last_valid_centroids = np.zeros((0,2))
        self.last_valid_bounds = np.zeros((0,5))
        self.last_valid_points = np.zeros((0,2))

    def Dynamic(self):
        self.add_input("clusters_in", rtmaps.types.FLOAT64)
        self.add_input("bounds_in", rtmaps.types.FLOAT64)
        self.add_input("points_in", rtmaps.types.FLOAT64)

    def Birth(self):
        self.last_draw_time = time.time()
        cv2.namedWindow("RTMaps CV Clusters", cv2.WINDOW_NORMAL)

    def world2img(self, x, y):
        """
        Convert world (mm) to image pixel coordinates.
        """
        ix = int((x - self.xmin) / (self.xmax - self.xmin) * self.img_w)
        iy = int(self.img_h - (y - self.ymin) / (self.ymax - self.ymin) * self.img_h)
        return ix, iy

    def draw_axes(self, img):
        """
        Draw X and Y axes ticks and labels on image (with bottom margin).
        """
        num_ticks = 10
        # X-axis ticks/labels (bottom, in margin)
        for i in range(num_ticks + 1):
            x_val = self.xmin + i * (self.xmax - self.xmin) / num_ticks
            ix, iy0 = self.world2img(x_val, self.ymin)
            # Tick line (at plot area bottom edge)
            cv2.line(img, (ix, self.img_h-1), (ix, self.img_h+7), (150, 150, 150), 1)
            # Label (centered in margin)
            cv2.putText(img, f"{int(x_val)}", (ix-25, self.img_h + self.margin_bottom - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80,80,80), 1)
        # Y-axis ticks/labels (left)
        for i in range(num_ticks + 1):
            y_val = self.ymin + i * (self.ymax - self.ymin) / num_ticks
            ix0, iy = self.world2img(self.xmin, y_val)
            # Tick line
            cv2.line(img, (0, iy), (8, iy), (150, 150, 150), 1)
            # Label (aligned left)
            cv2.putText(img, f"{int(y_val)}", (12, iy+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80,80,80), 1)
        # X-axis label (in margin, below plot)
        cv2.putText(img, "X (mm)", (self.img_w//2 - 40, self.img_h + self.margin_bottom - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        # Y-axis label (vertical left)
        cv2.putText(img, "Y (mm)", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        # Frame around main plotting area
        cv2.rectangle(img, (0,0), (self.img_w-1, self.img_h-1), (0,0,0), 2)

    def Core(self):
        # --- Get centroids ---
        clusters_ioelt = self.inputs["clusters_in"].ioelt if "clusters_in" in self.inputs else None
        if clusters_ioelt is not None and hasattr(clusters_ioelt, 'data') and clusters_ioelt.data is not None:
            clusters_array = np.array(clusters_ioelt.data, dtype=np.float64)
            centroids = clusters_array.reshape(-1, 2) if clusters_array.size >= 2 else np.zeros((0,2))
        else:
            centroids = np.zeros((0,2))

        # --- Get bounds (with sticky width per cluster) ---
        bounds_ioelt = self.inputs["bounds_in"].ioelt if "bounds_in" in self.inputs else None
        if bounds_ioelt is not None and hasattr(bounds_ioelt, 'data') and bounds_ioelt.data is not None:
            bounds_array = np.array(bounds_ioelt.data, dtype=np.float64)
            bounds = bounds_array.reshape(-1, 5) if bounds_array.size >= 5 else np.zeros((0,5))
        else:
            bounds = np.zeros((0,5))

        # --- Get all detection points ---
        points_ioelt = self.inputs["points_in"].ioelt if "points_in" in self.inputs else None
        if points_ioelt is not None and hasattr(points_ioelt, 'data') and points_ioelt.data is not None:
            points_array = np.array(points_ioelt.data, dtype=np.float64)
            points = points_array.reshape(-1, 2) if points_array.size >= 2 else np.zeros((0,2))
        else:
            points = np.zeros((0,2))

        # --- Only plot if all arrays are valid and counts match ---
        valid = (centroids.shape[0] > 0 and bounds.shape[0] > 0 and centroids.shape[0] == bounds.shape[0])
        if valid:
            self.last_valid_centroids = centroids
            self.last_valid_bounds = bounds
            self.last_valid_points = points

        # Use last valid data
        centroids = self.last_valid_centroids
        bounds = self.last_valid_bounds
        points = self.last_valid_points

        # --- Draw every 0.01s (100Hz, adjust as needed) ---
        current_time = time.time()
        if current_time - self.last_draw_time >= 0.01:
            # Create image (with margin for axis labels)
            img = np.full((self.img_h + self.margin_bottom, self.img_w, 3), 255, dtype=np.uint8)

            # Draw all detection points (blue)
            for px, py in points:
                ix, iy = self.world2img(px, py)
                cv2.circle(img, (ix, iy), 3, (255, 0, 0), -1)

            # Draw cluster centroids (red X)
            for cx, cy in centroids:
                ix, iy = self.world2img(cx, cy)
                cv2.drawMarker(img, (ix, iy), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)

            # Draw bounds as green lines and width labels
            for i in range(min(centroids.shape[0], bounds.shape[0])):
                x1, y1, x2, y2, width = bounds[i]
                ix1, iy1 = self.world2img(x1, y1)
                ix2, iy2 = self.world2img(x2, y2)
                cv2.line(img, (ix1, iy1), (ix2, iy2), (0, 200, 0), 3)
                # Label at centroid
                cx, cy = centroids[i]
                icx, icy = self.world2img(cx, cy)
                cv2.putText(img, f"{width:.1f} mm", (icx, icy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (34, 139, 34), 2)

            # Draw axes, ticks, and labels
            self.draw_axes(img)

            cv2.imshow("RTMaps CV Clusters", img)
            cv2.waitKey(1)
            self.last_draw_time = current_time

    def Death(self):
        cv2.destroyAllWindows()
