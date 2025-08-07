import numpy as np
import rtmaps.base_component
import rtmaps.types
from rtmaps.base_component import BaseComponent

class StickyWidthMemory:
    """
    Stores sticky width for each cluster, updates only when width changes by threshold.
    """
    def __init__(self, max_distance=300, width_threshold=100):
        self.prev_centroids = np.zeros((0,2))
        self.sticky_widths = np.zeros((0,))
        self.max_distance = max_distance
        self.width_threshold = width_threshold

    def update(self, current_centroids, measured_widths):
        updated_widths = []
        if len(self.prev_centroids) == 0:
            # First frame: use measured widths directly
            updated_widths = list(measured_widths)
            self.prev_centroids = current_centroids
            self.sticky_widths = np.array(updated_widths)
            return np.array(updated_widths)
        for idx, c in enumerate(current_centroids):
            # Find nearest previous centroid
            dists = np.linalg.norm(self.prev_centroids - c, axis=1)
            if len(dists) == 0:
                updated_widths.append(measured_widths[idx])
                continue
            min_idx = np.argmin(dists)
            if dists[min_idx] < self.max_distance:
                last_width = self.sticky_widths[min_idx]
                if abs(measured_widths[idx] - last_width) >= self.width_threshold:
                    updated_widths.append(measured_widths[idx])
                else:
                    updated_widths.append(last_width)
            else:
                updated_widths.append(measured_widths[idx])
        self.prev_centroids = current_centroids
        self.sticky_widths = np.array(updated_widths)
        return np.array(updated_widths)



class rtmaps_python(BaseComponent):
    """
    RTMaps Python bridge for object memory with sticky width.
    Inputs:
      clusters_in: [x0, y0, x1, y1, ...] (centroids)
      bounds_in:   [pt1_x0, pt1_y0, pt2_x0, pt2_y0, width0, pt1_x1, ...] (current measured endpoints and width)
    Outputs:
      clusters_out: same as input (centroids)
      bounds_out:   [pt1_x, pt1_y, pt2_x, pt2_y, width, ...] using sticky width
    """
    def __init__(self):
        BaseComponent.__init__(self)
        self._max_distance = 300
        self._width_threshold = 100

    def Dynamic(self):
        self.add_input("clusters_in", rtmaps.types.FLOAT64)
        self.add_input("bounds_in", rtmaps.types.FLOAT64)  # [pt1_x, pt1_y, pt2_x, pt2_y, width, ...]
        self.add_output("clusters_out", rtmaps.types.FLOAT64, 128)
        self.add_output("bounds_out", rtmaps.types.FLOAT64, 320)

    def Birth(self):
        self.memory = StickyWidthMemory(
            max_distance=self._max_distance,
            width_threshold=self._width_threshold
        )

    def safe_get_input(self, name, count_per_item):
        """
        Safely retrieves and reshapes an input array from RTMaps.
        Returns (N, count_per_item) shape, or zeros if input missing/invalid.
        """
        if name not in self.inputs:
            return np.zeros((0, count_per_item))
        ioelt = self.inputs[name].ioelt
        if ioelt is None or not hasattr(ioelt, "data") or ioelt.data is None:
            return np.zeros((0, count_per_item))
        arr = np.array(ioelt.data, dtype=np.float64)
        if arr.size == 0 or arr.size % count_per_item != 0:
            return np.zeros((0, count_per_item))
        return arr.reshape(-1, count_per_item)

    def Core(self):
        # --- Get current centroids and widths ---
        centroids = self.safe_get_input("clusters_in", 2)
        bounds_arr = np.array(self.inputs["bounds_in"].ioelt.data, dtype=np.float64) \
            if "bounds_in" in self.inputs and hasattr(self.inputs["bounds_in"].ioelt, "data") \
               and self.inputs["bounds_in"].ioelt.data is not None else np.zeros((0,))
        N = centroids.shape[0]
        # bounds_arr: [pt1_x, pt1_y, pt2_x, pt2_y, width, ...]
        if bounds_arr.size != N*5:
            self.outputs["clusters_out"].write(np.zeros((0,), dtype=np.float64))
            self.outputs["bounds_out"].write(np.zeros((0,), dtype=np.float64))
            return
        bounds = bounds_arr.reshape(-1, 5)
        measured_widths = bounds[:, 4]

        # --- Compute sticky widths ---
        sticky_widths = self.memory.update(centroids, measured_widths)

        # --- Use current bounds with sticky widths ---
        out_bounds = []
        for i in range(N):
            width = sticky_widths[i]
            # Use current bounds endpoints
            pt1 = bounds[i, :2]
            pt2 = bounds[i, 2:4]
            out_bounds.append([pt1[0], pt1[1], pt2[0], pt2[1], width])

        # --- Output as flat arrays ---
        self.outputs["clusters_out"].write(centroids.flatten())
        self.outputs["bounds_out"].write(np.array(out_bounds, dtype=np.float64).flatten())

    def Death(self):
        pass
