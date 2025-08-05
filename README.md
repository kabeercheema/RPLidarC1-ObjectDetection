# RPLidarC1-ObjectDetection
Real-time object detection and width estimation using RPLidar C1 and clustering algorithms in RTMaps, using C++ and Python.

## Overview
This project implements a **real-time object detection and width estimation system** using the **RPLidar C1 sensor** and **RTMaps**. It processes raw Lidar data to detect objects using **DBSCAN clustering**, calculates object widths, and visualizes results in real-time using **OpenCV**.

The system is designed for the University of Waterloo's Alternative Fuels Team (UWAFT), which competes in the EcoCAR EV Challenge. A robust and fast object detection algorithm was essential, as this LiDAR acts as a supervisory module within the perception stack, validating and verifying objects that the stock sensors on the vehicle detect.

---

## Features
- Real-time Lidar data processing from **RPLidar C1**
- Object detection using **DBSCAN clustering**
- **Width estimation** with hysteresis-based "sticky width" memory for stability
- **OpenCV visualization** with:
  - Blue points: Raw detections
  - Red X: Cluster centroids
  - Green lines: Width bounds
  - Axis ticks and labels for scale reference
- Fully integrated with **RTMaps** for time-synchronized data pipelines

---

## System Architecture

The object detection pipeline consists of the following stages:

1. **Lidar Data Acquisition**
   - RPLidar C1 streams raw angle-distance pairs into RTMaps using C++ to send requests to the LiDAR.

2. **Data Preprocessing**
   - Convert polar coordinates (angle, distance) → Cartesian (x, y).
   - Filter invalid points and apply range limits.

3. **Clustering (Object Detection)**
   - Apply **DBSCAN** to group points into clusters.
   - Compute each cluster's **centroid** (x, y).

4. **Width Estimation**
   - Determine the two farthest points in each cluster.
   - Compute **object width** using Euclidean distance.
   - Apply **sticky width hysteresis** to maintain stable width values over time.

5. **Visualization**
   - Display:
     - **Blue dots:** Raw Lidar points
     - **Red X:** Cluster centroids
     - **Green lines:** Width bounds with labels
   - Rendered using **OpenCV** with axes, ticks, and labels.

## Dependencies
- [RTMaps 4](https://intempora.com/download/RTMaps4/)
- [RPLidar SDK](https://github.com/Slamtec/rplidar_sdk) – used for interfacing with the RPLidar C1
- Python packages:
  - numpy
  - opencv-python
  - scikit-learn



