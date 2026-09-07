# LidarMapper
# LiDAR Road Mapper & Pothole Detector
## Project Framework & Developer Instructions

### 1. Project Overview
This repository contains the software stack for an autonomous LiDAR-based road mapping and pothole detection system. Using precise 3D point cloud data, this system identifies road surface discrepancies, calculates their depth, and maps them geographically.

**Primary Tools:**
*   **ROS 2 (Robot Operating System):** Core middleware for sensor integration and real-time processing.
*   **Open3D / PCL:** Point cloud processing and geometric algorithms.
*   **Google Jules:** AI coding agent used to autonomously build and iterate on the pipeline.
*   **Python 3:** Primary development language.

---

### 2. Workflow with Google Jules
Since this project is being developed with the assistance of Google Jules (an autonomous AI coding agent), follow this strict iterative workflow:

1.  **Assign Small Tasks:** Do not ask Jules to build the entire pipeline at once. Use the phased framework below to create small, manageable prompts.
2.  **Wait for Pull Request (PR):** Jules will clone the repo in a secure cloud VM, write the code, and submit a PR.
3.  **Review and Test:** Pull the PR branch locally to test against sample LiDAR `.pcd` or `.bag` files.
4.  **Merge & Repeat:** Once approved, merge the PR into the `main` branch and assign the next task.

*Security Note: Never commit API keys, cloud credentials, or passwords to this repository.*

---

### 3. Step-by-Step Implementation Framework

#### Phase 1: Workspace Initialization
*   **Task:** Set up the basic ROS 2 Python package structure (`lidar_mapper`).
*   **Dependencies:** Create a `package.xml` and `setup.py`. Include standard ROS 2 libraries, `sensor_msgs`, `open3d`, and `numpy`.
*   **Goal:** Establish the boilerplate environment so Jules understands the project context.

#### Phase 2: Data Acquisition (Node 1)
*   **Task:** Create a ROS 2 subscriber node that listens to the `sensor_msgs/PointCloud2` topic (e.g., `/lidar_points`).
*   **Processing:** Convert the ROS 2 message format into a standard Open3D point cloud object.
*   **Goal:** Ensure the software can successfully ingest hardware data.

#### Phase 3: Preprocessing & Filtering
*   **Task:** Implement a pass-through (crop) filter and voxel downsampling.
*   **Processing:** 
    *   Crop points above a certain height (e.g., > 1.5m) to remove trees, buildings, and tall vehicles.
    *   Crop points outside the lane boundaries.
    *   Downsample the point cloud to reduce computational load.
*   **Goal:** Clean the data and isolate the region of interest (the road).

#### Phase 4: Ground Segmentation (The Core Math)
*   **Task:** Apply the RANSAC (Random Sample Consensus) algorithm to the filtered point cloud.
*   **Processing:** Mathematically fit a plane to the data points. Points aligning with the plane are classified as "Road Surface" (inliers). Points deviating from the plane are "Obstacles" or "Discrepancies" (outliers).
*   **Goal:** Establish the mathematical baseline of the flat road.

#### Phase 5: Pothole Detection & Clustering
*   **Task:** Analyze the outliers from Phase 4.
*   **Processing:**
    *   Filter outliers that are *below* the established RANSAC ground plane by a specific threshold (e.g., > 3cm depth).
    *   Use clustering algorithms like DBSCAN (Density-Based Spatial Clustering of Applications with Noise) to group these points into individual potholes.
*   **Goal:** Isolate and count specific road defects.

#### Phase 6: Geotagging & Export
*   **Task:** Sync the timestamp of detected potholes with the GNSS/GPS topic stream.
*   **Processing:** Extract the latitude and longitude coordinates for each detected cluster. Export the data to a JSON or CSV format suitable for map visualization.
*   **Goal:** Create actionable mapping data.

---

### 4. Getting Started Checklist
- [ ] Connect repository to Google Jules.
- [ ] Feed Phase 1 prompt to Jules.
- [ ] Acquire sample LiDAR data (a ROS 2 `.db3` bag file) for local testing.
