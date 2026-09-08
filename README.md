# LidarMapper
# LiDAR Road Mapper & Pothole Detector
## Project Framework & Developer Instructions

### 1. Project Overview
This repository contains the software stack for an autonomous LiDAR-based road mapping and pothole detection system. Using precise 3D point cloud data, this system identifies road surface discrepancies, calculates their depth, and maps them geographically.

**Primary Tools:**
*   **ROS 2 (Robot Operating System):** Core middleware for sensor integration and real-time processing.
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
*   **Dependencies:** Create a `package.xml` and `setup.py`. Include standard ROS 2 libraries, `sensor_msgs`, and message filters.
*   **Goal:** Establish the boilerplate environment so Jules understands the project context.

#### Phase 2: Data Acquisition
*   **Task:** Create a ROS 2 subscriber node that listens to the `sensor_msgs/LaserScan` topic (`/scan`) and `sensor_msgs/NavSatFix` (`/gps/fix`).
*   **Processing:** Synchronize these streams using time.
*   **Goal:** Ensure the software can successfully ingest hardware data.

#### Phase 3: Ground Segmentation & Pothole Detection (The Core Math)
*   **Task:** Analyze the central 90-degree field of view of the 2D scan.
*   **Processing:** Maintain an Exponential Moving Average (EMA) to establish a dynamic ground baseline. Identify potholes as dips > 5cm below the baseline.
*   **Goal:** Detect road defects efficiently without heavy 3D processing.

#### Phase 6: Geotagging & Export
*   **Task:** Sync the timestamp of detected potholes with the GNSS/GPS topic stream.
*   **Processing:** Extract the latitude and longitude coordinates for each detected cluster. Export the data to a JSON or CSV format suitable for map visualization.
*   **Goal:** Create actionable mapping data.

---

### 4. Getting Started Checklist
- [ ] Connect repository to Google Jules.
- [ ] Feed Phase 1 prompt to Jules.
- [ ] Acquire sample LiDAR data (a ROS 2 `.db3` bag file) for local testing.
