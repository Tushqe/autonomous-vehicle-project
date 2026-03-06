# Autonomous Vehicle Project — ROS2 Stack with YOLOPv2 Perception

This repository contains the ROS2 stack I built for a small‑scale autonomous vehicle, combining classical control with a deep learning–based perception front‑end (YOLOPv2) deployed on both x86 and NVIDIA Jetson platforms.

The stack is designed to be easy to bring up on either a laptop (for development) or an embedded system (for deployment), with clear separation between sensor drivers, perception, localization, planning, and control.

---

## Pipeline Overview

**Sensors → Perception → Localization → Planning → Control**

- Sensors: LiDAR, RGB camera, IMU  
- Perception: YOLOPv2 lane and drivable‑area detection, image pre/post‑processing  
- Localization: visual–inertial odometry fused with IMU (EKF)  
- Planning: global path generation and local trajectory updates  
- Control: Pure Pursuit lateral controller and basic longitudinal control

---

## Repository Structure

```text
ros2_ws/
  ├── src/
  │   ├── sensors/           # LiDAR/camera/IMU drivers
  │   ├── perception_yolop/  # YOLOPv2 ROS2 node + preprocessing
  │   ├── localization/      # VIO + EKF fusion
  │   ├── planning/          # Global + local planners
  │   └── control/           # Pure Pursuit and speed control
  ├── launch/
  │   ├── bringup_sim.launch.py
  │   └── bringup_real.launch.py
  └── media/                 # Example input/output images for YOLOPv2
```
---

## YOLOPv2 Perception Results
The perception node runs YOLOPv2 for lane marking and drivable‑area segmentation. Below are qualitative results from both CPU (Mac) and GPU (Jetson) deployments.
1. Baseline CPU Inference (Mac)

- Model: YOLOPv2
- Hardware: Apple laptop CPU
- Runtime: ~1.6 s per 1280×720 image (used for validation and debugging before embedded deployment)

### Input Test Image:

<img width="1280" height="720" alt="11_Color" src="https://github.com/user-attachments/assets/b8cf86b7-558e-4c74-b058-9aa982518cbe" />

### Output Image 1: 

<img width="1280" height="720" alt="img1" src="https://github.com/user-attachments/assets/9ba0e7a6-a76d-48d9-b15f-c57934ff00f9" />

### Output Image 2: 

<img width="1280" height="720" alt="img3" src="https://github.com/user-attachments/assets/df7c5693-824c-4a82-8028-bc55bb13834c" />

These outputs validate the end‑to‑end preprocessing and post‑processing pipeline on a desktop environment before moving to embedded hardware.

2. Jetson Nane Deployment

- Model: YOLOPv2
- Hardware: Jetson Nano (GPU)
- Runtime: 300-400 ms/frame

### Output on Jetson Nano: 

<img width="80" height="80" alt="cmp_da_ts" src="https://github.com/user-attachments/assets/11ca3738-3a51-4145-ad4e-c00af481d891" />

This image shows drivable‑area segmentation produced directly on the embedded platform, demonstrating that the perception stack works within an embedded compute and power budget.

3. Improved GPU Drivable‑Area Mask

Further refinements to the drivable‑area rendering and post‑processing pipeline, including cleaner morphological processing and better thresholding for a smoother, more reliable mask suitable for downstream planning and control.

### Input Image: 

<img width="5712" height="4284" alt="IMG_0219" src="https://github.com/user-attachments/assets/17a76aa5-e08c-4983-9710-bfefc16bab09" />


### Improved drivable-area mask: 

<img width="640" height="640" alt="da_mask_mac" src="https://github.com/user-attachments/assets/a55e6f5c-386d-4dbb-be51-e32341f4788c" />

---

## Performance and Future Work 

- CPU prototype: ~1.6 s/frame on Mac, used for offline testing and data collection
- Embedded inference: reduced latency on Jetson Nano, suitable for low‑speed autonomous navigation

Planned Improvements: 

- TensorRT conversion and INT8/FP16 optimization for YOLOPv2
- Additional timing benchmarks across different resolutions
- Automated bagfile‑based regression tests for perception and control
























