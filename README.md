# 👁️ Chemist Eye  
### A Vision-Language Model Powered Safety Monitoring System for Self-Driving Laboratories

🚨 **Chemist Eye** is a distributed safety monitoring framework designed to enhance situational awareness in autonomous laboratories by combining multimodal perception, robot control, and vision-language reasoning.

The system detects safety hazards such as:

- Missing Personal Protective Equipment (PPE)
- Worker accidents or medical emergencies
- Fire risks
- Unsafe robot proximity to humans

It then **reasons about the environment** using Vision Language Models (VLMs) and dynamically commands mobile robots to reposition, restrict movement, and notify personnel in real time.

Developed and validated in the **Autonomous Chemistry Laboratory (ACL)** at the University of Liverpool. 

Demo:

https://www.youtube.com/watch?v=Ruk3M4lfYaw

<p align="center">
  <a href="https://www.youtube.com/watch?v=Ruk3M4lfYaw">
    <img src="http://img.youtube.com/vi/Ruk3M4lfYaw/0.jpg" alt="Chemist Eye Demo Video">
  </a>
</p>

---

## 📄 Paper

**Chemist Eye: A Visual Language Model-Powered System for Safety Monitoring and Robot Decision-Making in Self-Driving Laboratories**

<details>
  <summary><b>Paper</b></summary>

  Munguia-Galeano, Francisco and Zhou, Zhengxue and Veeramani, Satheeshkumar and Fakhruldeen, Hatem and Longley, Louis and Clowes, Rob and Cooper, Andrew I  
  [**Chemist Eye: A Visual Language Model-Powered System for Safety Monitoring and Robot Decision-Making in Self-Driving Laboratories**](https://arxiv.org/abs/2508.05148).  
  *arXiv preprint*, 2025. https://arxiv.org/abs/2508.05148 
  *(This content is a preprint and has not been peer-reviewed.)*

  ```bibtex
@article{munguia2025chemist,
  title={Chemist Eye: A Visual Language Model-Powered System for Safety Monitoring and Robot Decision-Making in Self-Driving Laboratories},
  author={Munguia-Galeano, Francisco and Zhou, Zhengxue and Veeramani, Satheeshkumar and Fakhruldeen, Hatem and Longley, Louis and Clowes, Rob and Cooper, Andrew I},
  journal={arXiv preprint arXiv:2508.05148},
  year={2025}
}
```
</details>

---

## 🧠 Key Capabilities

✅ PPE compliance monitoring  
✅ Accident detection via posture recognition  
✅ Fire detection using thermal sensing  
✅ Context-aware robot decision making  
✅ Slack / messaging integration  
✅ Real-time ROS visualization  
✅ Zero-shot reasoning with Vision Language Models  

---

## 🧱 System Architecture

Chemist Eye integrates heterogeneous sensing stations into a ROS-based robotic ecosystem.

**Monitoring Stations**
- RGB-D cameras (Intel RealSense)
- Infrared thermal cameras
- Edge compute devices (Jetson Orin Nano / Raspberry Pi)

**AI Stack**
- YOLO → human detection + localization  
- Vision Language Models (LLaVA-7B, LLaVA-Phi3) → safety reasoning  

**Robotics**
- ROS navigation
- Dynamic path rerouting
- Multi-robot coordination

**Notifications**
- Audible alerts
- Slack messaging
- Live RViz safety map

---

## 🔬 Example Safety Pipeline

1️⃣ Detect hazard (e.g., worker without lab coat)  
2️⃣ Query VLM with contextual map  
3️⃣ Generate safe robot positions  
4️⃣ Freeze or reroute robots  
5️⃣ Notify lab personnel  

---

## 📊 Performance Highlights

| Task | Accuracy |
|--------|------------|
| PPE Detection | **97.5%** |
| Accident Detection | **97%** |
| Robot Decision Success | **~95%** |

Achieved **without task-specific model training** using structured prompting.

---

## ⚠️ Research Disclaimer

Chemist Eye is a **research prototype** and should **NOT** replace institutional safety systems or human supervision.

The system is intended to augment laboratory awareness — not automate safety-critical decisions.

---

## 🖥️ Installation

### Requirements

- ROS (Noetic recommended)
- Python 3.8+
- CUDA-enabled GPU (recommended for VLMs)
- Intel RealSense SDK
- Jetson / Linux workstation

Clone:

```bash
git clone https://github.com/cooper-group-uol-robotics/RobInHoodPy
cd RobInHoodPy
catkin_make
```

## ▶️ Running Chemist Eye

Chemist Eye is a distributed system composed of multiple sensing stations connected to a ROS master. Each station hosts a lightweight server that streams camera data and executes remote instructions via SSH.

### 🖥️ Monitoring Stations

The system relies on two types of edge devices (Raspberry Pi as described in the paper):

---

### RGB-Depth Stations

Repository:

👉 [https://github.com/FranciscoMunguiaGaleano/chemist_eye_client](https://github.com/FranciscoMunguiaGaleano/chemist_eye_client)

These stations:

* Stream RGB-D data to the ROS master
* Execute robot-control instructions via SSH-triggered bash scripts
* Act as the primary perception layer for human detection and PPE monitoring

Start the server on each RGB-Depth station before launching the ROS system.

---

### Infrared (IR) Stations

Repository:

👉 [https://github.com/FranciscoMunguiaGaleano/chemist_eye_ir](https://github.com/FranciscoMunguiaGaleano/chemist_eye_ir)

These stations provide thermal monitoring for fire-risk detection and elevated temperature events.

Launch the IR server on each device prior to running the main system.

---

## 🚀 Launching the Full System

Once all monitoring stations are active, launch the ROS nodes:

```bash
roslaunch chemist_eye chemist_eye.launch
roslaunch chemist_eye chemist_eye_ir.launch
```

This will initialize:

* Vision pipelines
* Vision-Language reasoning
* Robot coordination
* Safety notifications
* RViz visualization

---

## 🧪 Reproducing Case Studies

The simulation experiments presented in the paper can be reproduced using the following launch files:

| Case Study                  | Launch File               |
| --------------------------- | ------------------------- |
| PPE Detection               | `labcoat_exp.launch`      |
| Prone Worker Detection      | `prone_exp.launch`        |
| PPE + Robot Navigation      | `labcoat_nav_exp.launch`  |
| Accident + Robot Response   | `accident_nav_exp.launch` |
| Fire Detection + Navigation | `fire_nav_exp.launch`     |

Example:

```bash
roslaunch chemist_eye labcoat_exp.launch
```

---

The Bag files can be found at: 

👉 [https://github.com/FranciscoMunguiaGaleano/vlm_chemist_eye_benchmark](https://github.com/FranciscoMunguiaGaleano/vlm_chemist_eye_benchmark)

## 📊 Detection Benchmark

For benchmarking the Vision Language Model detection performance:

👉 [https://github.com/FranciscoMunguiaGaleano/vlm_chemist_eye_benchmark](https://github.com/FranciscoMunguiaGaleano/vlm_chemist_eye_benchmark)

This repository contains the scripts used to evaluate hazard detection accuracy across the experimental scenarios described in the paper.

The dataset can be found at: 

👉 [https://doi.org/10.5281/zenodo.18482133](https://doi.org/10.5281/zenodo.18482133)

---

## ⚙️ System Notes

* Ensure SSH connectivity between the ROS master and all monitoring stations.
* Verify camera permissions before launching the servers.
* A CUDA-capable GPU is strongly recommended for real-time Vision-Language inference.

Because Chemist Eye operates as a distributed architecture, **all sensing servers must be active before starting ROS launch files.**

