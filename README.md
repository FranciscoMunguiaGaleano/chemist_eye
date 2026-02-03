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

---

## 📄 Paper

**Chemist Eye: A Visual Language Model-Powered System for Safety Monitoring and Robot Decision-Making in Self-Driving Laboratories**

*(Link will appear here once published)*

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
