# wcr_diplomski

**Robotics Development with NVIDIA Isaac Sim, Isaac Lab, and ROS 2 Humble**

This project leverages NVIDIA Isaac Sim and Isaac Lab for robotics simulation and development in the context of the 4WIS4WID WCR diploma project.

---

## 📋 Project Overview

This repository contains code, configurations, and simulation assets for integrating Isaac Sim, Isaac Lab, and ROS 2 Humble for advanced robotics development and testing (4WIS4WID mobile robot).

---

## ⚙️ Prerequisites

Before you begin, make sure the following components are installed on your system:

### 1. Python 3.12.x

Download and install Python 3.12.x from the official Python website:

🔗 [https://www.python.org/downloads/](https://www.python.org/downloads/)

During installation:

- Make sure to check **"Add Python to PATH"**.
- Verify installation by running:

`python --version`


### **2. ROS 2 Humble**

Follow the official ROS 2 Humble installation guide:

[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

#### Quick install for Ubuntu 22.04:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop

#### After installation, set up the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash


### **3. NVIDIA Isaac Sim & Isaac Lab**

Download and install Isaac Sim and Isaac Lab from the NVIDIA Omniverse website:

[https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)

#### Installation Steps:
1. Sign in to the NVIDIA Developer portal and download the installer for Isaac Sim.
2. Follow the official installation instructions provided on the download page.
3. To install Isaac Lab, follow the instructions in the Isaac Sim documentation or download it from the same page.

> **Note:** Ensure your system meets the necessary hardware and software requirements listed on the NVIDIA website.

Copy `wcr.py` and paste it into the following directory: IsaacLab\source\isaaclab_assets\isaaclab_assets\robots

Copy the folder `wcr` and paste it into the following directory: 
IsaacLab\source\isaaclab_tasks\isaaclab_tasks\direct

### Train & Play (Linux commands)

#### Training:

To see training with a small number of environments:
```bash
python3 scripts/reinforcement_learning/skrl/train.py --task Isaac-Wcr-Direct-v0 --num_envs 32

To accelerate training with more environments and headless mode (no graphical interface):

```bash
python3 scripts/reinforcement_learning/skrl/train.py --task Isaac-Wcr-Direct-v0 --num_envs 4096 --headless

#### Playing:

To run the best policy learned to this point (requires at least some training to generate a `.pth` file):

```bash
python3 scripts/reinforcement_learning/skrl/play.py --task Isaac-Leatherback-Direct-v0 --num_envs 32
