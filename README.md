# Trashcan Project Using Subsumption Architecture

This repository contains the implementation of a reactive robotic system that performs an autonomous trash collection task using **Subsumption Architecture**, a behavior-based control model for robotics. The project was developed in the **Webots** simulation environment and demonstrates the robot's ability to navigate, detect colored blocks, and perform sequential tasks based on sensor inputs and hierarchical behaviors.

## 🚀 Overview

This project is based on the Subsumption Architecture model introduced by Rodney Brooks. The robot operates in a simulated environment and completes the following objectives:

- Wanders around the environment
- Detects and approaches a red block (trash)
- Detects and approaches a green block (trashcan)
- Returns to the blue block (home)

Each behavior is managed through a layered control system that allows lower-level behaviors (like obstacle avoidance) to be overridden by higher-level tasks.

## 🧠 Architecture

The robot's behavior is divided into the following layers:

1. **Wandering & Obstacle Avoidance**
2. **Trash (Red Block) Detection**
3. **Trashcan (Green Block) Detection**
4. **Return Home (Blue Block) Detection**

Each layer is implemented using a **Finite State Machine (FSM)** and color recognition algorithms with HSV conversion to ensure reliability in varied lighting conditions.

## 🛠️ Technologies Used

- **Webots** – simulation environment
- **Python** – for robot control logic
- **HSV Color Detection** – for robust block recognition
- **Finite State Machine (FSM)** – for task control and switching

## 📁 Files Included

- `controller.py` – Main control logic
- `trashcan_project.wbt` – World file for Webots (if included)
- `README.md` – Project description
- `Trashcan_Project_Paper.pdf` – Full technical report detailing the implementation, algorithms, and results

## 📊 Experimental Results

The robot successfully performed the tasks in real time, adapting to its environment using sensor feedback and color-based decisions. Screenshots and full methodology are provided in the PDF report.

## 📄 Project Paper

A full technical paper describing the system design, subsumption architecture, algorithm, and experimental validation can be downloaded here:

📄 [Download PDF Report](./Implementing%20the%20Trashcan%20Project%20using%20Subsumption%20Architecture%20Erla%20Hoxha%20(1).pdf)

## 👤 Author

**Erla Hoxha**    
📧 erlahoxha04@gmail.com

---

