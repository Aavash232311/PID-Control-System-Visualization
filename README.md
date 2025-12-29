# PID Control System Visualization

## About
This project was inspired by an issue I faced while using an Xbox controller to fly a **Cessna 172** in a flight simulator and by observing how an **Airbus sidestick** controls aircraft pitch. To better understand control systems, I implemented a **simple PID (Proportional–Integral–Derivative) controller** in C.

This is **not a fly-by-wire system**—it is a basic PID control implementation created purely for learning purposes. The goal was to understand how feedback control works and how tuning PID parameters affects system behavior.

After taking a **Data Science course**, I chose **R** to visualize and analyze the controller output by plotting data exported as CSV files.

---

## Features
- PID controller implemented from scratch in **C**
- Real-time simulation of control response
- Outputs control data to a **CSV file**
- Visualization and analysis using **R**
- Focus on understanding PID tuning and system stability

---

## Technologies Used
- **C** — PID controller implementation
- **R** — Data visualization and plotting
- **CSV** — Data exchange format

---

## How It Works
1. The PID controller calculates the control output based on:
   - **Proportional error**
   - **Integral of error**
   - **Derivative of error**
2. The system state and control values are logged into a CSV file.
3. The CSV data is imported into R for visualization.
4. R plots are used to analyze system response, overshoot, and stability.

---
