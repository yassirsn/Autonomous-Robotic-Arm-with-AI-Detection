# Autonomous Robotic Arm with AI Detection

A complete robotics project featuring an intelligent robotic arm with real-time object detection and remote web control capabilities.

## 🎯 Project Overview

This project implements an autonomous robotic arm system that combines mechanical control, computer vision, and web-based interface for intelligent object manipulation. The system can detect objects in real-time using AI and execute precise movements through a user-friendly web interface.

## ✨ Features

- **Real-time Object Detection**: YOLOv5-based AI model for object recognition by category or color
- **Remote Web Control**: Intuitive web interface with live video streaming
- **Precise Motor Control**: 4-axis movement with calibrated servo positioning
- **Modular Architecture**: Distributed system using Raspberry Pi and ESP32
- **Live Video Feedback**: Real-time camera stream with detection overlay

## 🔧 Hardware Components

- **Raspberry Pi 4**: Main processing unit for AI computation and web interface
- **ESP32**: Microcontroller for servo motor control
- **PCA9685**: 16-channel PWM driver for precise servo control
- **4x SG995 Servo Motors**: Base, shoulder, elbow, and gripper control
- **XY3606 Power Module**: Stable 5V/5A power supply (12V to 5V conversion)
- **Camera Module**: For real-time object detection

## 🏗️ System Architecture

```
Raspberry Pi → Camera Capture → AI Processing (YOLOv5) → Flask Web Interface
     ↓
ESP32 → I2C Communication → PCA9685 → PWM Control → 4 Servo Motors
```

## 🖥️ Software Stack

- **Backend**: Python Flask API for web interface
- **AI/ML**: YOLOv5 for object detection
- **Microcontroller**: ESP32 firmware for motor control
- **Communication**: I2C protocol between ESP32 and PCA9685
- **Frontend**: Web-based control interface with real-time video stream

## 🚀 Key Capabilities

- Automated object manipulation based on visual detection
- Remote operation through web browser
- Real-time video streaming with detection overlays
- Precise 4-axis robotic arm control
- Modular and expandable architecture

## 🔮 Future Enhancements

- Voice recognition control
- 3D printed custom arm components
- Position feedback sensors
- Gesture and mobile app control
- Enhanced AI models for better object recognition

**Technologies**: Python • Flask • YOLOv5 • ESP32 • Raspberry Pi • Computer Vision • Robotics • IoT
