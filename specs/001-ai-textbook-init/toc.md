# Table of Contents: AI-Native Textbook on Physical AI & Humanoid Robotics

This document outlines the complete table of contents for the textbook, mapping all chapters to the four core modules and the final capstone project.

---

## Part 1: Foundations of Physical AI

*   **Chapter 1: Introduction to Physical AI and Embodied Intelligence**
    *   What is Physical AI?
    *   The Convergence of AI, Robotics, and Simulation
    *   Course Overview and Learning Path

*   **Chapter 2: Setting Up Your Robotics Development Environment**
    *   Introduction to Ubuntu 22.04 and the Linux Shell
    *   Installing Essential Tools: VS Code, Docker, Git
    *   Environment Verification

---

## Part 2: Module 1 - The Robot Nervous System (ROS 2)

*   **Chapter 3: Fundamentals of ROS 2**
    *   Core Concepts: Nodes, Topics, Services, Actions
    *   Understanding the ROS 2 Graph
    *   Lab: Creating a "Hello, World!" ROS 2 Publisher and Subscriber

*   **Chapter 4: Building and Structuring ROS 2 Packages**
    *   Creating a ROS 2 Workspace and Packages
    *   Writing Launch Files for Complex Applications
    *   Lab: Building a Simple Two-Node System

*   **Chapter 5: Advanced ROS 2: Transforms and Data Handling**
    *   Working with TF2 for Coordinate Transforms
    *   Handling Different Message Types
    *   Lab: Visualizing Robot Transforms

---

## Part 3: Module 2 - The Digital Twin (Gazebo & Unity)

*   **Chapter 6: Introduction to Robot Simulation with Gazebo**
    *   The Role of a Digital Twin in Robotics
    *   Gazebo Architecture: Physics, Sensors, and World Files
    *   Lab: Spawning a Simple Shape in an Empty World

*   **Chapter 7: Modeling Robots with URDF**
    *   Understanding the Unified Robot Description Format (URDF)
    *   Creating Links, Joints, and Visuals
    *   Lab: Building a Simple Humanoid Arm URDF

*   **Chapter 8: High-Fidelity Visualization with Unity (Optional Bonus)**
    *   Bridging Gazebo and Unity
    *   Creating Immersive HRI (Human-Robot Interaction) Scenarios
    *   Lab: Visualizing the Humanoid Arm in a Unity Scene

---

## Part 4: Module 3 - The AI Brain (NVIDIA Isaac)

*   **Chapter 9: Getting Started with NVIDIA Isaac Sim**
    *   Introduction to the Omniverse Platform
    *   Generating Synthetic Data for AI Training
    *   Lab: Setting up Isaac Sim and Importing a Robot

*   **Chapter 10: AI-Powered Perception with Isaac ROS**
    *   Leveraging Pre-trained Models for Perception
    *   Visual SLAM (vSLAM) for Robot Localization
    *   Lab: Running an Object Detection Model in Isaac Sim

*   **Chapter 11: From Simulation to Reality (Sim-to-Real)**
    *   The Sim-to-Real Challenge
    *   Deploying Models to an NVIDIA Jetson Edge Device
    *   Lab: Running Inference on a Jetson with a Live Camera Feed

---

## Part 5: Module 4 - Vision-Language-Action (VLA)

*   **Chapter 12: Voice-to-Action: Commanding Robots with Speech**
    *   Using Whisper for Speech-to-Text
    *   Mapping Voice Commands to ROS 2 Actions
    *   Lab: Controlling a Simulated Robot Arm with Voice

*   **Chapter 13: LLM-Powered Cognitive Planning**
    *   Using Large Language Models (LLMs) for Task Decomposition
    *   Generating Sequences of Actions from Natural Language
    *   Lab: "Pick up the red block" - From Language to ROS Actions

*   **Chapter 14: Multimodal Interaction**
    *   Fusing Vision, Language, and Action
    *   Building a Conversational, Interactive Robot
    *   Lab: A Simple VLA-powered "I see a block, what should I do?" scenario

---

## Part 6: Capstone Project

*   **Chapter 15: Capstone Project: Building a Simulated Conversational Humanoid Robot**
    *   System Architecture: Integrating All Modules
    *   Step-by-Step Implementation Guide
    *   Final Demonstration: A Robot that Understands, Sees, and Acts
