# Atomic Task Execution Plan: AI-Native Physical AI & Humanoid Robotics Textbook

## Purpose
This file defines all atomic tasks required to complete the AI-native Physical AI & Humanoid Robotics textbook and embedded RAG chatbot using a strict checkpoint-controlled workflow. Each task takes 15–45 minutes, has one acceptance criterion, one output, and clear dependencies.

--------------------------------------------------
## Phase 0: Project Initialization (30–45 minutes)
--------------------------------------------------

### Task 0.1 [x]: Initialize Git Repository & Project Structure
- Duration: 20 minutes
- Depends on: None
- Description: Create public Git repository, initialize Docusaurus project, and add Spec-Kit files.
- Acceptance Criterion:
  "Git repo created, Docusaurus builds successfully, sp files committed."
- Output: Public GitHub repo with initial commit

---

### Task 0.2 [x]: Verify Spec-Kit + Claude Code Integration
- Duration: 15 minutes
- Depends on: Task 0.1
- Description: Confirm Spec-Kit Plus workflow runs inside the project.
- Acceptance Criterion:
  "Spec-Kit commands execute without errors."
- Output: Verified Spec-Kit environment

---

### ✅ CHECKPOINT 0 — Project Bootstrap Review
Human must verify:
- Repo visibility
- Clean Docusaurus build
- Spec-Kit files present  
Approval required before Phase 1.

--------------------------------------------------
## Phase 1: Book Architecture & Outline (60–90 minutes)
--------------------------------------------------

### Task 1.1 [x]: Define Full Table of Contents
- Duration: 30 minutes
- Depends on: Checkpoint 0
- Description: Create chapter-by-chapter outline mapped to course modules.
- Acceptance Criterion:
  "All 4 modules + capstone mapped to chapters with logical flow."
- Output: `toc.md`

---

### Task 1.2 [x]: Define Chapter Template
- Duration: 15 minutes
- Depends on: Task 1.1
- Description: Create reusable structure for every chapter.
- Acceptance Criterion:
  "Template includes objectives, lab, summary, and assessment."
- Output: `chapter-template.md`

---

### Task 1.3 [x]: Define Capstone Project Scope
- Duration: 15 minutes
- Depends on: Task 1.1
- Description: Specify minimum required features for the Autonomous Humanoid capstone.
- Acceptance Criterion:
  "Capstone includes voice, navigation, vision, and manipulation."
- Output: `capstone-scope.md`

---

### ✅ CHECKPOINT 1 — Architecture & Scope Review
Human must verify:
- Coverage of all modules
- Realistic capstone complexity
- No missing learning stages  
Approval required before Phase 2.

--------------------------------------------------
## Phase 2: Module 0–1 Content Generation (Foundations + ROS 2)
--------------------------------------------------

### Task 2.1 [x]: Write Foundations of Physical AI Chapter
- Duration: 45 minutes
- Depends on: Checkpoint 1
- Description: Generate the introduction to Physical AI and embodied intelligence.
- Acceptance Criterion:
  "Concepts clear, no plagiarism, beginner-friendly."
- Output: `module-0-foundations.md`

---

### Task 2.2 [x]: Write ROS 2 Core Concepts Chapter
- Duration: 45 minutes
- Depends on: Task 2.1
- Description: Generate nodes, topics, services, actions explanation.
- Acceptance Criterion:
  "All ROS 2 basics explained with examples."
- Output: `module-1-ros2-core.md`

---

### Task 2.3 [x]: Create ROS 2 Hands-On Lab
- Duration: 30 minutes
- Depends on: Task 2.2
- Description: Create a simple ROS 2 publisher/subscriber Python lab.
- Acceptance Criterion:
  "Lab runs on Ubuntu 22.04 with Humble."
- Output: `lab-ros2-basic.md`

---

### ✅ CHECKPOINT 2 — Foundations & ROS 2 Review
Human must verify:
- Technical correctness
- Lab reproducibility
- Beginner clarity  
Approval required before Phase 3.

--------------------------------------------------
## Phase 3: Module 2 Content (Gazebo & Unity Digital Twin)
--------------------------------------------------

### Task 3.1: Write Gazebo Simulation Chapter
- Duration: 45 minutes
- Depends on: Checkpoint 2
- Description: Explain physics simulation, collisions, and sensors.
- Acceptance Criterion:
  "Gazebo workflow explained step-by-step."
- Output: `module-2-gazebo.md`

---

### Task 3.2: Create URDF Robot Modeling Lab
- Duration: 30 minutes
- Depends on: Task 3.1
- Description: Create a small humanoid URDF modeling exercise.
- Acceptance Criterion:
  "URDF loads correctly in Gazebo."
- Output: `lab-urdf.md`

---

### Task 3.3: Write Unity Visualization Chapter
- Duration: 30 minutes
- Depends on: Task 3.1
- Description: Explain Unity robot visualization for HRI.
- Acceptance Criterion:
  "Clear pipeline from Gazebo to Unity."
- Output: `module-2-unity.md`

---

### ✅ CHECKPOINT 3 — Digital Twin Review
Human must verify:
- Sensor simulation coverage
- URDF validity
- Visualization explanation  
Approval required before Phase 4.

--------------------------------------------------
## Phase 4: Module 3 Content (NVIDIA Isaac AI Brain)
--------------------------------------------------

### Task 4.1: Write Isaac Sim Fundamentals Chapter
- Duration: 45 minutes
- Depends on: Checkpoint 3
- Description: Explain Isaac Sim, synthetic data, Omniverse.
- Acceptance Criterion:
  "Isaac workflow clearly documented."
- Output: `module-3-isaac-sim.md`

---

### Task 4.2: Write Isaac ROS Perception Chapter
- Duration: 45 minutes
- Depends on: Task 4.1
- Description: Explain VSLAM, stereo vision, navigation.
- Acceptance Criterion:
  "Perception pipeline is complete."
- Output: `module-3-isaac-ros.md`

---

### Task 4.3: Create Sim-to-Real Transfer Lab
- Duration: 30 minutes
- Depends on: Task 4.2
- Description: Demonstrate simulation → Jetson inference.
- Acceptance Criterion:
  "Inference runs on Jetson Orin Nano."
- Output: `lab-sim2real.md`

---

### ✅ CHECKPOINT 4 — Isaac Platform Review
Human must verify:
- GPU requirements realistic
- Jetson steps valid
- No unsafe instructions  
Approval required before Phase 5.

--------------------------------------------------
## Phase 5: Module 4 Content (Vision-Language-Action)
--------------------------------------------------

### Task 5.1: Write Voice-to-Action Chapter
- Duration: 30 minutes
- Depends on: Checkpoint 4
- Description: Explain Whisper + ROS 2 voice commands.
- Acceptance Criterion:
  "Voice pipeline logically complete."
- Output: `module-4-voice.md`

---

### Task 5.2: Write LLM Cognitive Planning Chapter
- Duration: 45 minutes
- Depends on: Task 5.1
- Description: Explain natural language → task planning.
- Acceptance Criterion:
  "Examples map language to ROS actions."
- Output: `module-4-planning.md`

---

### Task 5.3: Write Multimodal Interaction Chapter
- Duration: 30 minutes
- Depends on: Task 5.2
- Description: Combine speech + vision + navigation.
- Acceptance Criterion:
  "Full VLA pipeline explained."
- Output: `module-4-vla.md`

---

### ✅ CHECKPOINT 5 — VLA Review
Human must verify:
- LLM usage clarity
- ROS integration logic
- Safety warnings present  
Approval required before Phase 6.

--------------------------------------------------
## Phase 6: Capstone Project Development
--------------------------------------------------

### Task 6.1: Write Capstone Architecture
- Duration: 30 minutes
- Depends on: Checkpoint 5
- Description: Define full Autonomous Humanoid system design.
- Acceptance Criterion:
  "Architecture includes perception, planning, control."
- Output: `capstone-architecture.md`

---

### Task 6.2: Write Step-by-Step Capstone Implementation
- Duration: 45 minutes
- Depends on: Task 6.1
- Description: Full pipeline from voice command to manipulation.
- Acceptance Criterion:
  "Steps are reproducible in simulation."
- Output: `capstone-implementation.md`

---

### ✅ CHECKPOINT 6 — Capstone Review
Human must verify:
- Logical correctness
- Reproducibility
- Alignment with skill level  
Approval required before Phase 7.

--------------------------------------------------
## Phase 7: RAG Chatbot Development
--------------------------------------------------

### Task 7.1: Implement Vector Ingestion Pipeline
- Duration: 30 minutes
- Depends on: Checkpoint 6
- Description: Chunk Markdown → Embed → Store in Qdrant.
- Acceptance Criterion:
  "All chapters indexed successfully."
- Output: `rag-ingest.py`

---

### Task 7.2: Implement RAG Query API
- Duration: 30 minutes
- Depends on: Task 7.1
- Description: FastAPI endpoint for book-only answering.
- Acceptance Criterion:
  "Responses are based only on retrieved chunks."
- Output: `rag-api.py`

---

### Task 7.3: Implement Selected-Text-Only Mode
- Duration: 20 minutes
- Depends on: Task 7.2
- Description: Restrict RAG answering to user-provided text only.
- Acceptance Criterion:
  "Out-of-text questions are rejected."
- Output: Updated RAG endpoint

---

### ✅ CHECKPOINT 7 — RAG Review
Human must verify:
- No hallucinations
- Chapter citations present
- Selected-text restriction works  
Approval required before Phase 8.

--------------------------------------------------
## Phase 8: Deployment & Demo
--------------------------------------------------

### Task 8.1: Deploy Book to GitHub Pages / Vercel
- Duration: 20 minutes
- Depends on: Checkpoint 7
- Description: Deploy static book.
- Acceptance Criterion:
  "Public URL accessible globally."
- Output: Live book URL

---

### Task 8.2: Deploy Backend APIs
- Duration: 20 minutes
- Depends on: Task 8.1
- Description: Deploy FastAPI + Qdrant + Neon.
- Acceptance Criterion:
  "Chatbot responds from production environment."
- Output: Live API endpoint

---

### Task 8.3: Record 90-Second Demo Video
- Duration: 30 minutes
- Depends on: Task 8.2
- Description: Show book, chatbot, and one simulation.
- Acceptance Criterion:
  "Video under 90 seconds, clear walkthrough."
- Output: Demo video link

---

### ✅ CHECKPOINT 8 — Final Submission Review
Human verifies:
- Book quality
- Chatbot accuracy
- Demo clarity  
Final Git commit marks project completion.

--------------------------------------------------
## Dependency Summary

Phase 0:
0.1 → 0.2 → ✅ CHECKPOINT 0

Phase 1:
✅0 → 1.1 → 1.2 → 1.3 → ✅ CHECKPOINT 1

Phase 2:
✅1 → 2.1 → 2.2 → 2.3 → ✅ CHECKPOINT 2

Phase 3:
✅2 → 3.1 → 3.2 → 3.3 → ✅ CHECKPOINT 3

Phase 4:
✅3 → 4.1 → 4.2 → 4.3 → ✅ CHECKPOINT 4

Phase 5:
✅4 → 5.1 → 5.2 → 5.3 → ✅ CHECKPOINT 5

Phase 6:
✅5 → 6.1 → 6.2 → ✅ CHECKPOINT 6

Phase 7:
✅6 → 7.1 → 7.2 → 7.3 → ✅ CHECKPOINT 7

Phase 8:
✅7 → 8.1 → 8.2 → 8.3 → ✅ CHECKPOINT 8

--------------------------------------------------
## Execution Rule
No task may begin without:
1. All dependencies completed
2. Human approval at the previous checkpoint
3. Git commit of approved work