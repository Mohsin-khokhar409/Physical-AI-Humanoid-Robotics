$featureDescription = @"
AI-Native Textbook on Physical AI & Humanoid Robotics

Target Audience:
- University students in AI, Robotics, and Mechatronics
- Software engineers moving into robotics
- AI practitioners learning Physical AI
- Technical educators and trainers

Primary Focus:
- Teaching Physical AI and embodied intelligence from zero to capstone level
- Bridging digital AI with real-world robotics using simulation and hardware
- Hands-on learning with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems
- Preparing students for industry-level humanoid robotics development

Success Criteria:
- Covers all four official modules:
  1. ROS 2 (Robot Nervous System)
  2. Gazebo & Unity (Digital Twin)
  3. NVIDIA Isaac (AI-Robot Brain)
  4. Vision-Language-Action (VLA)
- Includes at least:
  - 12–15 instructional chapters
  - 20+ hands-on labs and exercises
  - 1 complete capstone humanoid robotics project
- RAG chatbot can:
  - Answer questions only from book content
  - Answer questions based on selected text
  - Cite the source chapter
- Students can:
  - Build ROS 2 packages independently
  - Run humanoid simulations successfully
  - Deploy inference to a Jetson Edge device
- Book is successfully deployed on GitHub Pages or Vercel
- All code is reproducible on a fresh Ubuntu 22.04 system

Constraints:
- Content Length: 8,000–15,000 words
- Format: Markdown source for Docusaurus
- Language: English (Urdu translation optional as bonus)
- Programming Language: Python (primary), C++ optional
- Operating System: Ubuntu 22.04 LTS
- Simulation First: All projects must be testable without physical robots
- Hardware Dependency: Must provide both on-prem and cloud alternatives
- Timeline: Must be completed before Nov 30, 2025

Evidence & Validation Standards:
- All robotics and AI claims must be technically verifiable
- Vendor documentation (ROS, NVIDIA, Gazebo) allowed as primary references
- No unverified performance claims
- All safety-critical steps must be clearly marked
- Code examples must be tested or dry-run validated

RAG & Platform Constraints:
- Backend must be FastAPI-based
- Vector search must use Qdrant (Cloud Free Tier)
- User data must be stored in Neon Serverless PostgreSQL
- Authentication (if implemented) must use Better-Auth
- Chatbot must not use external knowledge beyond the book

Personalization Constraints (Bonus Scope):
- Personalization must be rule-based or agent-based
- User background questions limited to:
  - Programming level
  - AI knowledge level
  - Hardware availability
- Personalized content must not break core learning flow

Not Building:
- A general-purpose robotics encyclopedia
- A full mathematical robotics derivation textbook
- Commercial production-ready robot firmware
- Military or surveillance robotics systems
- Proprietary or closed-source integrations
- Full humanoid hardware manufacturing guide

Out of Scope:
- Ethical, legal, and societal impacts (can be a future volume)
- Advanced control theory proofs
- Low-level motor driver electronics
- FPGA or custom silicon development
- Medical or surgical robotics

Deliverables:
- Public GitHub repository with:
  - Full Docusaurus source
  - All Markdown chapters
  - Backend chatbot code
- Live deployed book link
- Working embedded RAG chatbot
- 90-second demo video
- Reproducible setup instructions

Final Outcome:
A complete AI-native, hands-on, industry-aligned textbook and learning platform that enables students to go from zero knowledge to building a simulated conversational humanoid robot with Physical AI principles.
"@

& '.specify/scripts/powershell/create-new-feature.ps1' -Json -Number 1 -ShortName 'ai-textbook-init' $featureDescription