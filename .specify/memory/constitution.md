<!--
Sync Impact Report:
- Version change: null → 1.0.0
- List of modified principles: All 6 principles have been filled with new content.
- Added sections: "Technical & Project Requirements", "Assessment & Quality Standards"
- Removed sections: None, placeholders were filled.
- Templates requiring updates:
    - .specify/templates/plan-template.md (⚠ pending)
    - .specify/templates/spec-template.md (⚠ pending)
    - .specify/templates/tasks-template.md (⚠ pending)
    - .specify/templates/commands/*.md (⚠ pending)
    - README.md, docs/quickstart.md or agent-specific guidance files (⚠ pending)
- Follow-up TODOs: None
-->
# AI-Native Textbook on Physical AI & Humanoid Robotics Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### Technical Accuracy & Verifiability
All technical explanations must be verifiable from trusted sources. Technical accuracy takes precedence over assumptions.
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### Clarity, Practicality & Hands-on Learning
Content must be clear for beginners without oversimplifying. Emphasize practical, hands-on learning with a simulation-first approach before real hardware.
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### Ethical AI & Robotics
Promote ethical and safe use of AI and robotics. Include safety warnings for physical robotics and discuss dataset/simulation bias.
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### AI-Native & Open-Source First
Embrace AI-native learning (content + chatbot + personalization). Maintain an open-source first mindset for all project components.
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### Runnable & Tested Code
Code examples must be runnable and tested. ROS 2 examples must target Ubuntu 22.04 LTS, with Python as the primary language.
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### Comprehensive Chapter Structure
Every chapter must include learning objectives, concept explanation, practical exercise, summary, and assessment/mini-project.

## Technical & Project Requirements
This section outlines the essential technical specifications, RAG chatbot functionalities, personalization/localization rules, and project constraints.
- **Technical Stack**: Book Framework (Docusaurus), Content Generation (Claude Code + Spec-Kit Plus), Chatbot (OpenAI Agents / ChatKit SDK), Backend (FastAPI), Database (Neon Serverless PostgreSQL), Vector Store (Qdrant Cloud - Free Tier), Deployment (GitHub Pages or Vercel).
- **RAG Chatbot**: Must answer only from book content, support Q&A from user-selected text, cite chapters, reject out-of-scope questions, support logged-in user personalization.
- **Personalization & Localization (Bonus)**: User background influences explanations, different depth for beginners vs. advanced users, Urdu translation preserves technical meaning, content consistency across languages.
- **Constraints**: Follow official hackathon course outline, cover all 4 modules in sequence, include complete Capstone Project, deployable on low-budget setups, demo video under 90 seconds, public and open-source.
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## Assessment & Quality Standards
This section defines the assessment criteria, success indicators, quality metrics, and ethical guidelines for the project.
- **Assessment**: ROS 2 package development, Gazebo simulation, Isaac perception pipeline, full humanoid simulation capstone with conversational AI.
- **Success Criteria**: Book deploys successfully, RAG chatbot answers correctly, modules match course outline, code runs without critical errors, zero plagiarism, clear learning progression, judges reproduce capstone, suitable for classroom deployment.
- **Quality Metrics**: Technical accuracy verified, code execution success rate > 95%, readability Flesch-Kincaid grade 10–12, clear theory/practice separation, no broken links.
- **Ethics & Safety**: No unsafe robot control instructions without warnings, no military/weaponized use cases, emphasis on human safety/responsible AI.
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
The constitution serves as the foundational document for project direction and execution. Amendments require thorough review and justification. All technical decisions and implementations must align with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->