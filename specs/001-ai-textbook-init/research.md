# Research Findings: AI-Native Textbook on Physical AI & Humanoid Robotics

**Feature**: AI-Native Textbook on Physical AI & Humanoid Robotics
**Date**: 2025-12-10

## Resolved "NEEDS CLARIFICATION" from Technical Context

### Vector Retrieval Latency
-   **Decision**: Target average vector retrieval latency of < 500ms for 95% of queries.
-   **Rationale**: This provides a reasonable user experience for the RAG chatbot without over-optimizing for the free-tier Qdrant or introducing excessive architectural complexity. It is measurable and achievable within typical cloud service constraints.
-   **Alternatives considered**:
    -   Lower latency (<100ms): Would likely require a paid Qdrant tier, more aggressive caching, or a custom self-hosted vector database, increasing cost and operational complexity.
    -   Higher latency (>1s): Would noticeably degrade the user experience of the RAG chatbot, making it feel sluggish.

## Decisions Needing Documentation

### 1. ROS Distribution
-   **Decision**: ROS Humble (LTS)
-   **Rationale**: Prioritizes stability, long-term support, and a larger, more mature community, which aligns with "Clarity for beginners without oversimplifying" and ensuring "Reproducible setup instructions." Humble's extensive package ecosystem provides a solid foundation for a textbook.
-   **Alternatives considered**: Iron (latest features): Offers newer functionalities but comes with potentially less stability and a smaller community, making it less ideal for a foundational textbook targeting beginners.

### 2. Simulation Priority
-   **Decision**: Gazebo-first
-   **Rationale**: Prioritizes accessibility and an open-source mindset, aligning with "Deployable on low-budget setups" and "Open-source first mindset." Gazebo is widely adopted and accessible for beginners without requiring specialized NVIDIA hardware, making the initial learning curve smoother. NVIDIA Isaac Sim will be introduced later as an advanced alternative for higher realism.
-   **Alternatives considered**: Isaac Sim-first: Offers superior realism and advanced features but demands NVIDIA hardware, which could be a barrier for a significant portion of the target audience.

### 3. Cloud vs On-Prem Default
-   **Decision**: Local workstation (On-Prem Default)
-   **Rationale**: Aligns with the "Deployable on low-budget setups" and "Simulation-first before real hardware" principles. Most beginners will start their learning journey on a local machine. Cloud alternatives will be thoroughly documented and provided as options for resource-intensive tasks or for users preferring cloud environments.
-   **Alternatives considered**: Cloud GPU: Offers better performance for complex simulations but comes with higher recurring costs and potentially less control for beginners.

### 4. Capstone Complexity
-   **Decision**: Full VLA + manipulation
-   **Rationale**: Aligns with "Preparing students for industry-level humanoid robotics development" and the "Final Outcome" of building a simulated conversational humanoid robot with Physical AI principles. This provides a comprehensive learning experience and a strong, impactful demonstration, fully satisfying the "Capstone Project" requirement. Reproducibility will be ensured through meticulous step-by-step instructions and tested code.
-   **Alternatives considered**: Navigation + Vision only: Simpler to implement and troubleshoot, but would offer a less complete and less industry-relevant learning experience, falling short of the project's ambition.

### 5. Personalization Depth
-   **Decision**: Explanations only
-   **Rationale**: Manages complexity for this bonus scope feature. Personalizing explanations based on user background provides significant value to the learner without introducing excessive complexity into the content generation, validation, and delivery pipeline. Personalizing exercises and labs would require a much more intricate system for content adaptation and assessment, increasing development time and potential for errors.
-   **Alternatives considered**: Exercises + Labs as well: Offers potentially higher learner value through adapted practicals, but at a substantial increase in development complexity and maintenance overhead.