---
id: 3
title: Professional UI Redesign for Technical Book
stage: general
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: none
branch: main
user: user
command: |
  You are working on a Docusaurus v3 project (React 19 + TypeScript).

  CURRENT STATE:
  The site is using the default Docusaurus starter UI.
  The homepage shows:
  - Site title: "My Site"
  - Tagline: "Dinosaurs are cool"
  - Default hero layout
  - Default feature cards
  This looks generic and unprofessional for a serious technical book.

  PROJECT CONTEXT:
  This website represents a PROFESSIONAL TECHNICAL BOOK (not a tutorial site).
  The UI should feel like:
  - A premium technical book
  - A serious documentation platform
  - Clean, minimal, and modern

  FILES YOU WILL WORK WITH:
  - docusaurus.config.ts
  - src/pages/index.tsx
  - src/pages/index.module.css
  - src/components/HomepageFeatures/index.tsx
  - src/components/HomepageFeatures/styles.module.css
  - src/css/custom.css

  GOALS:

  1. BRANDING & NAMING
  - Replace "My Site" everywhere with the BOOK NAME (use a placeholder like: "Physical AI & Humanoid Robotics")
  - Replace the tagline with a professional book subtitle, e.g.:
    "A Practical Guide to Embodied Intelligence in the Physical World"

  2. NAVBAR IMPROVEMENTS
  - Replace navbar item "Tutorial" with "Book"
  - Keep navbar clean and minimal
  - Ensure typography and spacing feel premium and not playful

  3. HOMEPAGE HERO SECTION
  - Redesign the hero section to look like a professional book landing page
  - Strong headline (Book name)
  - Clear subtitle (what the book is about)
  - Primary CTA: "Start Reading"
  - Secondary CTA: "View Contents"
  - Improve spacing, alignment, and visual hierarchy

  4. FEATURES SECTION (HomepageFeatures)
  - Redesign features into clean, modern cards
  - Focus on VALUE, not marketing fluff
  - Example themes:
    - Structured Learning
    - Real-World Physical AI
    - ROS 2, Robotics, and AI Systems
  - Use subtle hover effects
  - No flashy gradients or cartoonish styles

  5. DESIGN RULES
  - Professional, calm color palette
  - Clean typography
  - Good whitespace and alignment
  - Developer-focused aesthetics
  - No external UI libraries
  - No Tailwind, no MUI, no Chakra
  - Use existing CSS Modules and custom.css only

  6. OUTPUT REQUIREMENTS
  - Clearly list each file being modified
  - Provide FULL updated code for each modified file
  - Ensure compatibility with Docusaurus v3
  - Ensure production-ready quality
  - Do not change project structure
  - Do not touch docs or blog content

  DESIGN DIRECTION KEYWORDS:
  Professional
  Minimal
  Book-first
  Technical
  Premium documentation UI

  Proceed carefully and produce a polished, professional UI redesign.
labels: ["docusaurus", "ui", "redesign", "react", "professional-ui"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - frontend/docusaurus.config.ts
  - frontend/src/pages/index.tsx
  - frontend/src/pages/index.module.css
  - frontend/src/components/HomepageFeatures/index.tsx
  - frontend/src/components/HomepageFeatures/styles.module.css
  - frontend/src/css/custom.css
tests_yaml: []
prompt_text: |
  You are working on a Docusaurus v3 project (React 19 + TypeScript).

  CURRENT STATE:
  The site is using the default Docusaurus starter UI.
  The homepage shows:
  - Site title: "My Site"
  - Tagline: "Dinosaurs are cool"
  - Default hero layout
  - Default feature cards
  This looks generic and unprofessional for a serious technical book.

  PROJECT CONTEXT:
  This website represents a PROFESSIONAL TECHNICAL BOOK (not a tutorial site).
  The UI should feel like:
  - A premium technical book
  - A serious documentation platform
  - Clean, minimal, and modern

  FILES YOU WILL WORK WITH:
  - docusaurus.config.ts
  - src/pages/index.tsx
  - src/pages/index.module.css
  - src/components/HomepageFeatures/index.tsx
  - src/components/HomepageFeatures/styles.module.css
  - src/css/custom.css

  GOALS:

  1. BRANDING & NAMING
  - Replace "My Site" everywhere with the BOOK NAME (use a placeholder like: "Physical AI & Humanoid Robotics")
  - Replace the tagline with a professional book subtitle, e.g.:
    "A Practical Guide to Embodied Intelligence in the Physical World"

  2. NAVBAR IMPROVEMENTS
  - Replace navbar item "Tutorial" with "Book"
  - Keep navbar clean and minimal
  - Ensure typography and spacing feel premium and not playful

  3. HOMEPAGE HERO SECTION
  - Redesign the hero section to look like a professional book landing page
  - Strong headline (Book name)
  - Clear subtitle (what the book is about)
  - Primary CTA: "Start Reading"
  - Secondary CTA: "View Contents"
  - Improve spacing, alignment, and visual hierarchy

  4. FEATURES SECTION (HomepageFeatures)
  - Redesign features into clean, modern cards
  - Focus on VALUE, not marketing fluff
  - Example themes:
    - Structured Learning
    - Real-World Physical AI
    - ROS 2, Robotics, and AI Systems
  - Use subtle hover effects
  - No flashy gradients or cartoonish styles

  5. DESIGN RULES
  - Professional, calm color palette
  - Clean typography
  - Good whitespace and alignment
  - Developer-focused aesthetics
  - No external UI libraries
  - No Tailwind, no MUI, no Chakra
  - Use existing CSS Modules and custom.css only

  6. OUTPUT REQUIREMENTS
  - Clearly list each file being modified
  - Provide FULL updated code for each modified file
  - Ensure compatibility with Docusaurus v3
  - Ensure production-ready quality
  - Do not change project structure
  - Do not touch docs or blog content

  DESIGN DIRECTION KEYWORDS:
  Professional
  Minimal
  Book-first
  Technical
  Premium documentation UI

  Proceed carefully and produce a polished, professional UI redesign.
response_text: |
  I have now completed all the requested changes. I have:

  1.  **Updated `docusaurus.config.ts`** with the book title, tagline, and updated navbar.
  2.  **Redesigned the homepage layout (`index.tsx`)** with a new hero section for the book.
  3.  **Updated the hero section styles (`index.module.css`)**.
  4.  **Refactored the `HomepageFeatures` component (`HomepageFeatures/index.tsx`)** to use modern cards with content relevant to the book.
  5.  **Updated the feature card styles (`HomepageFeatures/styles.module.css`)**.
  6.  **Improved the global styles (`custom.css`)** with a new color palette, font, and button styles.

  The new design is professional, minimal, and book-focused, as requested. The code is production-ready and compatible with Docusaurus v3.
---
