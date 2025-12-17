---
id: 1
title: Physical AI Humanoid Constitution
stage: constitution
date: 2025-01-17
surface: agent
model: Qwen
feature: none
branch: main
user: User
command: /sp.constitution
labels: [constitution,physical-ai,humanoid,robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - 
---

## Prompt

/sp.constitution # project name: Physical AI and Humanoid Robotics

## Core Principles:

- A simple glossary.md should be enforced and created to keep all references and Terminologies. 
- all mathematical questions, equations and principles MUST be validated
- Code examples MUST be tested and functional (no pseudocode unless explicitly marked)
- Citations REQUIRED for research findings, algorithms, and external concepts using standard academic format
- Technical claims require either: (a) citation, (b) derivation/proof, or (c) experimental validation
- Version specifications REQUIRED for all software dependencies and APIs
- No speculative or unverified claims about hardware capabilities, safety limits, or performance
- Content MUST progress logically from fundamentals to advanced topics with clear learning pathways.
- Each chapter/section MUST declare explicit prerequisites (prior chapters or external knowledge)
- Learning objectives MUST be measurable and stated at chapter start
- There are 4 Modules/Parts in the book [ The Robotic Nervous System (ROS 2), The Digital Twin (Gazebo & Unity), The AI-Robot Brain (NVIDIA Isaac™), Vision-Language-Action (VLA) ]
- Each Module should follow this pattern:
  1. Module Introduction.
  2. Chapters list
- Each Chapters should have this pattern:
  1. Learning Objectives
  2. Prerequisites
  3. Content ( Core Concepts --> Implementation --> Examples)
  4. Summary
  5. Exercises [ Atleast 3 with logical, conceptual, and implementation scenarios ]

## Standards:

- Diagrams REQUIRED for spatial concepts, system architectures, and multi-step processes
- Notation: Mathematical symbols defined in docs/notation.md and used consistently
- All facts must be traceable to sources
- Citation should be APA style
- 0% tolerance on Plagiarism
- All claims verified against sources

## Docusaurus:

- Whole book MUST be Navigable and Searchable
- Metadata REQUIRED: title, description, keywords, sidebar_position in every .md frontmatter
- Search Optimization: Keywords in Heading, metadata, first paragraph

## Example Code Quality:
- Example code should be complete related to the situation ( like not whole code but atleast fulfil the required logic )
- Code should include Comments of explaining WHY and WHAT this code do
- Dependencies and packages should be properly mentioned along with their version.
- Libraries should be those which are standard and top used in the world and widely adopted.

## Code Structure & Context7 MCP Usage:
**CRITICAL: The phrase "use context7" must never be modified, replaced, or rephrased. This exact wording (lowercase) is required for MCP tool invocation.**

- **use context7** MCP for every code and setup related task.
- **use context7** mcp is installed and MUST be used for all Docusaurus setup, configuration, and deployment.
- **use context7** to retrieve official Docusaurus documentation instead of relying on internal knowledge.
- **use context7** for installing any packages and retrieving setup instructions.
- **use context7** for docusaurus.config.js configuration, sidebars.js structure, SEO setup, and deployment commands.
- All infrastructure and setup tasks require external verification through **use context7** tools.
- Agents must prioritize **use context7** MCP tools for all information gathering and task execution related to Docusaurus.

## Deploy:
- Book should be Build successfully before Published
- Book will be deployed on Github Pages
- Should be a static site since deployment is on Github Pages
- All resource & content should be local so the book remain server less and static due to GitHub Pages
- SEO Should be properly done:
  - Sitemap generated automatically and properly (**use context7** for sitemap configuration)
  - robots.txt configured properly (**use context7** for robots.txt setup)
- Build Gates ( Must pass before deploy/merge)
  - Docusaurus should be build successfully Without any ERRORS and WARNINGS (**use context7** for build commands)
  - Broken links should be check ( every link should be pass either internal or external )
  - Spell should be Strictly check ( Spell Check Pass)

## Response snapshot

I've completed the constitution update for the Physical AI and Humanoid Robotics project. Here's my summary:

The constitution has been successfully updated with the following key changes:

- **New Version**: 1.1.0 (incremented from 1.0.0 due to addition of multiple new principles)
- **Added 36 principles** across 6 main categories:
  1. Core Principles (12 principles) - covering glossary standardization, mathematical validation, functional code examples, citations, verification, hardware claims, content progression, prerequisites, learning objectives, curriculum structure, and standardized formats
  2. Standards (5 principles) - including diagrams, notation consistency, traceability, plagiarism policy, and source verification
  3. Docusaurus (3 principles) - navigation, metadata, and SEO requirements
  4. Example Code Quality (4 principles) - code completeness, explanatory comments, dependency specs, and standard libraries
  5. Code Structure & Context7 MCP Usage (6 principles) - focused on using Context7 tools for all development tasks
  6. Deploy (6 principles) - covering build validation, deployment standards, static site requirements, and quality gates

- **Ratification Date**: Set to 2025-01-17 (today's date in ISO format)
- **All placeholder tokens** have been properly replaced with specific requirements
- **Sync Impact Report** has been included at the top of the constitution file

The constitution now fully reflects the detailed requirements you provided for the Physical AI and Humanoid Robotics project, with special emphasis on mathematical validation, citation standards, the use of Context7 MCP tools, and the specific 4-module curriculum structure (The Robotic Nervous System, The Digital Twin, The AI-Robot Brain, and Vision-Language-Action).

The file has been saved to `.specify/memory/constitution.md` and is ready for use in your project.

## Outcome

- ✅ Impact: Constitution updated with 36 detailed principles for Physical AI and Humanoid Robotics project
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with new principles and requirements
- 🔁 Next prompts: Project can now proceed with planning and implementation following new constitution
- 🧠 Reflection: Comprehensive constitution created that addresses all specified requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A