<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles: None (new principles added)
Added sections: Core Principles (I-VI), Standards, Docusaurus, Example Code Quality, Code Structure & Context7 MCP Usage, Deploy
Removed sections: None
Templates requiring updates: ⚠ pending for .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# Physical AI and Humanoid Robotics Constitution

## Core Principles

### I. Glossary Standardization
A simple glossary.md should be enforced and created to keep all references and terminologies consistent across all documentation. All technical terms must be defined in this centralized resource to ensure clarity and eliminate ambiguity in communication.

### II. Mathematical Validation
All mathematical questions, equations and principles MUST be validated through rigorous proof or simulation before publication. Mathematical derivations should be checked by at least two experts and supported by computational verification where possible.

### III. Functional Code Examples
Code examples MUST be tested and functional (no pseudocode unless explicitly marked). Every example should be executable in the target environment, with clear setup instructions and verification steps to ensure reproducibility.

### IV. Citation Requirement for Research
Citations REQUIRED for research findings, algorithms, and external concepts using standard academic format (APA style). Any concept borrowed from external sources must be properly attributed with complete bibliographic information.

### V. Verification of Technical Claims
Technical claims require either: (a) citation to authoritative source, (b) derivation/proof, or (c) experimental validation. No assertion about technology, methodology, or capability should be made without supporting evidence.

### VI. Hardware and Performance Claims
No speculative or unverified claims about hardware capabilities, safety limits, or performance. All hardware specifications, performance benchmarks, and safety parameters must be measured and validated through repeatable experiments.

### VII. Logical Content Progression
Content MUST progress logically from fundamentals to advanced topics with clear learning pathways. The material should follow a structured approach that builds upon previously established concepts without skipping essential foundational knowledge.

### VIII. Prerequisite Declaration
Each chapter/section MUST declare explicit prerequisites (prior chapters or external knowledge). Learners should be able to determine what they need to understand before engaging with a particular module or lesson.

### IX. Measurable Learning Objectives
Learning objectives MUST be measurable and stated at chapter start. Objectives should follow the SMART criteria (Specific, Measurable, Achievable, Relevant, Time-bound) to ensure learners can assess their progress.

### X. Modular Curriculum Structure
There are 4 Modules/Parts in the book: The Robotic Nervous System (ROS 2), The Digital Twin (Gazebo & Unity), The AI-Robot Brain (NVIDIA Isaac™), and Vision-Language-Action (VLA). Each module serves a distinct function in the robotics curriculum.

### XI. Standardized Module Format
Each Module should follow this pattern: 1. Module Introduction, 2. Chapters list. This ensures consistent navigation and expectation setting for learners across all modules.

### XII. Standardized Chapter Format
Each Chapter should have this pattern: 1. Learning Objectives, 2. Prerequisites, 3. Content (Core Concepts → Implementation → Examples), 4. Summary, 5. Exercises [At least 3 with logical, conceptual, and implementation scenarios]. This maintains consistency and pedagogical effectiveness throughout all content.

## Standards

### XIII. Visual Communication Requirement
Diagrams REQUIRED for spatial concepts, system architectures, and multi-step processes. Complex ideas must be supported by visual representations to enhance understanding for readers with varied learning preferences.

### XIV. Mathematical Notation Consistency
Notation: Mathematical symbols defined in docs/notation.md and used consistently. All symbols must be defined in a single reference document to prevent confusion and maintain mathematical rigor.

### XV. Traceability Requirement
All facts must be traceable to sources. Claims about scientific principles, technological capabilities, or historical developments must include direct links or references to their origin.

### XVI. Zero-Tolerance Plagiarism Policy
0% tolerance on Plagiarism. All content must be original work or properly cited derivative work with explicit attribution to original sources.

### XVII. Source Verification Protocol
All claims verified against sources. Assertions must be confirmed to match the referenced material, avoiding misinterpretation or selective quotation that distorts original meaning.

## Docusaurus

### XVIII. Navigation and Searchability
Whole book MUST be Navigable and Searchable. The documentation system should provide both hierarchical navigation and full-text search capabilities for optimal user experience.

### XIX. Metadata Completeness
Metadata REQUIRED: title, description, keywords, sidebar_position in every .md frontmatter. Each document must be properly catalogued for system integration and search optimization.

### XX. Search Engine Optimization
Search Optimization: Keywords in Heading, metadata, first paragraph. Content structure should facilitate discovery by both internal documentation search and external search engines.

## Example Code Quality

### XXI. Complete Code Examples
Example code should be complete related to the situation (like not whole code but at least fulfill the required logic). Examples should be sufficient for readers to understand and implement the concept without requiring extensive additional resources.

### XXII. Explanatory Code Comments
Code should include Comments explaining WHY and WHAT this code does. Comments should clarify the intent behind implementation choices, not just restate the obvious functionality.

### XXIII. Dependency Specification
Dependencies and packages should be properly mentioned along with their version. All required software, libraries, and tools must be listed with specific version numbers to ensure reproducible environments.

### XXIV. Industry Standard Libraries
Libraries should be those which are standard and top used in the world and widely adopted. Only well-established, actively maintained libraries with strong community support should be recommended for use.

## Code Structure & Context7 MCP Usage

- Use context7 MCP for every code and setup-related task
- Context7 MCP is installed and SHOULD be used for installing any packages
- Docusaurus should fully utilize CONTEXT7 MCP documentation

### XXV. Use Context7 for All Tasks
**use context7** MCP for every code and setup related task. Agents must rely on official documentation and tools obtained through Context7 MCP rather than internal knowledge or assumptions.

### XXVI. Docusaurus Configuration via Context7
**use context7** mcp is installed and MUST be used for all Docusaurus setup, configuration, and deployment. The official Docusaurus guidelines should be retrieved through Context7 MCP tools.

### XXVII. Documentation Retrieval Protocol
**use context7** to retrieve official Docusaurus documentation instead of relying on internal knowledge. All configuration and setup instructions must come from verified sources accessed through Context7.

### XXVIII. Package Management via Context7
**use context7** for installing any packages and retrieving setup instructions. All installation and configuration procedures must follow official documentation accessed through Context7 MCP.

### XXIX. Configuration via Official Sources
**use context7** for docusaurus.config.js configuration, sidebars.js structure, SEO setup, and deployment commands. Configuration files must be created following official Docusaurus guidelines.

### XXX. External Verification Requirement
All infrastructure and setup tasks require external verification through **use context7** tools. No assumptions about configuration or installation procedures should be made without official verification.

## Deploy

### XXXI. Pre-deployment Build Validation
Book should be Build successfully before Published. A complete build process must succeed without errors or warnings before any deployment occurs.

### XXXII. GitHub Pages Deployment Standard
Book will be deployed on Github Pages. The deployment process must utilize GitHub Pages hosting with all content optimized for static site delivery.

### XXXIII. Static Site Requirements
Should be a static site since deployment is on Github Pages. All resources must be compatible with static site hosting without requiring server-side processing.

### XXXIV. Self-Contained Resource Policy
All resource & content should be local so the book remain serverless and static due to GitHub Pages. External dependencies that might break links should be avoided or mirrored locally.

### XXXV. SEO Compliance
SEO Should be properly done: Sitemap generated automatically and properly (**use context7** for sitemap configuration), robots.txt configured properly (**use context7** for robots.txt setup). Search engine optimization must follow best practices accessed through Context7 MCP.

### XXXVI. Build Gate Compliance
Build Gates (Must pass before deploy/merge): Docusaurus should be build successfully Without any ERRORS and WARNINGS (**use context7** for build commands), Broken links should be check (every link should pass either internal or external), Spell should be Strictly check (Spell Check Pass). All quality gates must pass before deployment or merging of content.

## Governance

All development and documentation activities must comply with these constitutional principles. Changes to this constitution require formal amendment through documented approval process with explicit justification. All contributors must acknowledge and follow these principles during development activities.

**Version**: 1.1.0 | **Ratified**: 2025-01-17 | **Last Amended**: 2025-01-17