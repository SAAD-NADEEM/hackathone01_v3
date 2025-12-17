# Data Model: Educational Resource Book on Physical AI and Humanoid Robotics

## Entities

### Module
- **Name**: String (required) - e.g., "The Robotic Nervous System (ROS 2)"
- **Description**: String (required) - Overview of the module's content and objectives
- **Weeks**: Number (required) - Duration in weeks (e.g., 5 for Module 1, 2 for Module 2, etc.)
- **Chapters**: Array of Chapter - List of chapters within the module
- **Learning Objectives**: Array of String - High-level objectives for the module
- **Prerequisites**: Array of String - Knowledge required before starting this module
- **Frontmatter**: Object - Required metadata (title, description, keywords, sidebar_position)

### Chapter
- **Title**: String (required) - The chapter title
- **Description**: String (required) - Brief description of chapter content
- **Learning Objectives**: Array of String (required) - Specific, measurable objectives
- **Prerequisites**: Array of String (required) - Knowledge needed before this chapter
- **Content Sections**: Array of Object - Core concepts, implementation, examples
- **Summary**: String (required) - Key takeaways from the chapter
- **Exercises**: Array of Exercise (required, min 3) - Logical, conceptual, implementation
- **Frontmatter**: Object (required) - title, description, keywords, sidebar_position
- **Dependencies**: Array of Object - Software, libraries, and their versions

### Exercise
- **Type**: String (required) - "logical", "conceptual", or "implementation"
- **Description**: String (required) - What the exercise involves
- **Difficulty**: String (required) - "beginner", "intermediate", or "advanced"
- **Solutions**: Array of Object - Possible solutions for the exercise

### Code Example
- **Language**: String (required) - e.g., "Python", "URDF", "launch"
- **Code**: String (required) - The actual code snippet with syntax highlighting
- **Comments**: Array of String - Explanatory comments about WHY and WHAT the code does
- **Dependencies**: Array of Object - Required libraries/packages with versions
- **Validation Status**: String - "tested", "untested", or "pending"

### Mathematical Expression
- **Expression**: String (required) - The mathematical formula or concept
- **Explanation**: String (required) - What the expression represents
- **Validation**: String (required) - "verified", "unverified", or "pending"
- **Source**: String - Reference or derivation for the expression

### Diagram
- **Type**: String (required) - e.g., "architecture", "workflow", "spatial"
- **Description**: String (required) - What the diagram represents
- **Source**: String - File path or reference to the diagram
- **Alt Text**: String (required) - Accessibility description

### Glossary Term
- **Term**: String (required) - The term being defined
- **Definition**: String (required) - Clear, concise definition
- **Module**: String - Which module the term is first used in
- **Related Terms**: Array of String - Other related terms

### Notation Symbol
- **Symbol**: String (required) - The mathematical or technical symbol
- **Meaning**: String (required) - What the symbol represents
- **Context**: String (required) - Where this symbol is used
- **Alternative Representations**: Array of String - Other ways to represent the same concept

## Relationships

- Module contains many Chapters
- Chapter contains many Exercises
- Chapter contains many Code Examples
- Chapter contains many Mathematical Expressions
- Chapter contains many Diagrams
- Chapter contains many Glossary Terms (references)
- Chapter references many Notation Symbols

## Validation Rules

### Module Validation
- Name must be one of the four constitutional modules: "The Robotic Nervous System (ROS 2)", "The Digital Twin (Gazebo & Unity)", "The AI-Robot Brain (NVIDIA Isaac™)", "Vision-Language-Action (VLA)"
- Must have at least one Chapter
- Learning Objectives must be measurable
- Prerequisites must be explicitly declared

### Chapter Validation
- Title must be unique within its Module
- Must have 3 Exercises of different types (logical, conceptual, implementation)
- Content must follow the constitutional format: Core Concepts → Implementation → Examples
- All frontmatter fields must be present: title, description, keywords, sidebar_position
- Prerequisites must be explicitly declared
- Learning Objectives must be measurable and stated at the start

### Code Example Validation
- Must be in a supported language (Python, URDF, launch configurations, etc.)
- Code must be tested and functional
- All dependencies must be specified with versions
- Comments must explain both WHAT the code does and WHY it does it

### Mathematical Expression Validation
- Must be validated through verification (Principle II)
- Source or derivation must be provided
- Must be consistently referenced in the notation.md file

## State Transitions

### Chapter States
1. **Draft** - Initial state when chapter is first created
2. **In Review** - Content has been written but needs validation
3. **In Validation** - Mathematical expressions and code examples are being verified
4. **Ready for Publication** - All validations passed
5. **Published** - Chapter is available in the live documentation