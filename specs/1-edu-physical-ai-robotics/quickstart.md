# Quick Start Guide: Contributing to the Physical AI and Humanoid Robotics Book

## Prerequisites

Before you begin contributing to this educational resource, you'll need:

1. **Node.js**: Install the LTS version (20.x or higher)
2. **npm**: Version 8 or higher (typically comes with Node.js)
3. **Git**: For version control
4. **GitHub Account**: To contribute changes
5. **Text Editor**: VS Code, Vim, or your preferred editor

## Setting Up the Development Environment

1. **Clone the repository**:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Start the local development server**:
   ```bash
   npm start
   ```
   This command starts a local server and opens your site in a browser. Most changes are reflected live without having to restart the server.

## Project Structure

The documentation is organized as follows:

```
docs/
├── intro.md
├── modules/
│   ├── module-1-ros2/
│   │   ├── intro.md
│   │   ├── chapter-1.md
│   │   ├── chapter-2.md
│   │   ├── chapter-3.md
│   │   ├── chapter-4.md
│   │   └── chapter-5.md
│   ├── module-2-digital-twin/
│   │   ├── intro.md
│   │   ├── chapter-6.md
│   │   └── chapter-7.md
│   ├── module-3-ai-brain/
│   │   ├── intro.md
│   │   ├── chapter-8.md
│   │   ├── chapter-9.md
│   │   └── chapter-10.md
│   └── module-4-vla/
│       ├── intro.md
│       ├── chapter-11.md
│       ├── chapter-12.md
│       └── chapter-13.md
├── glossary.md
├── notation.md
└── capstone-project.md
```

## Adding New Content

### Creating a New Chapter

1. **Create the markdown file** in the appropriate module directory:
   ```bash
   # For example, adding a new chapter to Module 1
   touch docs/modules/module-1-ros2/chapter-6.md
   ```

2. **Add the required frontmatter** at the top of your chapter:
   ```markdown
   ---
   title: "Chapter 6 Title"
   description: "Brief description of the chapter content"
   keywords: [list, of, relevant, keywords]
   sidebar_position: 6
   ---
   ```

3. **Follow the constitutional format**:
   - Learning Objectives (measurable)
   - Prerequisites (explicitly stated)
   - Core Concepts
   - Implementation
   - Examples
   - Summary
   - Exercises (minimum 3: logical, conceptual, implementation)

4. **Update the sidebar** by adding your chapter to `sidebars.js` in the appropriate position.

### Adding Code Examples

When including code examples:

1. **Use proper syntax highlighting**:
   ```python
   # Your Python code here
   ```

2. **Include explanatory comments** that explain both WHAT the code does and WHY it does it:
   ```python
   # Initialize the ROS node (WHAT)
   # This is required to communicate with the ROS master (WHY)
   rospy.init_node('example_node')
   ```

3. **Specify all dependencies and versions** in comments or in a requirements section.

4. **Mark pseudocode explicitly** if it's not intended to be executable:
   ```markdown
   ```pseudocode
   // This is pseudocode
   ```

### Adding Mathematical Expressions

1. **Use LaTeX syntax** for mathematical expressions:
   ```markdown
   The position of the robot is given by $p = p_0 + vt$ where $p_0$ is the initial position, $v$ is velocity, and $t$ is time.
   ```

2. **Define notation** in `docs/notation.md` and reference it in your content.

3. **Validate mathematical expressions** through derivation or simulation before publishing.

## Quality Requirements

### Constitutional Compliance

All content must adhere to the project constitution, including:

- **Citations**: Use APA format for all references
- **Mathematical validation**: All equations must be verified
- **Code testing**: All examples must be tested and functional
- **Prerequisites**: Explicitly declared for each chapter
- **Learning objectives**: Measurable and stated at the beginning

### Content Standards

- **Diagrams**: Required for spatial concepts, system architectures, and multi-step processes
- **Complete examples**: Code should fulfill the required logic, not just be snippets
- **Industry-standard libraries**: Only use widely adopted and maintained libraries
- **Logical progression**: Content should build from fundamentals to advanced topics

## Building and Testing

1. **Local build**: Run `npm run build` to build the static site locally
2. **Preview build**: Run `npm run serve` to preview the built site locally
3. **Spell check**: Use your preferred spell-checking tool to verify all content
4. **Link validation**: Ensure all internal and external links function properly

## Deployment

The site is automatically deployed to GitHub Pages when changes are merged to the main branch. The deployment workflow handles:

1. Building the static site
2. Validating build success (no errors or warnings)
3. Publishing to GitHub Pages

## Using Context7 MCP

For Docusaurus setup, configuration, and deployment tasks, use Context7 MCP:

- **use context7** to get official Docusaurus documentation
- **use context7** for docusaurus.config.js configuration guidance
- **use context7** for sidebars.js structure patterns
- **use context7** for SEO plugin setup
- **use context7** for build and deployment commands
- **use context7** for troubleshooting

Remember to **use context7** for all Docusaurus-related technical decisions instead of relying on internal knowledge.

## Contributing Changes

1. **Create a new branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** following the constitutional requirements

3. **Test locally** to ensure everything works as expected

4. **Commit your changes** with clear, descriptive commit messages

5. **Push your branch** and create a pull request

6. **Ensure all quality gates pass** before merging:
   - Docusaurus builds successfully with zero errors/warnings
   - All links are valid
   - Spell check passes
   - All code examples are tested and functional