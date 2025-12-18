# Physical AI and Humanoid Robotics Educational Resource

This repository contains a comprehensive educational resource for graduate-level engineering students studying Physical AI and Humanoid Robotics. The content is organized into four modules covering the fundamental and advanced aspects of modern robotics.

## Course Overview

This 13-week curriculum is organized into 4 modules:

1. **Module 1: The Robotic Nervous System (ROS 2)** - 5 weeks
2. **Module 2: The Digital Twin (Gazebo & Unity)** - 2 weeks  
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - 3 weeks
4. **Module 4: Vision-Language-Action (VLA)** - 3 weeks

Each module includes multiple chapters with learning objectives, prerequisites, core concepts, implementation examples, exercises, and summaries.

## Getting Started

### Prerequisites

- Node.js LTS (version 18 or higher)
- npm 8 or higher
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/hackathone01_v3.git
   ```

2. Navigate to the project directory:
   ```bash
   cd hackathone01_v3
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

### Local Development

1. Start the development server:
   ```bash
   npm start
   ```

2. Open your browser at `http://localhost:3000` to view the site

### Build for Production

To build the static site for deployment:

```bash
npm run build
```

## Project Structure

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
├── getting-started.md
└── capstone-project.md
```

## Contributing

We welcome contributions to improve this educational resource. Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
5. Push to the branch (`git push origin feature/AmazingFeature`)
6. Open a Pull Request

## License

This educational resource is provided for academic purposes.

## Acknowledgments

- [Docusaurus](https://docusaurus.io/) for the static site generation framework
- The open-source robotics community for valuable resources and documentation