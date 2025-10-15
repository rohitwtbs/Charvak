# Charvak

![Charvak Logo](charvak.png)

**Charvak** - A comprehensive gaming engine ecosystem featuring both Python and C implementations.

## Overview

Charvak is an open-source game engine designed to make AAA games accessible through multiple programming languages and deployment targets. The project consists of both high-level Python components and high-performance C implementations.

**Long-term goal**: Write game logic in Python and build for all platforms (VR, Windows, Linux, Mac, Android, iOS, Web, etc.)

![Architecture](architecture.png)

## Project Structure

```
Charvak/
├── charvak_engine/           # Python-based engine components
│   ├── main.py              # Main engine entry point
│   ├── compute/
│   │   └── physics.py       # Python physics implementation
│   ├── graphics/
│   │   └── point_renderer.py # Graphics rendering
│   └── scene/
│       └── scene.py         # Scene management
└── physics_engine_c/        # High-performance C physics engine
    ├── include/             # Header files
    ├── src/                 # C implementation
    ├── examples/            # Demo programs
    ├── build/               # Compiled libraries and executables
    └── README.md            # Physics engine documentation
```

## Components

### 🐍 Python Engine (`charvak_engine/`)
- High-level game logic and scripting
- Scene management and rendering pipeline
- Cross-platform compatibility through Python
- Easy prototyping and development workflow

### ⚡ C Physics Engine (`physics_engine_c/`)
- **High-performance 3D physics simulation**
- **Rigid body dynamics** with full 6DOF motion
- **Advanced collision detection**: Sphere, AABB, and plane primitives
- **Multiple integration methods**: Euler, Verlet, and Runge-Kutta 4
- **Optimized performance**: Broad-phase detection, sleeping bodies
- **Cross-platform C99** implementation with no external dependencies
- **Complete build system** with static/shared libraries

#### Physics Engine Features
- ✅ Real-time collision detection and response
- ✅ Impulse-based physics with restitution and friction
- ✅ Gravity, damping, and force application
- ✅ Memory-efficient body management
- ✅ Production-ready with comprehensive API

## Quick Start

### Python Engine
```python
# Run the main engine
cd charvak_engine
python main.py
```

### C Physics Engine
```bash
# Build and run physics demos
cd physics_engine_c
make run-demo

# Build libraries
make static        # Static library
make shared        # Shared library
make              # Build everything
```

### Physics Engine Usage Example
```c
#include "physics_world.h"

// Create physics world
PhysicsWorld* world = physics_world_create();
physics_world_set_gravity(world, vector3_create(0.0f, -9.81f, 0.0f));

// Add a bouncing sphere
RigidBody* sphere = rigid_body_create();
rigid_body_init_sphere(sphere, vector3_create(0.0f, 5.0f, 0.0f), 1.0f, 1.0f);
physics_world_add_body(world, sphere);

// Simulate physics
physics_world_step(world);
```

## Build Requirements

### Python Components
- Python 3.7+
- Dependencies listed in requirements files

### C Physics Engine
- GCC or Clang with C99 support
- Make build system
- Math library (libm)

## Integration Goals

- **Raylib integration** for cross-platform graphics
- **WebAssembly compilation** for web deployment
- **Python-C bindings** for seamless interoperability
- **Multi-platform builds** for all target devices

## Development Timeline

- **May 6, 2024**: Project inception
- **October 15, 2025**: C physics engine implementation complete
- **Ongoing**: Python engine development and integration

## Contributing

Contributions welcome! The project is designed with modular architecture to support:
- New physics features and optimizations
- Graphics and rendering improvements  
- Platform-specific optimizations
- Language bindings and integrations

## License

Open source - see license file for details.