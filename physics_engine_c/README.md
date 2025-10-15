# Charvak Physics Engine (C Implementation)

A lightweight, high-performance 3D physics engine written in C for real-time simulations and games.

## Features

- **Rigid Body Dynamics**: Full 3D rigid body simulation with linear and angular motion
- **Collision Detection**: Sphere-sphere, sphere-AABB, AABB-AABB, and plane collision detection
- **Collision Response**: Impulse-based collision resolution with restitution and friction
- **Numerical Integration**: Multiple integration methods (Euler, Verlet, RK4)
- **Physics World**: Complete world management with gravity, damping, and time control
- **Optimized**: Broad-phase collision detection and sleeping bodies for performance
- **Cross-Platform**: Pure C99 implementation with no external dependencies

## Directory Structure

```
physics_engine_c/
├── include/           # Header files
│   ├── vector_math.h      # Vector and math operations
│   ├── rigid_body.h       # Rigid body definitions
│   ├── integration.h      # Numerical integration methods
│   ├── collision_detection.h    # Collision detection algorithms
│   ├── collision_response.h     # Collision response and resolution
│   └── physics_world.h          # Main physics world management
├── src/              # Source implementation files
├── examples/         # Example programs and demos
│   └── demo.c            # Bouncing spheres and collision demos
├── build/            # Build output directory (created by make)
├── Makefile          # Build system
└── README.md         # This file
```

## Building

### Requirements
- GCC or Clang compiler with C99 support
- Make build system
- Math library (libm)

### Quick Start
```bash
# Build everything (static lib, shared lib, and demo)
make

# Build and run the demo
make run-demo

# Build only static library
make static

# Build only shared library  
make shared

# Clean build files
make clean
```

### Advanced Build Options
```bash
# Debug build with symbols and no optimization
make debug

# Release build with full optimization
make release

# Memory checking with valgrind
make valgrind

# Install to system directories
sudo make install

# Format code (requires clang-format)
make format

# Static analysis (requires cppcheck)
make analyze
```

## Usage Example

```c
#include "physics_world.h"

int main() {
    // Create physics world
    PhysicsWorld* world = physics_world_create();
    physics_world_set_gravity(world, vector3_create(0.0f, -9.81f, 0.0f));
    
    // Create a sphere
    RigidBody* sphere = rigid_body_create();
    rigid_body_init_sphere(sphere, vector3_create(0.0f, 5.0f, 0.0f), 1.0f, 1.0f);
    physics_world_add_body(world, sphere);
    
    // Create ground plane
    RigidBody* ground = rigid_body_create();
    rigid_body_init_plane(ground, vector3_create(0.0f, 1.0f, 0.0f), 0.0f);
    physics_world_add_body(world, ground);
    
    // Run simulation
    for (int i = 0; i < 1000; i++) {
        physics_world_step(world);
        printf("Sphere height: %.2f\n", sphere->position.y);
    }
    
    // Cleanup
    physics_world_destroy(world);
    return 0;
}
```

## API Reference

### Vector Math
- `Vector3 vector3_create(float x, float y, float z)`
- `Vector3 vector3_add(Vector3 a, Vector3 b)`
- `Vector3 vector3_subtract(Vector3 a, Vector3 b)`
- `float vector3_dot(Vector3 a, Vector3 b)`
- `Vector3 vector3_cross(Vector3 a, Vector3 b)`
- `Vector3 vector3_normalize(Vector3 v)`

### Rigid Bodies
- `RigidBody* rigid_body_create()`
- `void rigid_body_init_sphere(RigidBody* body, Vector3 pos, float radius, float mass)`
- `void rigid_body_init_aabb(RigidBody* body, Vector3 pos, Vector3 half_extents, float mass)`
- `void rigid_body_add_force(RigidBody* body, Vector3 force)`
- `void rigid_body_set_velocity(RigidBody* body, Vector3 velocity)`

### Physics World
- `PhysicsWorld* physics_world_create()`
- `int physics_world_add_body(PhysicsWorld* world, RigidBody* body)`
- `void physics_world_set_gravity(PhysicsWorld* world, Vector3 gravity)`
- `void physics_world_step(PhysicsWorld* world)`
- `void physics_world_destroy(PhysicsWorld* world)`

## Integration Methods

The engine supports multiple numerical integration methods:

1. **Euler Integration**: Simple and fast, suitable for non-critical simulations
2. **Verlet Integration**: Better stability and energy conservation (default)
3. **Runge-Kutta 4**: Highest accuracy but more computationally expensive

## Collision Shapes

Supported collision primitives:
- **Spheres**: Perfect for balls, particles, and rounded objects
- **AABBs**: Axis-aligned boxes for containers, walls, and rectangular objects  
- **Planes**: Infinite planes for ground, walls, and boundaries

## Performance Features

- **Broad-phase collision detection**: AABB overlap tests before expensive narrow-phase
- **Sleeping bodies**: Inactive bodies are excluded from simulation until disturbed
- **Spatial optimization**: Bodies are put to sleep when velocity drops below threshold
- **Memory management**: Object pooling and efficient memory layout

## Demo Programs

Run `make run-demo` to see:
1. **Bouncing Spheres**: Multiple spheres with different properties bouncing in a box
2. **Sphere-Box Collision**: Sphere colliding with a box and ground
3. **Basic Tests**: Verification of core functionality

## Extensions and Customization

The engine is designed to be easily extensible:
- Add new collision shapes by implementing detection/response functions
- Integrate with graphics libraries for visualization
- Add constraints and joints for complex mechanisms
- Implement spatial partitioning (octrees, BSP) for large worlds
- Add multithreading for performance scaling

## Performance Tips

1. Use `INTEGRATION_VERLET` for best stability/performance balance
2. Adjust damping values to prevent numerical instabilities
3. Keep timestep reasonable (1/60 second or smaller)
4. Use sleeping bodies for static or slow-moving objects
5. Consider spatial partitioning for scenes with >100 objects

## License

This physics engine is part of the Charvak project. See main project license for details.

## Contributing

Contributions welcome! Please follow the existing code style and add tests for new features.