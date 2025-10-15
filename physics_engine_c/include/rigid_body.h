#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "vector_math.h"
#include <stdbool.h>

// Shape types for collision detection
typedef enum {
    SHAPE_SPHERE,
    SHAPE_AABB,
    SHAPE_PLANE
} ShapeType;

// Sphere collision shape
typedef struct {
    float radius;
} SphereShape;

// Axis-Aligned Bounding Box collision shape
typedef struct {
    Vector3 half_extents;  // Half-widths in each dimension
} AABBShape;

// Plane collision shape
typedef struct {
    Vector3 normal;
    float distance;  // Distance from origin along normal
} PlaneShape;

// Union for different collision shapes
typedef union {
    SphereShape sphere;
    AABBShape aabb;
    PlaneShape plane;
} CollisionShape;

// Rigid body structure
typedef struct {
    // Linear motion
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    
    // Angular motion (simplified - using Euler angles for now)
    Vector3 rotation;
    Vector3 angular_velocity;
    Vector3 angular_acceleration;
    
    // Physical properties
    float mass;
    float inverse_mass;     // 1/mass, cached for performance
    float restitution;      // Bounciness (0 = no bounce, 1 = perfect bounce)
    float friction;         // Surface friction coefficient
    
    // Force and torque accumulators
    Vector3 force_accumulator;
    Vector3 torque_accumulator;
    
    // Collision properties
    ShapeType shape_type;
    CollisionShape shape;
    
    // State flags
    bool is_static;         // Static bodies don't move
    bool is_sleeping;       // Sleeping bodies are temporarily inactive
    
    // Unique identifier
    int id;
} RigidBody;

// Rigid body creation and destruction
RigidBody* rigid_body_create(void);
void rigid_body_destroy(RigidBody* body);

// Initialization functions
void rigid_body_init_sphere(RigidBody* body, Vector3 position, float radius, float mass);
void rigid_body_init_aabb(RigidBody* body, Vector3 position, Vector3 half_extents, float mass);
void rigid_body_init_plane(RigidBody* body, Vector3 normal, float distance);

// Property setters
void rigid_body_set_position(RigidBody* body, Vector3 position);
void rigid_body_set_velocity(RigidBody* body, Vector3 velocity);
void rigid_body_set_mass(RigidBody* body, float mass);
void rigid_body_set_restitution(RigidBody* body, float restitution);
void rigid_body_set_friction(RigidBody* body, float friction);
void rigid_body_set_static(RigidBody* body, bool is_static);

// Force application
void rigid_body_add_force(RigidBody* body, Vector3 force);
void rigid_body_add_force_at_point(RigidBody* body, Vector3 force, Vector3 point);
void rigid_body_add_torque(RigidBody* body, Vector3 torque);
void rigid_body_add_impulse(RigidBody* body, Vector3 impulse);

// Utility functions
void rigid_body_clear_forces(RigidBody* body);
Vector3 rigid_body_get_point_velocity(RigidBody* body, Vector3 point);
float rigid_body_get_kinetic_energy(RigidBody* body);

#endif // RIGID_BODY_H