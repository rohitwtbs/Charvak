#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include "rigid_body.h"
#include "collision_detection.h"
#include "collision_response.h"
#include "integration.h"

// Maximum number of bodies and collisions
#define MAX_BODIES 1000
#define MAX_COLLISIONS 2000

// Physics world structure
typedef struct {
    // Bodies management
    RigidBody* bodies[MAX_BODIES];
    int body_count;
    
    // Collision pairs from this frame
    CollisionInfo collisions[MAX_COLLISIONS];
    int collision_count;
    
    // World properties
    Vector3 gravity;
    float timestep;
    IntegrationMethod integration_method;
    
    // Damping
    float linear_damping;
    float angular_damping;
    
    // Simulation control
    bool is_paused;
    float time_scale;
    int simulation_iterations;
    
    // Performance tracking
    float last_frame_time;
    int collision_checks_performed;
} PhysicsWorld;

// World management
PhysicsWorld* physics_world_create(void);
void physics_world_destroy(PhysicsWorld* world);
void physics_world_init(PhysicsWorld* world);

// Body management
int physics_world_add_body(PhysicsWorld* world, RigidBody* body);
bool physics_world_remove_body(PhysicsWorld* world, int body_id);
RigidBody* physics_world_get_body(PhysicsWorld* world, int body_id);
void physics_world_clear_bodies(PhysicsWorld* world);

// World properties
void physics_world_set_gravity(PhysicsWorld* world, Vector3 gravity);
void physics_world_set_timestep(PhysicsWorld* world, float timestep);
void physics_world_set_integration_method(PhysicsWorld* world, IntegrationMethod method);
void physics_world_set_damping(PhysicsWorld* world, float linear_damping, float angular_damping);

// Simulation control
void physics_world_step(PhysicsWorld* world);
void physics_world_step_with_dt(PhysicsWorld* world, float dt);
void physics_world_pause(PhysicsWorld* world, bool paused);
void physics_world_set_time_scale(PhysicsWorld* world, float scale);

// Collision detection and response
void physics_world_detect_collisions(PhysicsWorld* world);
void physics_world_resolve_collisions(PhysicsWorld* world);

// Utility functions
void physics_world_apply_forces(PhysicsWorld* world);
void physics_world_integrate_bodies(PhysicsWorld* world, float dt);
void physics_world_wake_sleeping_bodies(PhysicsWorld* world);

// Debug and statistics
int physics_world_get_body_count(PhysicsWorld* world);
int physics_world_get_collision_count(PhysicsWorld* world);
float physics_world_get_total_kinetic_energy(PhysicsWorld* world);

#endif // PHYSICS_WORLD_H