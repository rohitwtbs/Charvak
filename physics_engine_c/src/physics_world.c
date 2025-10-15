#include "../include/physics_world.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

PhysicsWorld* physics_world_create(void) {
    PhysicsWorld* world = (PhysicsWorld*)malloc(sizeof(PhysicsWorld));
    if (!world) return NULL;
    
    physics_world_init(world);
    return world;
}

void physics_world_destroy(PhysicsWorld* world) {
    if (!world) return;
    
    // Clean up all bodies
    physics_world_clear_bodies(world);
    free(world);
}

void physics_world_init(PhysicsWorld* world) {
    if (!world) return;
    
    // Initialize body management
    memset(world->bodies, 0, sizeof(world->bodies));
    world->body_count = 0;
    world->collision_count = 0;
    
    // Set default world properties
    world->gravity = vector3_create(0.0f, -9.81f, 0.0f);  // Earth gravity
    world->timestep = 1.0f / 60.0f;  // 60 FPS
    world->integration_method = INTEGRATION_VERLET;
    
    // Set default damping
    world->linear_damping = 0.01f;
    world->angular_damping = 0.05f;
    
    // Simulation control
    world->is_paused = false;
    world->time_scale = 1.0f;
    world->simulation_iterations = 1;
    
    // Performance tracking
    world->last_frame_time = 0.0f;
    world->collision_checks_performed = 0;
}

int physics_world_add_body(PhysicsWorld* world, RigidBody* body) {
    if (!world || !body || world->body_count >= MAX_BODIES) {
        return -1;
    }
    
    world->bodies[world->body_count] = body;
    world->body_count++;
    
    return body->id;
}

bool physics_world_remove_body(PhysicsWorld* world, int body_id) {
    if (!world) return false;
    
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i] && world->bodies[i]->id == body_id) {
            // Shift remaining bodies down
            for (int j = i; j < world->body_count - 1; j++) {
                world->bodies[j] = world->bodies[j + 1];
            }
            world->bodies[world->body_count - 1] = NULL;
            world->body_count--;
            return true;
        }
    }
    
    return false;
}

RigidBody* physics_world_get_body(PhysicsWorld* world, int body_id) {
    if (!world) return NULL;
    
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i] && world->bodies[i]->id == body_id) {
            return world->bodies[i];
        }
    }
    
    return NULL;
}

void physics_world_clear_bodies(PhysicsWorld* world) {
    if (!world) return;
    
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i]) {
            rigid_body_destroy(world->bodies[i]);
            world->bodies[i] = NULL;
        }
    }
    
    world->body_count = 0;
}

void physics_world_set_gravity(PhysicsWorld* world, Vector3 gravity) {
    if (world) {
        world->gravity = gravity;
    }
}

void physics_world_set_timestep(PhysicsWorld* world, float timestep) {
    if (world && timestep > 0.0f) {
        world->timestep = timestep;
    }
}

void physics_world_set_integration_method(PhysicsWorld* world, IntegrationMethod method) {
    if (world) {
        world->integration_method = method;
    }
}

void physics_world_set_damping(PhysicsWorld* world, float linear_damping, float angular_damping) {
    if (world) {
        world->linear_damping = fmaxf(0.0f, fminf(1.0f, linear_damping));
        world->angular_damping = fmaxf(0.0f, fminf(1.0f, angular_damping));
    }
}

void physics_world_step(PhysicsWorld* world) {
    if (!world) return;
    
    physics_world_step_with_dt(world, world->timestep);
}

void physics_world_step_with_dt(PhysicsWorld* world, float dt) {
    if (!world || world->is_paused || dt <= 0.0f) return;
    
    // Apply time scale
    float scaled_dt = dt * world->time_scale;
    
    // Perform multiple simulation iterations for stability
    float sub_dt = scaled_dt / (float)world->simulation_iterations;
    
    for (int iter = 0; iter < world->simulation_iterations; iter++) {
        // Wake up sleeping bodies that might be affected by moving objects
        physics_world_wake_sleeping_bodies(world);
        
        // Apply forces (gravity, user forces, etc.)
        physics_world_apply_forces(world);
        
        // Integrate motion
        physics_world_integrate_bodies(world, sub_dt);
        
        // Detect collisions
        physics_world_detect_collisions(world);
        
        // Resolve collisions
        physics_world_resolve_collisions(world);
        
        // Apply damping
        for (int i = 0; i < world->body_count; i++) {
            if (world->bodies[i]) {
                apply_damping(world->bodies[i], world->linear_damping, world->angular_damping);
            }
        }
    }
}

void physics_world_pause(PhysicsWorld* world, bool paused) {
    if (world) {
        world->is_paused = paused;
    }
}

void physics_world_set_time_scale(PhysicsWorld* world, float scale) {
    if (world && scale >= 0.0f) {
        world->time_scale = scale;
    }
}

void physics_world_detect_collisions(PhysicsWorld* world) {
    if (!world) return;
    
    world->collision_count = 0;
    world->collision_checks_performed = 0;
    
    // Broad phase: check all pairs of bodies
    for (int i = 0; i < world->body_count; i++) {
        for (int j = i + 1; j < world->body_count; j++) {
            RigidBody* body_a = world->bodies[i];
            RigidBody* body_b = world->bodies[j];
            
            if (!body_a || !body_b) continue;
            
            // Skip if both bodies are static
            if (body_a->is_static && body_b->is_static) continue;
            
            // Skip if both bodies are sleeping
            if (body_a->is_sleeping && body_b->is_sleeping) continue;
            
            world->collision_checks_performed++;
            
            // Check for collision
            if (world->collision_count < MAX_COLLISIONS) {
                CollisionInfo* collision = &world->collisions[world->collision_count];
                
                if (detect_collision(body_a, body_b, collision)) {
                    world->collision_count++;
                    
                    // Wake up sleeping bodies involved in collision
                    body_a->is_sleeping = false;
                    body_b->is_sleeping = false;
                }
            }
        }
    }
}

void physics_world_resolve_collisions(PhysicsWorld* world) {
    if (!world) return;
    
    // Resolve all detected collisions
    for (int i = 0; i < world->collision_count; i++) {
        resolve_collision(&world->collisions[i]);
    }
}

void physics_world_apply_forces(PhysicsWorld* world) {
    if (!world) return;
    
    // Apply gravity to all non-static bodies
    for (int i = 0; i < world->body_count; i++) {
        RigidBody* body = world->bodies[i];
        if (!body || body->is_static || body->is_sleeping) continue;
        
        // Apply gravity: F = mg
        Vector3 gravity_force = vector3_scale(world->gravity, body->mass);
        rigid_body_add_force(body, gravity_force);
    }
}

void physics_world_integrate_bodies(PhysicsWorld* world, float dt) {
    if (!world) return;
    
    for (int i = 0; i < world->body_count; i++) {
        RigidBody* body = world->bodies[i];
        if (!body || body->is_static) continue;
        
        integrate_body(body, dt, world->integration_method);
    }
}

void physics_world_wake_sleeping_bodies(PhysicsWorld* world) {
    if (!world) return;
    
    // Simple approach: wake all bodies within a certain distance of moving bodies
    const float wake_distance = 5.0f;
    
    for (int i = 0; i < world->body_count; i++) {
        RigidBody* moving_body = world->bodies[i];
        if (!moving_body || moving_body->is_static || moving_body->is_sleeping) continue;
        
        // Check if this body is moving fast enough to wake others
        float speed_sq = vector3_length_squared(moving_body->velocity);
        if (speed_sq < 0.1f) continue;
        
        for (int j = 0; j < world->body_count; j++) {
            if (i == j) continue;
            
            RigidBody* sleeping_body = world->bodies[j];
            if (!sleeping_body || !sleeping_body->is_sleeping) continue;
            
            float distance = vector3_distance(moving_body->position, sleeping_body->position);
            if (distance < wake_distance) {
                sleeping_body->is_sleeping = false;
            }
        }
    }
}

int physics_world_get_body_count(PhysicsWorld* world) {
    return world ? world->body_count : 0;
}

int physics_world_get_collision_count(PhysicsWorld* world) {
    return world ? world->collision_count : 0;
}

float physics_world_get_total_kinetic_energy(PhysicsWorld* world) {
    if (!world) return 0.0f;
    
    float total_energy = 0.0f;
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i]) {
            total_energy += rigid_body_get_kinetic_energy(world->bodies[i]);
        }
    }
    
    return total_energy;
}