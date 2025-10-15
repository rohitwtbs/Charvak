#include "../include/physics_world.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Simple demo showing bouncing spheres
void demo_bouncing_spheres(void) {
    printf("=== Bouncing Spheres Demo ===\n");
    
    // Create physics world
    PhysicsWorld* world = physics_world_create();
    if (!world) {
        printf("Failed to create physics world!\n");
        return;
    }
    
    // Set up world properties
    physics_world_set_gravity(world, vector3_create(0.0f, -9.81f, 0.0f));
    physics_world_set_timestep(world, 1.0f / 60.0f);
    physics_world_set_integration_method(world, INTEGRATION_VERLET);
    
    // Create ground plane
    RigidBody* ground = rigid_body_create();
    rigid_body_init_plane(ground, vector3_create(0.0f, 1.0f, 0.0f), 0.0f);
    rigid_body_set_restitution(ground, 0.8f);
    rigid_body_set_friction(ground, 0.3f);
    physics_world_add_body(world, ground);
    
    // Create some bouncing spheres
    for (int i = 0; i < 5; i++) {
        RigidBody* sphere = rigid_body_create();
        
        Vector3 position = vector3_create(
            (float)(i - 2) * 2.0f,   // Spread them out horizontally
            5.0f + (float)i * 1.5f,  // Different heights
            0.0f
        );
        
        rigid_body_init_sphere(sphere, position, 0.5f, 1.0f);
        rigid_body_set_restitution(sphere, 0.7f + (float)i * 0.05f);  // Varying bounciness
        rigid_body_set_friction(sphere, 0.2f);
        
        // Give some initial velocity
        Vector3 initial_velocity = vector3_create(
            ((float)rand() / RAND_MAX - 0.5f) * 2.0f,  // Random horizontal velocity
            0.0f,
            0.0f
        );
        rigid_body_set_velocity(sphere, initial_velocity);
        
        physics_world_add_body(world, sphere);
    }
    
    // Add some walls
    RigidBody* left_wall = rigid_body_create();
    rigid_body_init_plane(left_wall, vector3_create(1.0f, 0.0f, 0.0f), 10.0f);
    rigid_body_set_restitution(left_wall, 0.9f);
    physics_world_add_body(world, left_wall);
    
    RigidBody* right_wall = rigid_body_create();
    rigid_body_init_plane(right_wall, vector3_create(-1.0f, 0.0f, 0.0f), 10.0f);
    rigid_body_set_restitution(right_wall, 0.9f);
    physics_world_add_body(world, right_wall);
    
    // Simulation loop
    printf("Running simulation for 10 seconds...\n");
    printf("Time\tSphere Positions (y-coordinate only)\n");
    
    float simulation_time = 0.0f;
    float total_time = 10.0f;
    
    while (simulation_time < total_time) {
        // Step the physics
        physics_world_step(world);
        simulation_time += world->timestep;
        
        // Print status every 0.5 seconds
        if (fmodf(simulation_time, 0.5f) < world->timestep) {
            printf("%.1fs\t", simulation_time);
            
            // Print sphere positions (skip ground plane)
            for (int i = 1; i < physics_world_get_body_count(world) - 2; i++) {  // Skip ground and walls
                RigidBody* body = world->bodies[i];
                if (body && body->shape_type == SHAPE_SPHERE) {
                    printf("%.2f ", body->position.y);
                }
            }
            printf("\n");
        }
    }
    
    // Print final statistics
    printf("\nFinal Statistics:\n");
    printf("Total bodies: %d\n", physics_world_get_body_count(world));
    printf("Total kinetic energy: %.2f J\n", physics_world_get_total_kinetic_energy(world));
    printf("Last collision count: %d\n", physics_world_get_collision_count(world));
    
    // Clean up
    physics_world_destroy(world);
    printf("Demo completed successfully!\n");
}

// Demo showing sphere-box collisions
void demo_sphere_box_collision(void) {
    printf("\n=== Sphere-Box Collision Demo ===\n");
    
    PhysicsWorld* world = physics_world_create();
    if (!world) {
        printf("Failed to create physics world!\n");
        return;
    }
    
    physics_world_set_gravity(world, vector3_create(0.0f, -9.81f, 0.0f));
    
    // Create ground
    RigidBody* ground = rigid_body_create();
    rigid_body_init_plane(ground, vector3_create(0.0f, 1.0f, 0.0f), 0.0f);
    physics_world_add_body(world, ground);
    
    // Create a box
    RigidBody* box = rigid_body_create();
    rigid_body_init_aabb(box, vector3_create(0.0f, 2.0f, 0.0f), 
                        vector3_create(1.0f, 1.0f, 1.0f), 5.0f);
    rigid_body_set_restitution(box, 0.6f);
    physics_world_add_body(world, box);
    
    // Create a sphere that will hit the box
    RigidBody* sphere = rigid_body_create();
    rigid_body_init_sphere(sphere, vector3_create(-5.0f, 5.0f, 0.0f), 0.5f, 1.0f);
    rigid_body_set_velocity(sphere, vector3_create(3.0f, -1.0f, 0.0f));
    rigid_body_set_restitution(sphere, 0.8f);
    physics_world_add_body(world, sphere);
    
    printf("Sphere starts at (-5, 5, 0) moving towards box at (0, 2, 0)\n");
    printf("Time\tSphere Position\t\tBox Position\n");
    
    float simulation_time = 0.0f;
    float total_time = 5.0f;
    
    while (simulation_time < total_time) {
        physics_world_step(world);
        simulation_time += world->timestep;
        
        if (fmodf(simulation_time, 0.2f) < world->timestep) {
            printf("%.1fs\t(%.2f, %.2f, %.2f)\t(%.2f, %.2f, %.2f)\n",
                   simulation_time,
                   sphere->position.x, sphere->position.y, sphere->position.z,
                   box->position.x, box->position.y, box->position.z);
        }
    }
    
    physics_world_destroy(world);
    printf("Sphere-Box demo completed!\n");
}

// Test basic physics engine functionality
void run_basic_tests(void) {
    printf("\n=== Basic Physics Engine Tests ===\n");
    
    // Test vector operations
    printf("Testing vector operations...\n");
    Vector3 a = vector3_create(1.0f, 2.0f, 3.0f);
    Vector3 b = vector3_create(4.0f, 5.0f, 6.0f);
    Vector3 sum = vector3_add(a, b);
    printf("Vector addition: (1,2,3) + (4,5,6) = (%.1f,%.1f,%.1f)\n", sum.x, sum.y, sum.z);
    
    float dot = vector3_dot(a, b);
    printf("Dot product: (1,2,3) · (4,5,6) = %.1f\n", dot);
    
    // Test rigid body creation
    printf("Testing rigid body creation...\n");
    RigidBody* body = rigid_body_create();
    rigid_body_init_sphere(body, vector3_create(0.0f, 0.0f, 0.0f), 1.0f, 2.0f);
    printf("Created sphere: radius=%.1f, mass=%.1f\n", 
           body->shape.sphere.radius, body->mass);
    
    // Test force application
    printf("Testing force application...\n");
    Vector3 force = vector3_create(0.0f, 10.0f, 0.0f);
    rigid_body_add_force(body, force);
    printf("Applied force: (%.1f, %.1f, %.1f)\n", 
           body->force_accumulator.x, body->force_accumulator.y, body->force_accumulator.z);
    
    rigid_body_destroy(body);
    printf("Basic tests completed successfully!\n");
}

int main(void) {
    printf("Charvak Physics Engine Demo\n");
    printf("============================\n");
    
    // Seed random number generator
    srand((unsigned int)time(NULL));
    
    // Run tests and demos
    run_basic_tests();
    demo_bouncing_spheres();
    demo_sphere_box_collision();
    
    printf("\nAll demos completed successfully!\n");
    return 0;
}