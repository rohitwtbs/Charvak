#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "rigid_body.h"
#include <stdbool.h>

// Collision information structure
typedef struct {
    bool has_collision;
    Vector3 contact_point;
    Vector3 normal;           // Normal pointing from body A to body B
    float penetration_depth;
    RigidBody* body_a;
    RigidBody* body_b;
} CollisionInfo;

// Main collision detection function
bool detect_collision(RigidBody* body_a, RigidBody* body_b, CollisionInfo* info);

// Specific collision detection functions
bool sphere_sphere_collision(RigidBody* sphere_a, RigidBody* sphere_b, CollisionInfo* info);
bool sphere_aabb_collision(RigidBody* sphere, RigidBody* aabb, CollisionInfo* info);
bool aabb_aabb_collision(RigidBody* aabb_a, RigidBody* aabb_b, CollisionInfo* info);
bool sphere_plane_collision(RigidBody* sphere, RigidBody* plane, CollisionInfo* info);
bool aabb_plane_collision(RigidBody* aabb, RigidBody* plane, CollisionInfo* info);

// Utility functions
Vector3 closest_point_on_aabb(Vector3 point, RigidBody* aabb);
float distance_to_plane(Vector3 point, RigidBody* plane);
bool point_in_aabb(Vector3 point, RigidBody* aabb);

// Broad phase collision detection (for optimization)
bool aabb_overlap_test(RigidBody* body_a, RigidBody* body_b);
Vector3 get_aabb_min(RigidBody* body);
Vector3 get_aabb_max(RigidBody* body);

#endif // COLLISION_DETECTION_H