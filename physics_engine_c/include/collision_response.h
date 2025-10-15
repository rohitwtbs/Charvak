#ifndef COLLISION_RESPONSE_H
#define COLLISION_RESPONSE_H

#include "collision_detection.h"

// Collision response functions
void resolve_collision(CollisionInfo* collision);
void separate_bodies(CollisionInfo* collision);
void apply_impulse_response(CollisionInfo* collision);
void apply_friction(CollisionInfo* collision);

// Utility functions for collision response
float calculate_relative_velocity(CollisionInfo* collision);
float calculate_impulse_magnitude(CollisionInfo* collision, float restitution);
Vector3 calculate_friction_impulse(CollisionInfo* collision, float friction_coefficient);

// Position correction to prevent sinking
void position_correction(CollisionInfo* collision, float correction_percentage, float slop);

#endif // COLLISION_RESPONSE_H