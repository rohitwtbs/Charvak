#include "../include/collision_response.h"

void resolve_collision(CollisionInfo* collision) {
    if (!collision || !collision->has_collision) return;
    
    // First, separate the bodies to prevent overlap
    separate_bodies(collision);
    
    // Then apply impulse response to handle velocities
    apply_impulse_response(collision);
    
    // Apply friction
    apply_friction(collision);
    
    // Apply position correction to prevent floating point drift
    position_correction(collision, 0.8f, 0.01f);
}

void separate_bodies(CollisionInfo* collision) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return;
    
    float total_inverse_mass = body_a->inverse_mass + body_b->inverse_mass;
    if (total_inverse_mass <= 0.0f) return;  // Both bodies are static
    
    // Calculate separation based on inverse mass ratio
    float separation_a = body_a->inverse_mass / total_inverse_mass;
    float separation_b = body_b->inverse_mass / total_inverse_mass;
    
    Vector3 separation_vector = vector3_scale(collision->normal, collision->penetration_depth);
    
    // Move bodies apart
    if (!body_a->is_static) {
        Vector3 move_a = vector3_scale(separation_vector, -separation_a);
        body_a->position = vector3_add(body_a->position, move_a);
    }
    
    if (!body_b->is_static) {
        Vector3 move_b = vector3_scale(separation_vector, separation_b);
        body_b->position = vector3_add(body_b->position, move_b);
    }
}

void apply_impulse_response(CollisionInfo* collision) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return;
    
    float relative_velocity = calculate_relative_velocity(collision);
    
    // Don't resolve if velocities are separating
    if (relative_velocity > 0.0f) return;
    
    // Calculate restitution (combine restitution of both bodies)
    float restitution = fminf(body_a->restitution, body_b->restitution);
    
    // Calculate impulse magnitude
    float impulse_magnitude = calculate_impulse_magnitude(collision, restitution);
    
    // Apply impulse
    Vector3 impulse = vector3_scale(collision->normal, impulse_magnitude);
    
    if (!body_a->is_static) {
        Vector3 impulse_a = vector3_scale(impulse, -body_a->inverse_mass);
        body_a->velocity = vector3_add(body_a->velocity, impulse_a);
    }
    
    if (!body_b->is_static) {
        Vector3 impulse_b = vector3_scale(impulse, body_b->inverse_mass);
        body_b->velocity = vector3_add(body_b->velocity, impulse_b);
    }
}

void apply_friction(CollisionInfo* collision) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return;
    
    // Calculate relative velocity at contact point
    Vector3 relative_velocity_vec = vector3_subtract(body_b->velocity, body_a->velocity);
    
    // Calculate tangent vector (perpendicular to normal in the plane of contact)
    Vector3 normal = collision->normal;
    float relative_velocity_normal = vector3_dot(relative_velocity_vec, normal);
    Vector3 tangent = vector3_subtract(relative_velocity_vec, vector3_scale(normal, relative_velocity_normal));
    
    float tangent_length = vector3_length(tangent);
    if (tangent_length < VECTOR_EPSILON) return;  // No tangential motion
    
    tangent = vector3_normalize(tangent);
    
    // Calculate friction coefficient (combine friction of both bodies)
    float friction_coefficient = sqrtf(body_a->friction * body_b->friction);
    
    // Calculate friction impulse magnitude
    float total_inverse_mass = body_a->inverse_mass + body_b->inverse_mass;
    if (total_inverse_mass <= 0.0f) return;
    
    float friction_impulse_magnitude = -vector3_dot(relative_velocity_vec, tangent) / total_inverse_mass;
    
    // Clamp friction impulse to Coulomb friction model
    float normal_impulse_magnitude = calculate_impulse_magnitude(collision, 0.0f);  // No restitution for friction calculation
    float max_friction_impulse = friction_coefficient * fabsf(normal_impulse_magnitude);
    
    if (fabsf(friction_impulse_magnitude) > max_friction_impulse) {
        friction_impulse_magnitude = copysignf(max_friction_impulse, friction_impulse_magnitude);
    }
    
    Vector3 friction_impulse = vector3_scale(tangent, friction_impulse_magnitude);
    
    // Apply friction impulse
    if (!body_a->is_static) {
        Vector3 friction_a = vector3_scale(friction_impulse, -body_a->inverse_mass);
        body_a->velocity = vector3_add(body_a->velocity, friction_a);
    }
    
    if (!body_b->is_static) {
        Vector3 friction_b = vector3_scale(friction_impulse, body_b->inverse_mass);
        body_b->velocity = vector3_add(body_b->velocity, friction_b);
    }
}

float calculate_relative_velocity(CollisionInfo* collision) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return 0.0f;
    
    Vector3 relative_velocity = vector3_subtract(body_b->velocity, body_a->velocity);
    return vector3_dot(relative_velocity, collision->normal);
}

float calculate_impulse_magnitude(CollisionInfo* collision, float restitution) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return 0.0f;
    
    float relative_velocity = calculate_relative_velocity(collision);
    float total_inverse_mass = body_a->inverse_mass + body_b->inverse_mass;
    
    if (total_inverse_mass <= 0.0f) return 0.0f;  // Both bodies are static
    
    // Impulse magnitude = -(1 + e) * relative_velocity / total_inverse_mass
    float impulse_magnitude = -(1.0f + restitution) * relative_velocity / total_inverse_mass;
    
    return impulse_magnitude;
}

Vector3 calculate_friction_impulse(CollisionInfo* collision, float friction_coefficient) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return vector3_zero();
    
    Vector3 relative_velocity = vector3_subtract(body_b->velocity, body_a->velocity);
    Vector3 normal = collision->normal;
    
    // Calculate tangential velocity
    float normal_velocity = vector3_dot(relative_velocity, normal);
    Vector3 tangential_velocity = vector3_subtract(relative_velocity, vector3_scale(normal, normal_velocity));
    
    float tangential_speed = vector3_length(tangential_velocity);
    if (tangential_speed < VECTOR_EPSILON) return vector3_zero();
    
    Vector3 tangent = vector3_normalize(tangential_velocity);
    
    // Calculate friction impulse
    float total_inverse_mass = body_a->inverse_mass + body_b->inverse_mass;
    if (total_inverse_mass <= 0.0f) return vector3_zero();
    
    float friction_impulse_magnitude = -tangential_speed / total_inverse_mass;
    
    // Apply Coulomb friction limit
    float normal_impulse = fabsf(calculate_impulse_magnitude(collision, 0.0f));
    float max_friction = friction_coefficient * normal_impulse;
    
    if (fabsf(friction_impulse_magnitude) > max_friction) {
        friction_impulse_magnitude = copysignf(max_friction, friction_impulse_magnitude);
    }
    
    return vector3_scale(tangent, friction_impulse_magnitude);
}

void position_correction(CollisionInfo* collision, float correction_percentage, float slop) {
    RigidBody* body_a = collision->body_a;
    RigidBody* body_b = collision->body_b;
    
    if (!body_a || !body_b) return;
    
    float total_inverse_mass = body_a->inverse_mass + body_b->inverse_mass;
    if (total_inverse_mass <= 0.0f) return;
    
    // Only apply correction if penetration is significant
    float penetration = collision->penetration_depth - slop;
    if (penetration <= 0.0f) return;
    
    float correction_magnitude = penetration * correction_percentage / total_inverse_mass;
    Vector3 correction = vector3_scale(collision->normal, correction_magnitude);
    
    if (!body_a->is_static) {
        Vector3 correction_a = vector3_scale(correction, -body_a->inverse_mass);
        body_a->position = vector3_add(body_a->position, correction_a);
    }
    
    if (!body_b->is_static) {
        Vector3 correction_b = vector3_scale(correction, body_b->inverse_mass);
        body_b->position = vector3_add(body_b->position, correction_b);
    }
}