#include "../include/rigid_body.h"
#include <stdlib.h>
#include <string.h>

static int next_body_id = 1;

RigidBody* rigid_body_create(void) {
    RigidBody* body = (RigidBody*)malloc(sizeof(RigidBody));
    if (!body) return NULL;
    
    // Initialize all fields to zero/default values
    memset(body, 0, sizeof(RigidBody));
    
    // Set default values
    body->mass = 1.0f;
    body->inverse_mass = 1.0f;
    body->restitution = 0.5f;
    body->friction = 0.3f;
    body->is_static = false;
    body->is_sleeping = false;
    body->id = next_body_id++;
    
    return body;
}

void rigid_body_destroy(RigidBody* body) {
    if (body) {
        free(body);
    }
}

void rigid_body_init_sphere(RigidBody* body, Vector3 position, float radius, float mass) {
    if (!body) return;
    
    body->position = position;
    body->shape_type = SHAPE_SPHERE;
    body->shape.sphere.radius = radius;
    rigid_body_set_mass(body, mass);
}

void rigid_body_init_aabb(RigidBody* body, Vector3 position, Vector3 half_extents, float mass) {
    if (!body) return;
    
    body->position = position;
    body->shape_type = SHAPE_AABB;
    body->shape.aabb.half_extents = half_extents;
    rigid_body_set_mass(body, mass);
}

void rigid_body_init_plane(RigidBody* body, Vector3 normal, float distance) {
    if (!body) return;
    
    body->position = vector3_zero();
    body->shape_type = SHAPE_PLANE;
    body->shape.plane.normal = vector3_normalize(normal);
    body->shape.plane.distance = distance;
    
    // Planes are always static and have infinite mass
    body->is_static = true;
    body->mass = INFINITY;
    body->inverse_mass = 0.0f;
}

void rigid_body_set_position(RigidBody* body, Vector3 position) {
    if (body && !body->is_static) {
        body->position = position;
    }
}

void rigid_body_set_velocity(RigidBody* body, Vector3 velocity) {
    if (body && !body->is_static) {
        body->velocity = velocity;
    }
}

void rigid_body_set_mass(RigidBody* body, float mass) {
    if (!body) return;
    
    if (mass <= 0.0f || body->is_static) {
        body->mass = INFINITY;
        body->inverse_mass = 0.0f;
    } else {
        body->mass = mass;
        body->inverse_mass = 1.0f / mass;
    }
}

void rigid_body_set_restitution(RigidBody* body, float restitution) {
    if (body) {
        body->restitution = fmaxf(0.0f, fminf(1.0f, restitution));
    }
}

void rigid_body_set_friction(RigidBody* body, float friction) {
    if (body) {
        body->friction = fmaxf(0.0f, friction);
    }
}

void rigid_body_set_static(RigidBody* body, bool is_static) {
    if (!body) return;
    
    body->is_static = is_static;
    if (is_static) {
        body->velocity = vector3_zero();
        body->angular_velocity = vector3_zero();
        body->mass = INFINITY;
        body->inverse_mass = 0.0f;
    } else {
        rigid_body_set_mass(body, body->mass);
    }
}

void rigid_body_add_force(RigidBody* body, Vector3 force) {
    if (body && !body->is_static) {
        body->force_accumulator = vector3_add(body->force_accumulator, force);
    }
}

void rigid_body_add_force_at_point(RigidBody* body, Vector3 force, Vector3 point) {
    if (!body || body->is_static) return;
    
    // Add the force
    rigid_body_add_force(body, force);
    
    // Calculate torque from the offset
    Vector3 offset = vector3_subtract(point, body->position);
    Vector3 torque = vector3_cross(offset, force);
    rigid_body_add_torque(body, torque);
}

void rigid_body_add_torque(RigidBody* body, Vector3 torque) {
    if (body && !body->is_static) {
        body->torque_accumulator = vector3_add(body->torque_accumulator, torque);
    }
}

void rigid_body_add_impulse(RigidBody* body, Vector3 impulse) {
    if (body && !body->is_static) {
        Vector3 velocity_change = vector3_scale(impulse, body->inverse_mass);
        body->velocity = vector3_add(body->velocity, velocity_change);
    }
}

void rigid_body_clear_forces(RigidBody* body) {
    if (body) {
        body->force_accumulator = vector3_zero();
        body->torque_accumulator = vector3_zero();
    }
}

Vector3 rigid_body_get_point_velocity(RigidBody* body, Vector3 point) {
    if (!body) return vector3_zero();
    
    Vector3 offset = vector3_subtract(point, body->position);
    Vector3 rotational_velocity = vector3_cross(body->angular_velocity, offset);
    return vector3_add(body->velocity, rotational_velocity);
}

float rigid_body_get_kinetic_energy(RigidBody* body) {
    if (!body) return 0.0f;
    
    float linear_ke = 0.5f * body->mass * vector3_length_squared(body->velocity);
    // For angular kinetic energy, we'd need moment of inertia tensor
    // For now, just return linear kinetic energy
    return linear_ke;
}