#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "rigid_body.h"

// Integration methods
typedef enum {
    INTEGRATION_EULER,
    INTEGRATION_VERLET,
    INTEGRATION_RK4
} IntegrationMethod;

// Integration functions
void integrate_euler(RigidBody* body, float dt);
void integrate_verlet(RigidBody* body, float dt);
void integrate_rk4(RigidBody* body, float dt);

// Main integration dispatcher
void integrate_body(RigidBody* body, float dt, IntegrationMethod method);

// Utility functions for integration
void update_acceleration(RigidBody* body);
void apply_damping(RigidBody* body, float linear_damping, float angular_damping);

#endif // INTEGRATION_H