#include "../include/integration.h"

void update_acceleration(RigidBody* body) {
    if (!body || body->is_static) return;
    
    // Calculate linear acceleration from forces: a = F/m
    body->acceleration = vector3_scale(body->force_accumulator, body->inverse_mass);
    
    // For angular acceleration, we'd need the inertia tensor
    // For now, use a simplified approach
    body->angular_acceleration = vector3_scale(body->torque_accumulator, body->inverse_mass);
}

void integrate_euler(RigidBody* body, float dt) {
    if (!body || body->is_static || body->is_sleeping) return;
    
    // Update acceleration from accumulated forces
    update_acceleration(body);
    
    // Integrate velocity: v = v + a * dt
    Vector3 velocity_change = vector3_scale(body->acceleration, dt);
    body->velocity = vector3_add(body->velocity, velocity_change);
    
    Vector3 angular_velocity_change = vector3_scale(body->angular_acceleration, dt);
    body->angular_velocity = vector3_add(body->angular_velocity, angular_velocity_change);
    
    // Integrate position: p = p + v * dt
    Vector3 position_change = vector3_scale(body->velocity, dt);
    body->position = vector3_add(body->position, position_change);
    
    Vector3 rotation_change = vector3_scale(body->angular_velocity, dt);
    body->rotation = vector3_add(body->rotation, rotation_change);
    
    // Clear force accumulators for next frame
    rigid_body_clear_forces(body);
}

void integrate_verlet(RigidBody* body, float dt) {
    if (!body || body->is_static || body->is_sleeping) return;
    
    // Store previous acceleration
    Vector3 prev_acceleration = body->acceleration;
    Vector3 prev_angular_acceleration = body->angular_acceleration;
    
    // Update acceleration from current forces
    update_acceleration(body);
    
    // Verlet integration for position: 
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
    Vector3 velocity_term = vector3_scale(body->velocity, dt);
    Vector3 acceleration_term = vector3_scale(body->acceleration, 0.5f * dt * dt);
    body->position = vector3_add(body->position, vector3_add(velocity_term, acceleration_term));
    
    // Verlet integration for rotation
    Vector3 angular_velocity_term = vector3_scale(body->angular_velocity, dt);
    Vector3 angular_acceleration_term = vector3_scale(body->angular_acceleration, 0.5f * dt * dt);
    body->rotation = vector3_add(body->rotation, vector3_add(angular_velocity_term, angular_acceleration_term));
    
    // Update velocity using average of previous and current acceleration:
    // v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
    Vector3 avg_acceleration = vector3_scale(vector3_add(prev_acceleration, body->acceleration), 0.5f);
    Vector3 velocity_change = vector3_scale(avg_acceleration, dt);
    body->velocity = vector3_add(body->velocity, velocity_change);
    
    Vector3 avg_angular_acceleration = vector3_scale(vector3_add(prev_angular_acceleration, body->angular_acceleration), 0.5f);
    Vector3 angular_velocity_change = vector3_scale(avg_angular_acceleration, dt);
    body->angular_velocity = vector3_add(body->angular_velocity, angular_velocity_change);
    
    // Clear force accumulators
    rigid_body_clear_forces(body);
}

void integrate_rk4(RigidBody* body, float dt) {
    if (!body || body->is_static || body->is_sleeping) return;
    
    // RK4 is more complex and computationally expensive
    // For a physics engine, Verlet is usually preferred
    // This is a simplified RK4 implementation
    
    Vector3 initial_pos = body->position;
    Vector3 initial_vel = body->velocity;
    Vector3 initial_rot = body->rotation;
    Vector3 initial_ang_vel = body->angular_velocity;
    
    // k1
    update_acceleration(body);
    Vector3 k1_vel = body->acceleration;
    Vector3 k1_pos = body->velocity;
    Vector3 k1_ang_vel = body->angular_acceleration;
    Vector3 k1_rot = body->angular_velocity;
    
    // k2
    body->position = vector3_add(initial_pos, vector3_scale(k1_pos, dt * 0.5f));
    body->velocity = vector3_add(initial_vel, vector3_scale(k1_vel, dt * 0.5f));
    body->rotation = vector3_add(initial_rot, vector3_scale(k1_rot, dt * 0.5f));
    body->angular_velocity = vector3_add(initial_ang_vel, vector3_scale(k1_ang_vel, dt * 0.5f));
    
    update_acceleration(body);
    Vector3 k2_vel = body->acceleration;
    Vector3 k2_pos = body->velocity;
    Vector3 k2_ang_vel = body->angular_acceleration;
    Vector3 k2_rot = body->angular_velocity;
    
    // k3
    body->position = vector3_add(initial_pos, vector3_scale(k2_pos, dt * 0.5f));
    body->velocity = vector3_add(initial_vel, vector3_scale(k2_vel, dt * 0.5f));
    body->rotation = vector3_add(initial_rot, vector3_scale(k2_rot, dt * 0.5f));
    body->angular_velocity = vector3_add(initial_ang_vel, vector3_scale(k2_ang_vel, dt * 0.5f));
    
    update_acceleration(body);
    Vector3 k3_vel = body->acceleration;
    Vector3 k3_pos = body->velocity;
    Vector3 k3_ang_vel = body->angular_acceleration;
    Vector3 k3_rot = body->angular_velocity;
    
    // k4
    body->position = vector3_add(initial_pos, vector3_scale(k3_pos, dt));
    body->velocity = vector3_add(initial_vel, vector3_scale(k3_vel, dt));
    body->rotation = vector3_add(initial_rot, vector3_scale(k3_rot, dt));
    body->angular_velocity = vector3_add(initial_ang_vel, vector3_scale(k3_ang_vel, dt));
    
    update_acceleration(body);
    Vector3 k4_vel = body->acceleration;
    Vector3 k4_pos = body->velocity;
    Vector3 k4_ang_vel = body->angular_acceleration;
    Vector3 k4_rot = body->angular_velocity;
    
    // Final update
    Vector3 vel_change = vector3_scale(vector3_add(vector3_add(k1_vel, vector3_scale(k2_vel, 2.0f)), 
                                                  vector3_add(vector3_scale(k3_vel, 2.0f), k4_vel)), dt / 6.0f);
    Vector3 pos_change = vector3_scale(vector3_add(vector3_add(k1_pos, vector3_scale(k2_pos, 2.0f)), 
                                                  vector3_add(vector3_scale(k3_pos, 2.0f), k4_pos)), dt / 6.0f);
    Vector3 ang_vel_change = vector3_scale(vector3_add(vector3_add(k1_ang_vel, vector3_scale(k2_ang_vel, 2.0f)), 
                                                      vector3_add(vector3_scale(k3_ang_vel, 2.0f), k4_ang_vel)), dt / 6.0f);
    Vector3 rot_change = vector3_scale(vector3_add(vector3_add(k1_rot, vector3_scale(k2_rot, 2.0f)), 
                                                  vector3_add(vector3_scale(k3_rot, 2.0f), k4_rot)), dt / 6.0f);
    
    body->position = vector3_add(initial_pos, pos_change);
    body->velocity = vector3_add(initial_vel, vel_change);
    body->rotation = vector3_add(initial_rot, rot_change);
    body->angular_velocity = vector3_add(initial_ang_vel, ang_vel_change);
    
    // Clear force accumulators
    rigid_body_clear_forces(body);
}

void integrate_body(RigidBody* body, float dt, IntegrationMethod method) {
    switch (method) {
        case INTEGRATION_EULER:
            integrate_euler(body, dt);
            break;
        case INTEGRATION_VERLET:
            integrate_verlet(body, dt);
            break;
        case INTEGRATION_RK4:
            integrate_rk4(body, dt);
            break;
        default:
            integrate_verlet(body, dt);  // Default to Verlet
            break;
    }
}

void apply_damping(RigidBody* body, float linear_damping, float angular_damping) {
    if (!body || body->is_static) return;
    
    // Apply linear damping: v = v * (1 - damping)
    body->velocity = vector3_scale(body->velocity, 1.0f - linear_damping);
    
    // Apply angular damping
    body->angular_velocity = vector3_scale(body->angular_velocity, 1.0f - angular_damping);
    
    // Put body to sleep if velocity is very low
    float linear_speed_sq = vector3_length_squared(body->velocity);
    float angular_speed_sq = vector3_length_squared(body->angular_velocity);
    
    const float sleep_threshold = 0.01f;
    if (linear_speed_sq < sleep_threshold && angular_speed_sq < sleep_threshold) {
        body->is_sleeping = true;
        body->velocity = vector3_zero();
        body->angular_velocity = vector3_zero();
    }
}