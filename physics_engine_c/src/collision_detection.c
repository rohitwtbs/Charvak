#include "../include/collision_detection.h"
#include <float.h>

bool detect_collision(RigidBody* body_a, RigidBody* body_b, CollisionInfo* info) {
    if (!body_a || !body_b || !info) return false;
    
    // Initialize collision info
    info->has_collision = false;
    info->body_a = body_a;
    info->body_b = body_b;
    
    // Quick broad-phase check
    if (!aabb_overlap_test(body_a, body_b)) {
        return false;
    }
    
    // Dispatch to specific collision detection based on shape types
    if (body_a->shape_type == SHAPE_SPHERE && body_b->shape_type == SHAPE_SPHERE) {
        return sphere_sphere_collision(body_a, body_b, info);
    }
    else if (body_a->shape_type == SHAPE_SPHERE && body_b->shape_type == SHAPE_AABB) {
        return sphere_aabb_collision(body_a, body_b, info);
    }
    else if (body_a->shape_type == SHAPE_AABB && body_b->shape_type == SHAPE_SPHERE) {
        return sphere_aabb_collision(body_b, body_a, info);
    }
    else if (body_a->shape_type == SHAPE_AABB && body_b->shape_type == SHAPE_AABB) {
        return aabb_aabb_collision(body_a, body_b, info);
    }
    else if (body_a->shape_type == SHAPE_SPHERE && body_b->shape_type == SHAPE_PLANE) {
        return sphere_plane_collision(body_a, body_b, info);
    }
    else if (body_a->shape_type == SHAPE_PLANE && body_b->shape_type == SHAPE_SPHERE) {
        return sphere_plane_collision(body_b, body_a, info);
    }
    else if (body_a->shape_type == SHAPE_AABB && body_b->shape_type == SHAPE_PLANE) {
        return aabb_plane_collision(body_a, body_b, info);
    }
    else if (body_a->shape_type == SHAPE_PLANE && body_b->shape_type == SHAPE_AABB) {
        return aabb_plane_collision(body_b, body_a, info);
    }
    
    return false;
}

bool sphere_sphere_collision(RigidBody* sphere_a, RigidBody* sphere_b, CollisionInfo* info) {
    float radius_a = sphere_a->shape.sphere.radius;
    float radius_b = sphere_b->shape.sphere.radius;
    
    Vector3 center_to_center = vector3_subtract(sphere_b->position, sphere_a->position);
    float distance = vector3_length(center_to_center);
    float combined_radius = radius_a + radius_b;
    
    if (distance < combined_radius) {
        info->has_collision = true;
        info->penetration_depth = combined_radius - distance;
        
        if (distance > VECTOR_EPSILON) {
            info->normal = vector3_normalize(center_to_center);
        } else {
            // Spheres are at same position, choose arbitrary normal
            info->normal = vector3_create(1.0f, 0.0f, 0.0f);
        }
        
        // Contact point is on the surface of sphere A
        Vector3 contact_offset = vector3_scale(info->normal, radius_a - info->penetration_depth * 0.5f);
        info->contact_point = vector3_add(sphere_a->position, contact_offset);
        
        return true;
    }
    
    return false;
}

bool sphere_aabb_collision(RigidBody* sphere, RigidBody* aabb, CollisionInfo* info) {
    Vector3 closest_point = closest_point_on_aabb(sphere->position, aabb);
    Vector3 sphere_to_closest = vector3_subtract(closest_point, sphere->position);
    float distance = vector3_length(sphere_to_closest);
    
    if (distance < sphere->shape.sphere.radius) {
        info->has_collision = true;
        info->penetration_depth = sphere->shape.sphere.radius - distance;
        info->contact_point = closest_point;
        
        if (distance > VECTOR_EPSILON) {
            info->normal = vector3_normalize(vector3_negate(sphere_to_closest));
        } else {
            // Sphere center is inside AABB, find the closest face
            Vector3 aabb_center = aabb->position;
            Vector3 to_sphere = vector3_subtract(sphere->position, aabb_center);
            Vector3 half_extents = aabb->shape.aabb.half_extents;
            
            // Find the axis with minimum penetration
            float min_penetration = FLT_MAX;
            Vector3 min_normal = vector3_create(1.0f, 0.0f, 0.0f);
            
            // Check X axis
            float x_penetration = half_extents.x - fabsf(to_sphere.x);
            if (x_penetration < min_penetration) {
                min_penetration = x_penetration;
                min_normal = vector3_create(to_sphere.x > 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
            }
            
            // Check Y axis
            float y_penetration = half_extents.y - fabsf(to_sphere.y);
            if (y_penetration < min_penetration) {
                min_penetration = y_penetration;
                min_normal = vector3_create(0.0f, to_sphere.y > 0 ? 1.0f : -1.0f, 0.0f);
            }
            
            // Check Z axis
            float z_penetration = half_extents.z - fabsf(to_sphere.z);
            if (z_penetration < min_penetration) {
                min_penetration = z_penetration;
                min_normal = vector3_create(0.0f, 0.0f, to_sphere.z > 0 ? 1.0f : -1.0f);
            }
            
            info->normal = min_normal;
        }
        
        return true;
    }
    
    return false;
}

bool aabb_aabb_collision(RigidBody* aabb_a, RigidBody* aabb_b, CollisionInfo* info) {
    Vector3 min_a = get_aabb_min(aabb_a);
    Vector3 max_a = get_aabb_max(aabb_a);
    Vector3 min_b = get_aabb_min(aabb_b);
    Vector3 max_b = get_aabb_max(aabb_b);
    
    // Check for overlap on all axes
    bool overlap_x = (min_a.x <= max_b.x) && (max_a.x >= min_b.x);
    bool overlap_y = (min_a.y <= max_b.y) && (max_a.y >= min_b.y);
    bool overlap_z = (min_a.z <= max_b.z) && (max_a.z >= min_b.z);
    
    if (overlap_x && overlap_y && overlap_z) {
        info->has_collision = true;
        
        // Calculate penetration on each axis
        float x_penetration = fminf(max_a.x - min_b.x, max_b.x - min_a.x);
        float y_penetration = fminf(max_a.y - min_b.y, max_b.y - min_a.y);
        float z_penetration = fminf(max_a.z - min_b.z, max_b.z - min_a.z);
        
        // Find the axis with minimum penetration (separation axis)
        if (x_penetration < y_penetration && x_penetration < z_penetration) {
            info->penetration_depth = x_penetration;
            info->normal = vector3_create(aabb_a->position.x < aabb_b->position.x ? -1.0f : 1.0f, 0.0f, 0.0f);
        } else if (y_penetration < z_penetration) {
            info->penetration_depth = y_penetration;
            info->normal = vector3_create(0.0f, aabb_a->position.y < aabb_b->position.y ? -1.0f : 1.0f, 0.0f);
        } else {
            info->penetration_depth = z_penetration;
            info->normal = vector3_create(0.0f, 0.0f, aabb_a->position.z < aabb_b->position.z ? -1.0f : 1.0f);
        }
        
        // Calculate contact point (center of overlap region)
        Vector3 overlap_min = vector3_create(
            fmaxf(min_a.x, min_b.x),
            fmaxf(min_a.y, min_b.y),
            fmaxf(min_a.z, min_b.z)
        );
        Vector3 overlap_max = vector3_create(
            fminf(max_a.x, max_b.x),
            fminf(max_a.y, max_b.y),
            fminf(max_a.z, max_b.z)
        );
        info->contact_point = vector3_scale(vector3_add(overlap_min, overlap_max), 0.5f);
        
        return true;
    }
    
    return false;
}

bool sphere_plane_collision(RigidBody* sphere, RigidBody* plane, CollisionInfo* info) {
    float distance = distance_to_plane(sphere->position, plane);
    float radius = sphere->shape.sphere.radius;
    
    if (distance < radius) {
        info->has_collision = true;
        info->penetration_depth = radius - distance;
        info->normal = plane->shape.plane.normal;
        
        // Contact point is on the sphere surface closest to the plane
        Vector3 contact_offset = vector3_scale(info->normal, -radius);
        info->contact_point = vector3_add(sphere->position, contact_offset);
        
        return true;
    }
    
    return false;
}

bool aabb_plane_collision(RigidBody* aabb, RigidBody* plane, CollisionInfo* info) {
    Vector3 half_extents = aabb->shape.aabb.half_extents;
    Vector3 plane_normal = plane->shape.plane.normal;
    
    // Calculate the extent of the AABB along the plane normal
    float extent = fabsf(half_extents.x * plane_normal.x) +
                   fabsf(half_extents.y * plane_normal.y) +
                   fabsf(half_extents.z * plane_normal.z);
    
    float distance = distance_to_plane(aabb->position, plane);
    
    if (distance < extent) {
        info->has_collision = true;
        info->penetration_depth = extent - distance;
        info->normal = plane_normal;
        
        // Contact point is the closest point on the AABB to the plane
        Vector3 contact_offset = vector3_scale(plane_normal, -distance);
        info->contact_point = vector3_add(aabb->position, contact_offset);
        
        return true;
    }
    
    return false;
}

Vector3 closest_point_on_aabb(Vector3 point, RigidBody* aabb) {
    Vector3 min = get_aabb_min(aabb);
    Vector3 max = get_aabb_max(aabb);
    
    Vector3 closest;
    closest.x = fmaxf(min.x, fminf(point.x, max.x));
    closest.y = fmaxf(min.y, fminf(point.y, max.y));
    closest.z = fmaxf(min.z, fminf(point.z, max.z));
    
    return closest;
}

float distance_to_plane(Vector3 point, RigidBody* plane) {
    Vector3 normal = plane->shape.plane.normal;
    float d = plane->shape.plane.distance;
    return vector3_dot(point, normal) - d;
}

bool point_in_aabb(Vector3 point, RigidBody* aabb) {
    Vector3 min = get_aabb_min(aabb);
    Vector3 max = get_aabb_max(aabb);
    
    return (point.x >= min.x && point.x <= max.x) &&
           (point.y >= min.y && point.y <= max.y) &&
           (point.z >= min.z && point.z <= max.z);
}

bool aabb_overlap_test(RigidBody* body_a, RigidBody* body_b) {
    Vector3 min_a = get_aabb_min(body_a);
    Vector3 max_a = get_aabb_max(body_a);
    Vector3 min_b = get_aabb_min(body_b);
    Vector3 max_b = get_aabb_max(body_b);
    
    return (min_a.x <= max_b.x && max_a.x >= min_b.x) &&
           (min_a.y <= max_b.y && max_a.y >= min_b.y) &&
           (min_a.z <= max_b.z && max_a.z >= min_b.z);
}

Vector3 get_aabb_min(RigidBody* body) {
    if (body->shape_type == SHAPE_SPHERE) {
        float radius = body->shape.sphere.radius;
        return vector3_create(
            body->position.x - radius,
            body->position.y - radius,
            body->position.z - radius
        );
    } else if (body->shape_type == SHAPE_AABB) {
        Vector3 half_extents = body->shape.aabb.half_extents;
        return vector3_create(
            body->position.x - half_extents.x,
            body->position.y - half_extents.y,
            body->position.z - half_extents.z
        );
    } else {
        // For planes, return a very large negative value
        return vector3_create(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
}

Vector3 get_aabb_max(RigidBody* body) {
    if (body->shape_type == SHAPE_SPHERE) {
        float radius = body->shape.sphere.radius;
        return vector3_create(
            body->position.x + radius,
            body->position.y + radius,
            body->position.z + radius
        );
    } else if (body->shape_type == SHAPE_AABB) {
        Vector3 half_extents = body->shape.aabb.half_extents;
        return vector3_create(
            body->position.x + half_extents.x,
            body->position.y + half_extents.y,
            body->position.z + half_extents.z
        );
    } else {
        // For planes, return a very large positive value
        return vector3_create(FLT_MAX, FLT_MAX, FLT_MAX);
    }
}