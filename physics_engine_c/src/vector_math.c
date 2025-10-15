#include "../include/vector_math.h"

// Vector2 implementations
Vector2 vector2_create(float x, float y) {
    return (Vector2){x, y};
}

Vector2 vector2_zero(void) {
    return (Vector2){0.0f, 0.0f};
}

Vector2 vector2_add(Vector2 a, Vector2 b) {
    return (Vector2){a.x + b.x, a.y + b.y};
}

Vector2 vector2_subtract(Vector2 a, Vector2 b) {
    return (Vector2){a.x - b.x, a.y - b.y};
}

Vector2 vector2_scale(Vector2 v, float scalar) {
    return (Vector2){v.x * scalar, v.y * scalar};
}

Vector2 vector2_negate(Vector2 v) {
    return (Vector2){-v.x, -v.y};
}

float vector2_dot(Vector2 a, Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

float vector2_cross(Vector2 a, Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

float vector2_length_squared(Vector2 v) {
    return v.x * v.x + v.y * v.y;
}

float vector2_length(Vector2 v) {
    return sqrtf(vector2_length_squared(v));
}

Vector2 vector2_normalize(Vector2 v) {
    float len = vector2_length(v);
    if (len < VECTOR_EPSILON) {
        return vector2_zero();
    }
    return vector2_scale(v, 1.0f / len);
}

float vector2_distance(Vector2 a, Vector2 b) {
    return vector2_length(vector2_subtract(b, a));
}

bool vector2_equals(Vector2 a, Vector2 b, float epsilon) {
    return fabsf(a.x - b.x) < epsilon && fabsf(a.y - b.y) < epsilon;
}

// Vector3 implementations
Vector3 vector3_create(float x, float y, float z) {
    return (Vector3){x, y, z};
}

Vector3 vector3_zero(void) {
    return (Vector3){0.0f, 0.0f, 0.0f};
}

Vector3 vector3_add(Vector3 a, Vector3 b) {
    return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 vector3_subtract(Vector3 a, Vector3 b) {
    return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 vector3_scale(Vector3 v, float scalar) {
    return (Vector3){v.x * scalar, v.y * scalar, v.z * scalar};
}

Vector3 vector3_negate(Vector3 v) {
    return (Vector3){-v.x, -v.y, -v.z};
}

float vector3_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 vector3_cross(Vector3 a, Vector3 b) {
    return (Vector3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float vector3_length_squared(Vector3 v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

float vector3_length(Vector3 v) {
    return sqrtf(vector3_length_squared(v));
}

Vector3 vector3_normalize(Vector3 v) {
    float len = vector3_length(v);
    if (len < VECTOR_EPSILON) {
        return vector3_zero();
    }
    return vector3_scale(v, 1.0f / len);
}

float vector3_distance(Vector3 a, Vector3 b) {
    return vector3_length(vector3_subtract(b, a));
}

bool vector3_equals(Vector3 a, Vector3 b, float epsilon) {
    return fabsf(a.x - b.x) < epsilon && 
           fabsf(a.y - b.y) < epsilon && 
           fabsf(a.z - b.z) < epsilon;
}