#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <math.h>
#include <stdbool.h>

// 2D Vector structure
typedef struct {
    float x, y;
} Vector2;

// 3D Vector structure  
typedef struct {
    float x, y, z;
} Vector3;

// Vector2 operations
Vector2 vector2_create(float x, float y);
Vector2 vector2_zero(void);
Vector2 vector2_add(Vector2 a, Vector2 b);
Vector2 vector2_subtract(Vector2 a, Vector2 b);
Vector2 vector2_scale(Vector2 v, float scalar);
Vector2 vector2_negate(Vector2 v);
float vector2_dot(Vector2 a, Vector2 b);
float vector2_cross(Vector2 a, Vector2 b);
float vector2_length_squared(Vector2 v);
float vector2_length(Vector2 v);
Vector2 vector2_normalize(Vector2 v);
float vector2_distance(Vector2 a, Vector2 b);
bool vector2_equals(Vector2 a, Vector2 b, float epsilon);

// Vector3 operations
Vector3 vector3_create(float x, float y, float z);
Vector3 vector3_zero(void);
Vector3 vector3_add(Vector3 a, Vector3 b);
Vector3 vector3_subtract(Vector3 a, Vector3 b);
Vector3 vector3_scale(Vector3 v, float scalar);
Vector3 vector3_negate(Vector3 v);
float vector3_dot(Vector3 a, Vector3 b);
Vector3 vector3_cross(Vector3 a, Vector3 b);
float vector3_length_squared(Vector3 v);
float vector3_length(Vector3 v);
Vector3 vector3_normalize(Vector3 v);
float vector3_distance(Vector3 a, Vector3 b);
bool vector3_equals(Vector3 a, Vector3 b, float epsilon);

// Useful constants
#define VECTOR_EPSILON 1e-6f

#endif // VECTOR_MATH_H