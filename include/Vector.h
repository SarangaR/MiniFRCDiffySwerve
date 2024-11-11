#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>

class Vector {
    public:
        Vector(float x, float y, float z);
        float x;
        float y;
        float z;
        float magnitude();
        float angle();
        Vector operator+(Vector v);
        Vector operator-(Vector v);
        Vector operator*(float scalar);
        Vector operator/(float scalar);
        float dot(Vector v);
        Vector cross(Vector v);
        Vector normalize();
        Vector rotate(float angle);
        Vector project(Vector v);
        Vector reflect(Vector normal);
        Vector operator+=(Vector v);
        Vector operator-=(Vector v);
        Vector operator*=(float scalar);
        Vector operator/=(float scalar);
        bool operator==(Vector v);
        bool operator!=(Vector v);
        bool operator<(Vector v);
        bool operator<=(Vector v);
        bool operator>(Vector v);
        bool operator>=(Vector v);
        float operator[](int index);
};
#endif // VECTOR_H