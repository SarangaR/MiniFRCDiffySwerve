#include <Vector.h>

Vector::Vector(float x, float y, float z) :
    x(x),
    y(y),
    z(z)
{}

float Vector::magnitude() {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float Vector::angle() {
    return atan2(y, x);
}

Vector Vector::operator+(Vector v) {
    return Vector(x + v.x, y + v.y, z + v.z);
}

Vector Vector::operator-(Vector v) {
    return Vector(x - v.x, y - v.y, z - v.z);
}

Vector Vector::operator*(float scalar) {
    return Vector(x * scalar, y * scalar, z * scalar);
}

Vector Vector::operator/(float scalar) {
    return Vector(x / scalar, y / scalar, z / scalar);
}

float Vector::dot(Vector v) {
    return x * v.x + y * v.y + z * v.z;
}

Vector Vector::cross(Vector v) {
    return Vector(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}

Vector Vector::normalize() {
    return *this / magnitude();
}

Vector Vector::rotate(float angle) {
    float x1 = x * cos(angle) - y * sin(angle);
    float y1 = x * sin(angle) + y * cos(angle);
    return Vector(x1, y1, z);
}

Vector Vector::project(Vector v) {
    return v * (dot(v) / pow(v.magnitude(), 2));
}

Vector Vector::reflect(Vector normal) {
    return *this - normal * (2 * dot(normal));
}

Vector Vector::operator+=(Vector v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector Vector::operator-=(Vector v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vector Vector::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector Vector::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

bool Vector::operator==(Vector v) {
    return x == v.x && y == v.y && z == v.z;
}

bool Vector::operator!=(Vector v) {
    return x != v.x || y != v.y || z != v.z;
}

bool Vector::operator<(Vector v) {
    return magnitude() < v.magnitude();
}

bool Vector::operator<=(Vector v) {
    return magnitude() <= v.magnitude();
}

bool Vector::operator>(Vector v) {
    return magnitude() > v.magnitude();
}

bool Vector::operator>=(Vector v) {
    return magnitude() >= v.magnitude();
}

float Vector::operator[](int index) {
    if (index == 0) {
        return x;
    }
    else if (index == 1) {
        return y;
    }
    else if (index == 2) {
        return z;
    }
    return 0;
}

