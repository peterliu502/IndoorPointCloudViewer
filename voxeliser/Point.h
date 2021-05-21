#ifndef Point_h
#define Point_h

#include <cassert>

struct Point {
    float x, y, z;

    Point() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    Point(const float &x, const float &y, const float &z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    float &operator[](const int &coordinate) {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    float operator[](const int &coordinate) const {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    bool operator== (const Point &other) const {
        double tol = 0.01;
        if((abs(x - other.x) < tol) && (abs(y - other.y) < tol) && (abs(z - other.z) < tol)){
            return true;
        }
        else{
            return false;
        }
    }

    const Point operator+(const Point &other) const {
        return Point(x + other.x, y + other.y, z + other.z);
    }

    const Point operator-(const Point &other) const {
        return Point(x - other.x, y - other.y, z - other.z);
    }

    const Point operator*(const float &other) const {
        return Point(x * other, y * other, z * other);
    }

    const Point operator/(const float &other) const {
        return Point(x / other, y / other, z / other);
    }

    float dot(const Point &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    const Point cross(const Point &other) const {
        return Point(y * other.z - z * other.y, -(x * other.z - z * other.x), x * other.y - y * other.x);
    }
};

std::ostream &operator<<(std::ostream &os, const Point &p) {
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

#endif /* Point_h */
