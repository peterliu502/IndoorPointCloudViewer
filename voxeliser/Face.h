//
// Created by Julia on 21/02/18.
//

#ifndef HW1_FACE_H
#define HW1_FACE_H

struct Face {
    unsigned int x, y, z;

    Face() {
        x = 0;
        y = 0;
        z = 0;
    }

    Face(const unsigned int &x, const unsigned int &y, const unsigned int &z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    unsigned int & operator[] (const int &coordinate) {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    unsigned int operator[](const int &coordinate) const {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }
};
#endif //HW1_FACE_H
