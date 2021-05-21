#ifndef Rows_h
#define Rows_h

struct Rows {
    unsigned int x, y, z;

    Rows() {
        x = 0;
        y = 0;
        z = 0;
    }

    Rows(unsigned int x, unsigned int y, unsigned int z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    unsigned int &operator[](int coordinate) {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    unsigned int operator[](int coordinate) const {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

   bool operator==(const Rows &other) const {
       if(x == other.x && y == other.y && z == other.z){
           return true;
       }
       else{
           return false;
       }
    }

    bool operator!=(const Rows &other) const {
        if(x != other.x || y != other.y || z != other.z){
            return true;
        }
        else{
            return false;
        }
    }

};

std::ostream &operator<<(std::ostream &os, const Rows &r) {
    os << "(" << r.x << ", " << r.y << ", " << r.z << ")";
    return os;
}

#endif /* Rows_h */
