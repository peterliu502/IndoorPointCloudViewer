#ifndef VoxelGrid_h
#define VoxelGrid_h

class Voxel{
public:
    unsigned int row_x, row_y, row_z;
    bool is_intersectant = false;
    // 2 - interior; 1 - intersects; 0 - exterior;
    unsigned short int volume = 2;
//    std::pair<Point, Point> target1;
//    std::pair<Point, Point> target2;
//    std::pair<Point, Point> target3;

    Voxel(): row_x(0), row_y(0), row_z(0){};

    Voxel(unsigned int _x, unsigned int _y, unsigned int _z): row_x(_x), row_y(_y), row_z(_z) {};

};

struct VoxelGrid {
    std::vector<Voxel> voxels;
//    std::vector<unsigned int> voxels;
    unsigned int max_row_x, max_row_y, max_row_z;
    float max_coord_x, max_coord_y, max_coord_z, min_coord_x, min_coord_y, min_coord_z;
    float voxel_size = 1;

    VoxelGrid(unsigned int &x, unsigned int &y, unsigned int &z){
        max_row_x = x;
        max_row_y = y;
        max_row_z = z;
        unsigned int total_voxels = x * y * z;
        voxels.reserve(total_voxels);

        for (unsigned int i = 0; i < total_voxels; ++i) {
            voxels.emplace_back(0, 0, 0);
        }

        for(unsigned int row_x = 0; row_x < max_row_x; ++row_x){
            for(unsigned int row_y = 0; row_y < max_row_y; ++row_y){
                for(unsigned int row_z = 0; row_z < max_row_z; ++row_z){
                    unsigned int idx = row_x + row_y * max_row_x + row_z * max_row_x * max_row_y;
                    voxels[idx].row_x = row_x;
                    voxels[idx].row_y = row_y;
                    voxels[idx].row_z = row_z;
                }
            }
        }
    }

    Voxel &operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) {
        assert(x >= 0 && x < max_row_x);
        assert(y >= 0 && y < max_row_y);
        assert(z >= 0 && z < max_row_z);
        return voxels[x + y * max_row_x + z * max_row_x * max_row_y];
    }

    Voxel operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) const {
        assert(x >= 0 && x < max_row_x);
        assert(y >= 0 && y < max_row_y);
        assert(z >= 0 && z < max_row_z);
        return voxels[x + y * max_row_x + z * max_row_x * max_row_y];
    }

    std::vector<std::pair<Point, Point>> get_targets(unsigned i, unsigned j, unsigned k){
        auto row_x = voxels[i + j * max_row_x + k * max_row_x * max_row_y].row_x;
        auto row_y = voxels[i + j * max_row_x + k * max_row_x * max_row_y].row_y;
        auto row_z = voxels[i + j * max_row_x + k * max_row_x * max_row_y].row_z;
        std::vector<std::pair<Point, Point>> result;
        result.emplace_back(Point(float(row_x) * voxel_size + min_coord_x, float(row_y) * voxel_size + min_coord_y + voxel_size / 2, float(row_z) * voxel_size + min_coord_z + voxel_size / 2),
                            Point(float(row_x) * voxel_size + min_coord_x + voxel_size , float(row_y) * voxel_size + min_coord_y + voxel_size/ 2, float(row_z) * voxel_size + min_coord_z + voxel_size / 2));
        result.emplace_back(Point(float(row_x) * voxel_size + min_coord_x + voxel_size / 2, float(row_y) * voxel_size + min_coord_y, float(row_z) * voxel_size + min_coord_z + voxel_size / 2),
                            Point(float(row_x) * voxel_size + min_coord_x + voxel_size / 2, float(row_y) * voxel_size + min_coord_y + voxel_size, float(row_z) * voxel_size + min_coord_z + voxel_size / 2));
        result.emplace_back(Point(float(row_x) * voxel_size + min_coord_x + voxel_size / 2, float(row_y) * voxel_size + min_coord_y + voxel_size / 2, float(row_z) * voxel_size + min_coord_z),
                            Point(float(row_x) * voxel_size + min_coord_x + voxel_size / 2, float(row_y) * voxel_size + min_coord_y + voxel_size / 2, float(row_z) * voxel_size + min_coord_z + voxel_size));

        return result;
    }

    Point get_mid_pt(unsigned i, unsigned j, unsigned k){
        return (get_targets(i, j, k)[0].first + get_targets(i, j, k)[0].second) / 2;
    }

    std::vector<Point> get_vertices(float zoom_scale, unsigned i, unsigned j, unsigned k){
        // lf:left->negative direction of x
        // rt:right->positive direction of x
        // ft:front->negative directive of y
        // bk:back->positive direction of y
        // dn:down->negative directive of z
        // up:up->positive directive of z

        // store order: [0]lf_ft_dn -> [1]rt_ft_dn -> [2]rt_bk_dn -> [3]lf_bk_dn -> [4]lf_ft_up -> [5]rt_ft_up -> [6]rt_bk_up -> [7]lf_bk_up

        auto mid_pt = get_mid_pt(i, j, k);
        std::vector<Point> result;
        double offset = voxel_size / 2 * zoom_scale;
        result.emplace_back(mid_pt.x - offset, mid_pt.y - offset, mid_pt.z - offset); // lf_ft_dn
        result.emplace_back(mid_pt.x + offset, mid_pt.y - offset, mid_pt.z - offset); // rt_ft_dn
        result.emplace_back(mid_pt.x + offset, mid_pt.y + offset, mid_pt.z - offset); // rt_bk_dn
        result.emplace_back(mid_pt.x - offset, mid_pt.y + offset, mid_pt.z - offset); // lf_bk_dn
        result.emplace_back(mid_pt.x - offset, mid_pt.y - offset, mid_pt.z + offset); // lf_ft_up
        result.emplace_back(mid_pt.x + offset, mid_pt.y - offset, mid_pt.z + offset); // rt_ft_up
        result.emplace_back(mid_pt.x + offset, mid_pt.y + offset, mid_pt.z + offset); // rt_bk_up
        result.emplace_back(mid_pt.x - offset, mid_pt.y + offset, mid_pt.z + offset); // lf_bk_up

        return result;
    }

    Voxel* find_cell_by_coord(Point &pt){
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        unsigned int row_x = int((x - min_coord_x) / voxel_size);
        unsigned int row_y = int((y - min_coord_y) / voxel_size);
        unsigned int row_z = int((z - min_coord_z) / voxel_size);
        if (row_x >= max_row_x || row_y >= max_row_y || row_z >= max_row_z) assert(false);
        return &voxels[row_x + row_y * this->max_row_x + row_z * this->max_row_x * this->max_row_y];
    }

};

std::ostream &operator<<(std::ostream &os, const VoxelGrid &r) {
    os << "(" << r.max_row_x << ", " << r.max_row_y << ", " << r.max_row_z << ")";
    return os;
}

#endif /* VoxelGrid_h */
