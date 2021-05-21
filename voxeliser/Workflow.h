#ifndef HW1_WORKFLOW_H
#define HW1_WORKFLOW_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <queue>

#include "Point.h"
#include "Face.h"
#include "Rows.h"
#include "VoxelGrid.h"
#include <cstdlib>

std::ostream &operator<<(std::ostream &os, std::pair<Point, Point> &target) {
    os << "("
       << target.first.x << ","
       << target.first.y << ","
       << target.first.z << ") ("
       << target.second.x << ","
       << target.second.y << ","
       << target.second.z << ")";
    return os;
}

unsigned int nd_to_1d(const unsigned int &x, const unsigned int &y, const unsigned int &z, VoxelGrid &grid) {
    return x + y * grid.max_row_x + z * grid.max_row_x * grid.max_row_y;
}

void read_file(std::vector<Point> &vertices, std::vector<Face> &faces, const char *file_in) {
//    std::cout << "===========================" << std::endl;
//    std::cout << "1. read_file: start" << std::endl;
    std::ifstream in(file_in, std::ios::in);
    if (!in) {
        std::cerr << "Cannot open " << file_in << std::endl;
        exit(1);
    }
    std::string line;
    while (std::getline(in, line)) {
        //check v for vertices
        if (line.substr(0, 2) == "v ") {
            std::istringstream v(line.substr(2));
            float x, y, z;
            v >> x;
            v >> y;
            v >> z;
            vertices.emplace_back(x, y, z);
        }
            //check for faces
        else if (line.substr(0, 2) == "f ") {
            std::istringstream f(line.substr(2));
            unsigned int a, b, c; //to store mesh index

            f >> a;
            f >> b;
            f >> c;
            faces.emplace_back(a, b, c);
        }
    }
    //test top 10 data of vertices and faces
//    std::cout<<"-- test top 10 data of vertices and faces"<<std::endl;
//    for (int i = 0; i < 10; i++) {
//        std::cout <<"--"<<  vertices[i] << std::endl;//ok
//    }
//    for (int i = 0; i < 10; i++) {
//        std::cout <<"--"<< faces[i][0]<< std::endl;//ok
//    }

//    std::cout << "1. read_file: end" << std::endl;
//    std::cout << "===========================" << std::endl;
}

VoxelGrid get_grid(std::vector<Point> &vertices, float voxel_size) {
    //find max and min
//    std::cout << "2. get_rows: start" << std::endl;
    float max_x = 0.0;
    auto min_x = float(INT_MAX);
    float max_y = 0.0;
    auto min_y = float(INT_MAX);
    float max_z = 0.0;
    auto min_z = float(INT_MAX);
    for (auto &vertice : vertices) {
        //x
        if (max_x < vertice[0]) {
            max_x = vertice[0];
        }
        if (min_x > vertice[0]) {
            min_x = vertice[0];
        }
        //y
        if (max_y < vertice[1]) {
            max_y = vertice[1];
        }
        if (min_y > vertice[1]) {
            min_y = vertice[1];
        }
        //z
        if (max_z < vertice[2]) {
            max_z = vertice[2];
        }
        if (min_z > vertice[2]) {
            min_z = vertice[2];
        }
    }
//    std::cout<<"--"<< "max_x: "<<max_x<<" min_x: "<<min_x<<'\n'
//             <<"--"<< "max_y: "<<max_y<<" min_y: "<<min_y<<'\n'
//             <<"--"<< "max_z: "<<max_z<<" min_z: "<<min_z<<'\n';

    //calculate number of voxels:
    unsigned int cnt_x = ceil((max_x - min_x) / voxel_size);
    unsigned int cnt_y = ceil((max_y - min_y) / voxel_size);
    unsigned int cnt_z = ceil((max_z - min_z) / voxel_size);
//    std::cout<<"--"<< "x: " << cnt_x<<" y: "<<cnt_y<<" z: "<<cnt_z<<'\n';
    VoxelGrid result = {cnt_x, cnt_y, cnt_z};

    result.voxel_size = voxel_size;
    result.max_coord_x = max_x;
    result.max_coord_y = max_y;
    result.max_coord_z = max_z;
    result.min_coord_x = min_x;
    result.min_coord_y = min_y;
    result.min_coord_z = min_z;

    // test the grid
//    std::cout << "length of grid.voxel: " << result.voxels.size() << '\n';
//    std::cout << "number of voxels: " << result.max_row_x * result.max_row_y * result.max_row_z << '\n';
//    std::cout << "target1 of first voxel: " << result.get_targets(0, 0, 0)[0] << std::endl;
//    std::cout << "target2 of first voxel: " << result.get_targets(0, 0, 0)[1] << std::endl;
//    std::cout << "target3 of first voxel: " << result.get_targets(0, 0, 0)[2] << std::endl;

//    std::cout << "2. get_rows: end" << std::endl;
//    std::cout << "===========================" << std::endl;
    return result;
}

float signed_volume(const Point &a, const Point &b, const Point &c, const Point &d) {
    // to do
    return (a - d).dot((b - d).cross(c - d)) / 6;
}

bool intersects(const Point &orig, const Point &dest, const Point &v0, const Point &v1, const Point &v2) {
    // to do
    auto product1 = signed_volume(v0, v1, v2, orig) * signed_volume(v0, v1, v2, dest);
    auto product2 = signed_volume(v0, orig, dest, v1) * signed_volume(v0, orig, dest, v2);
    auto product3 = signed_volume(v1, orig, dest, v2) * signed_volume(v1, orig, dest, v0);
    auto product4 = signed_volume(v2, orig, dest, v0) * signed_volume(v2, orig, dest, v1);
    if (product1 <= 0 && product2 <= 0 && product3 <= 0 && product4 <= 0) {
        return true;
    } else {
        return false;
    }
}

std::pair<Rows, Rows> triangle_bbox(const std::vector<Point> &vertices, const Face &triangle, VoxelGrid &voxels) {
    Rows min_row;
    Rows max_row;
    Point v0 = vertices[triangle.x - 1];
    Point v1 = vertices[triangle.y - 1];
    Point v2 = vertices[triangle.z - 1];
    float min_coord_x = std::min(std::min(v0.x, v1.x), v2.x);
    float min_coord_y = std::min(std::min(v0.y, v1.y), v2.y);
    float min_coord_z = std::min(std::min(v0.z, v1.z), v2.z);
    float max_coord_x = std::max(std::max(v0.x, v1.x), v2.x);
    float max_coord_y = std::max(std::max(v0.y, v1.y), v2.y);
    float max_coord_z = std::max(std::max(v0.z, v1.z), v2.z);

    // min_row_x/y/z use ceil(), round up
    min_row.x = floor((min_coord_x - voxels.min_coord_x) / voxels.voxel_size);
    min_row.y = floor((min_coord_y - voxels.min_coord_y) / voxels.voxel_size);
    min_row.z = floor((min_coord_z - voxels.min_coord_z) / voxels.voxel_size);

    // max_row_x/y/z use floor(), round down
    max_row.x = ceil((max_coord_x - voxels.min_coord_x) / voxels.voxel_size);
    max_row.y = ceil((max_coord_y - voxels.min_coord_y) / voxels.voxel_size);
    max_row.z = ceil((max_coord_z - voxels.min_coord_z) / voxels.voxel_size);

    return {min_row, max_row};
}

void triangles_intersects(const std::vector<Point> &vertices, const std::vector<Face> &faces, VoxelGrid &voxels) {
//    std::cout << "3. triangles_intersects: start" << std::endl;
    int cnt = 0;
    for (auto const &triangle: faces) {
        Point v0 = vertices[triangle.x - 1];
        Point v1 = vertices[triangle.y - 1];
        Point v2 = vertices[triangle.z - 1];
        auto bbox = triangle_bbox(vertices, triangle, voxels);
        //bbox.first -- min; bbox.second -- max;
        for (unsigned int i = bbox.first.x; i < bbox.second.x; ++i) {
            for (unsigned int j = bbox.first.y; j < bbox.second.y; ++j) {
                for (unsigned int k = bbox.first.z; k < bbox.second.z; ++k) {
                    auto tg1 = voxels.get_targets(i, j, k)[0];
                    auto tg2 = voxels.get_targets(i, j, k)[1];
                    auto tg3 = voxels.get_targets(i, j, k)[2];

//                    std::cout
//                    <<intersects(tg1.first, tg1.second, v0, v1, v2)<<", "
//                    <<intersects(tg2.first, tg2.second, v0, v1, v2)<<", "
//                    <<intersects(tg3.first, tg3.second, v0, v1, v2)
//                    <<std::endl;

                    if (intersects(tg1.first, tg1.second, v0, v1, v2) ||
                        intersects(tg2.first, tg2.second, v0, v1, v2) ||
                        intersects(tg3.first, tg3.second, v0, v1, v2)) {
                        voxels(i, j, k).is_intersectant = true;
                        voxels(i, j, k).volume = 1;
                    }
                }
            }
        }
    }

    for (auto vxl:voxels.voxels) {
        if (vxl.is_intersectant && vxl.volume == 1) {
            cnt++;
        }
    }

//    std::cout << "Number of intersected voxels: " << cnt << std::endl;
//    std::cout << "3. triangles_intersects: end" << std::endl;
//    std::cout << "===========================" << std::endl;
}

double Floodfill(VoxelGrid &voxels) {
//    std::cout << "4. Calculate volume: start" << std::endl;
    voxels(0, 0, 0).volume = 0;
    std::queue<Rows> q;
    q.push(Rows(0, 0, 0));

    int dx[6] = {1, -1, 0, 0, 0, 0};
    int dy[6] = {0, 0, 1, -1, 0, 0};
    int dz[6] = {0, 0, 0, 0, 1, -1};

    while (!q.empty()) {
        Rows temp = q.front();
        q.pop();

        for (int i = 0; i < 6; i++) {
            unsigned int x = temp.x + dx[i];
            unsigned int y = temp.y + dy[i];
            unsigned int z = temp.z + dz[i];

            if (x < 0 || x >= voxels.max_row_x || y < 0 || y >= voxels.max_row_y || z < 0 || z >= voxels.max_row_z)
                continue;
            if (voxels(x, y, z).volume == 2) {
                voxels(x, y, z).volume = 0;
                q.push(Rows(x, y, z));
            }
        }
    }
    int sum_v = 0;
    for (auto &voxel : voxels.voxels) {
        sum_v += voxel.volume;
    }
//    sum_v = voxels.voxels[0].volume;
    std::cout << "volume of object(voxel_size= " << voxels.voxel_size << "): "
              << float(sum_v) / 2 * pow(voxels.voxel_size, 3) << '\n';
//    std::cout << "4. Calculate volume: end" << std::endl;
//    std::cout << "===========================" << std::endl;
    return double(sum_v) / 2 * pow(voxels.voxel_size, 3);
}

double volume(VoxelGrid &voxels) {
//    std::cout << "4. Calculate volume: start" << std::endl;
    std::vector<Rows> neighbour;
    voxels(0, 0, voxels.max_row_z - 1).volume = 0;
    neighbour.emplace_back(0, 0, voxels.max_row_z - 1);

    while (!neighbour.empty()) {

        auto center = neighbour.front();
        int x_begin = int(center.x) - 1;
        x_begin = std::max(x_begin, 0);
        int x_end = int(center.x) + 2;
        x_end = std::min(x_end, int(voxels.max_row_x));
        int y_begin = int(center.y) - 1;
        y_begin = std::max(y_begin, 0);
        int y_end = int(center.y) + 2;
        y_end = std::min(y_end, int(voxels.max_row_y));
        int z_begin = int(center.z) - 1;
        z_begin = std::max(z_begin, 0);
        int z_end = int(center.z) + 2;
        z_end = std::min(z_end, int(voxels.max_row_z));


        for (unsigned int row_x = x_begin; row_x < x_end; ++row_x) {
            for (unsigned int row_y = y_begin; row_y < y_end; ++row_y) {
                for (unsigned int row_z = z_begin; row_z < z_end; ++row_z) {
                    if ((abs(int(neighbour.front().x) - int(voxels(row_x, row_y, row_z).row_x)) +
                         abs(int(neighbour.front().y) - int(voxels(row_x, row_y, row_z).row_y)) +
                         abs(int(neighbour.front().z) - int(voxels(row_x, row_y, row_z).row_z))) == 1 &&
                        voxels(row_x, row_y, row_z).volume == 2) {
                        voxels(row_x, row_y, row_z).volume = 0;
                        neighbour.emplace_back(row_x, row_y, row_z);
                    }
                }
            }
        }

//        auto back = neighbour.back();
        neighbour.erase(neighbour.begin());
    }

    int sum_v = 0;
    for (auto &voxel : voxels.voxels) {
        sum_v += voxel.volume;
    }
//    sum_v = voxels.voxels[0].volume;
    std::cout << "volume of object(voxel_size= " << voxels.voxel_size << "): "
              << float(sum_v) / 2 * pow(voxels.voxel_size, 3) << '\n';
//    std::cout << "4. Calculate volume: end" << std::endl;
//    std::cout << "===========================" << std::endl;
    return double(sum_v) / 2 * pow(voxels.voxel_size, 3);
}


void write_v(const char *file_out, Point pt) {
    std::ofstream outfile;
    outfile.open(file_out, std::ios::app);
    outfile << "v " << pt.x << " " << pt.y << " " << pt.z << std::endl;
    //space
    outfile << std::endl;
}


void write_f(const char *file_out, std::vector<int> &idx) {
    std::ofstream outfile;
    outfile.open(file_out, std::ios::app);

    outfile << "f " << idx[0] << " " << idx[1] << " " << idx[2] << " " << idx[3] << std::endl;
    outfile << "f " << idx[0] << " " << idx[1] << " " << idx[5] << " " << idx[4] << std::endl;
    outfile << "f " << idx[0] << " " << idx[3] << " " << idx[7] << " " << idx[4] << std::endl;
    outfile << "f " << idx[6] << " " << idx[7] << " " << idx[4] << " " << idx[5] << std::endl;
    outfile << "f " << idx[6] << " " << idx[7] << " " << idx[3] << " " << idx[2] << std::endl;
    outfile << "f " << idx[6] << " " << idx[5] << " " << idx[1] << " " << idx[2] << std::endl;
    //space
    outfile << std::endl;
}

int File_Exist(const char *file_out) {
    FILE *fp;
    fp = fopen(file_out, "r");
    if (fp == NULL)
        return 0; // not exist
    else {
        fclose(fp);
        return 1; //exist
    }
}

void test_file_exist(const char *file_out) {
    if (File_Exist(file_out) == 1) {
        remove(file_out);
//        printf("File Exist and Deleted!\n");
    }
}


void output_file_compress(const char *file_out, const float scale, VoxelGrid &grid) {
//    std::cout << "5. Write voxels: start" << std::endl;
    test_file_exist(file_out);
    int idx = 0;
    int obj_pt_cnt = 0;
    std::vector<std::vector<int>> face_idx;
    for (int row_z = 0; row_z < grid.max_row_z; ++row_z) {
        for (int row_y = 0; row_y < grid.max_row_y; ++row_y) {
            for (int row_x = 0; row_x < grid.max_row_x; ++row_x) {
//                std::cout<< "voxel idx: " << nd_to_1d(row_x, row_y, row_z, grid) <<std::endl;
                std::vector<int> tmp;
                auto vxl = grid(row_x, row_y, row_z);
                if (vxl.volume == 0) {

                    for (int i = 0; i < 8; ++i) {
                        tmp.push_back(0);
                    }

                    face_idx.emplace_back(tmp);

                    if (face_idx.size() > grid.max_row_x * grid.max_row_y * 2) {
                        face_idx.erase(face_idx.begin());
                    }

//                    for(auto elm: tmp) std::cout<<elm<< " ";
//                    std::cout<<std::endl;
                } else {
                    obj_pt_cnt++;
                    auto pt_set1 = grid.get_vertices(scale, row_x, row_y, row_z);
                    if (idx == 0) {
                        for (int i = 0; i < pt_set1.size(); ++i) {
                            idx++;
                            tmp.push_back(idx);
                            // TODO: write_v(file_out, pt_size1[i], grid);
                            write_v(file_out, pt_set1[i]);
                        }
                        face_idx.emplace_back(tmp);
                        // TODO: write_f(file_out, tmp);
                        write_f(file_out, tmp);
                        if (face_idx.size() > grid.max_row_x * grid.max_row_y * 2) {
                            face_idx.erase(face_idx.begin());
                        }
//                        std::cout<<idx<<std::endl;
//                        for(auto elm: tmp) std::cout<<elm<< " ";
//                        std::cout<<std::endl;

                    } else {
                        for (auto pt1: pt_set1) {
                            bool redundant = false;

                            auto center = Rows(row_x, row_y, row_z);
                            int x_begin = int(center.x) - 1;
                            x_begin = std::max(x_begin, 0);
                            int x_end = int(center.x) + 2;
                            x_end = std::min(x_end, int(grid.max_row_x));
                            int y_begin = int(center.y) - 1;
                            y_begin = std::max(y_begin, 0);
                            int y_end = int(center.y) + 2;
                            y_end = std::min(y_end, int(grid.max_row_y));
                            int z_begin = int(center.z) - 1;
                            z_begin = std::max(z_begin, 0);
                            int z_end = int(center.z) + 2;
                            z_end = std::min(z_end, int(grid.max_row_z));

                            for (unsigned int row_k = z_begin; row_k < z_end && !redundant; ++row_k) {
                                for (unsigned int row_j = y_begin; row_j < y_end && !redundant; ++row_j) {
                                    for (unsigned int row_i = x_begin; row_i < x_end && !redundant; ++row_i) {
                                        if (grid(row_i, row_j, row_k).volume != 0) {
                                            auto pt_set2 = grid.get_vertices(scale, row_i, row_j, row_k);
                                            for (int i = 0; i < pt_set2.size() && !redundant; ++i) {
                                                auto pt2 = pt_set2[i];
                                                int distance = int(nd_to_1d(row_x, row_y, row_z, grid)) -
                                                               int(nd_to_1d(row_i, row_j, row_k, grid));
                                                if (pt1 == pt2 && !redundant && distance > 0) {
//                                                    std::cout<<idx<<": ";
                                                    redundant = true;
                                                    tmp.push_back(face_idx[face_idx.size() - distance][i]);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            if (!redundant) {
                                idx++;
                                // TODO: write_v(file_out, pt1, grid);
                                write_v(file_out, pt1);
//                                std::cout<<idx<<std::endl;
                                tmp.push_back(idx);
                            }
                        }
                        face_idx.emplace_back(tmp);
                        if (face_idx.size() > grid.max_row_x * grid.max_row_y * 2) {
                            face_idx.erase(face_idx.begin());
                        }
                        // TODO: write_f(file_out, tmp);
                        write_f(file_out, tmp);
//                        for(auto elm: tmp) std::cout<<elm<< " ";
//                        std::cout<<std::endl;
                    }
                }
//                std::cout<< "progress: " << (double(nd_to_1d(row_x, row_y, row_z, grid)) - 1) / grid.voxels.size() <<std::endl;
            }
        }
    }
    std::cout << "genarate " << obj_pt_cnt * 8 << " points (with redundant pts)" << std::endl;
    std::cout << "genarate " << idx << " points (without redundant pts)" << std::endl;
//    std::cout<<face_idx.size()<<std::endl;
//    auto length = face_idx.size();
//    for(int k = 0; k < length; ++k){
//        std::cout<<k<<":: ";
//        auto pair_vec = face_idx.front();
//        face_idx.erase(face_idx.begin());
//        for(int i = 0; i < 8; ++i){
//            std::cout<<pair_vec[i]<<", ";
//        }
//        std::cout<<std::endl;
//    }
//    std::cout << "5. Write voxels: end" << std::endl;
//    std::cout << "===========================" << std::endl;
}

std::vector<Point>
zoom_coords(const float scale, const unsigned int x, const unsigned int y, const unsigned int z, VoxelGrid &grid) {
    auto mid_pt = (grid.get_targets(x, y, z)[0].first + grid.get_targets(x, y, z)[0].second) / 2;
    // lf:left->negative direction of x
    // rt:right->positive direction of x
    // ft:front->negative directive of y
    // bk:back->positive direction of y
    // dn:down->negative directive of z
    // up:up->positive directive of z

    // store order: [0]lf_ft_dn -> [1]rt_ft_dn -> [2]rt_bk_dn -> [3]lf_bk_dn -> [4]lf_ft_up -> [5]rt_ft_up -> [6]rt_bk_up -> [7]lf_bk_up
    std::vector<Point> result;
    double offset = grid.voxel_size / 2 * scale;
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

void output_file_normal(const char *file_out, const float scale, VoxelGrid &grid) {
//    std::cout << "5. Write voxels: start" << std::endl;
    test_file_exist(file_out);
    std::ofstream outfile;
    outfile.open(file_out, std::ios::out | std::ios::trunc);
    int face_cnt = 0;
    int pt_cnt = 0;

    for (unsigned int row_x = 0; row_x < grid.max_row_x; ++row_x) {
//        std::cout<<"mark2"<<std::endl;
        for (unsigned int row_y = 0; row_y < grid.max_row_y; ++row_y) {
//            std::cout<<"mark3"<<std::endl;
            for (unsigned int row_z = 0; row_z < grid.max_row_z; ++row_z) {
                if (grid(row_x, row_y, row_z).is_intersectant) {
                    //xyz location of voxel
//                    outfile << "g voxel_" << row_x << "_" << row_y << "_" << row_z << std::endl;

                    // 8 vertices
                    std::vector<Point> p8 = zoom_coords(scale, row_x, row_y, row_z, grid);
                    for (int i = 0; i < 8; i++) {
                        outfile << "v " << p8[i].x << " " << p8[i].y << " " << p8[i].z << std::endl;
                        pt_cnt++;
                    }
                    //space
                    outfile << std::endl;

                    //6 faces - counterclockwise
                    //f1 1234
                    outfile << "f " << face_cnt * 8 + 1 << " " << face_cnt * 8 + 2 << " " << face_cnt * 8 + 3
                            << " " << face_cnt * 8 + 4 << std::endl;
                    //f2 1265
                    outfile << "f " << face_cnt * 8 + 1 << " " << face_cnt * 8 + 2 << " " << face_cnt * 8 + 6
                            << " " << face_cnt * 8 + 5 << std::endl;
                    //f3 1485
                    outfile << "f " << face_cnt * 8 + 1 << " " << face_cnt * 8 + 4 << " " << face_cnt * 8 + 8
                            << " " << face_cnt * 8 + 5 << std::endl;
                    //f4 7856
                    outfile << "f " << face_cnt * 8 + 7 << " " << face_cnt * 8 + 8 << " " << face_cnt * 8 + 5
                            << " " << face_cnt * 8 + 6 << std::endl;
                    //f5 7843
                    outfile << "f " << face_cnt * 8 + 7 << " " << face_cnt * 8 + 8 << " " << face_cnt * 8 + 4
                            << " " << face_cnt * 8 + 3 << std::endl;
                    //f6 7623
                    outfile << "f " << face_cnt * 8 + 7 << " " << face_cnt * 8 + 6 << " " << face_cnt * 8 + 2
                            << " " << face_cnt * 8 + 3 << std::endl;
                    face_cnt++;

                    //space
                    outfile << std::endl;
                }
            }
        }
    }
//    std::cout << "genarate " << pt_cnt << " points (with redundant pts)" << std::endl;
//    std::cout << "genarate " << file_out << " faces" << std::endl;
//    std::cout << "5. Write voxels: end" << std::endl;
//    std::cout << "===========================" << std::endl;
}

#endif //HW1_WORKFLOW_H
