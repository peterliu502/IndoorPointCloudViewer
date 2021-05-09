#include <iostream>
#include <pcl/io/pcd_io.h>
#include "basic_methods.cpp"

int read_pts(std::string& path, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pts
){
    // PCL can load PLY file and PCD file, but loading PCD is faster.
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (path, *pts) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); // If the path doesn't exist, give a warning.
        return (-1);
    }

    std::cout << "LOAD: "
              << pts->width * pts->height
              << " points from the input file."
              << std::endl;

//    for (int i = 0; i < 5; ++i){
//        auto point = pts->points[i];
//        std::cout << "    " << point.x
//                  << " "    << point.y
//                  << " "    << point.z
//                  << " "    << point.intensity
//                  <<std::endl;
//    }

    return 0;
}

void remove_outliers(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_out,
                     double r,
                     int min_neighbours){
    pts_out->clear();
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> pcFilter;    // create a filter object
    pcFilter.setInputCloud(pts_in);                        // import the pts_in into the filter object
    pcFilter.setRadiusSearch(r);                           // set the radius of the search circle
    pcFilter.setMinNeighborsInRadius(min_neighbours);      // set the minimum neighbours in the search circle
    pcFilter.filter(*pts_out);                             // store the filter result into the pts_out
}

//void remove_fake_faces(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts,
//                       pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_out,
//                       double size,
//                       int loop_times){
//    for(int i = 0; i < loop_times; ++i){
//        pts_out->clear();
//        size += i * 0.05;
//        Grid grid_density(pts, size);
//        std::vector<double> vec_density;
//        for (int row = 0; row < grid_density.get_row_num(); ++row) {
//            for (int col = 0; col < grid_density.get_col_num(); ++col) {
////            vec_density.push_back(grid_density.find_cell_by_idx(row, col)->pt_vec.size() / pow(grid_density.get_size(), 2));
//                if(grid_density.find_cell_by_idx(row, col)->pt_vec.size() / pow(grid_density.get_size(), 2) > 300){
//                    for (auto ptr_pt: grid_density.find_cell_by_idx(row, col)->pt_vec) pts_out->push_back(*ptr_pt);
//                }
//            }
//        }
//    }
////    std::sort(vec_density.begin(), vec_density.end());
////    for (auto &density: vec_density) std::cout << density << std::endl;
//}

void remove_fake_faces(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_out,
                       double& floor_height,
                       int& roof_idx,
                       int& ground_idx,
                       double size){
    // find the largest and second largest faces
    std::map<unsigned int, std::vector<double>> map_z;
    for(auto &pt: pts->points) map_z[pt.intensity].push_back(pt.z);
    std::pair<std::pair<int, double>, std::pair<int, double>> two_floors = {{-1, 99999}, {-1, 99999}};
    for(auto &vec: map_z){
        if (two_floors.first.first == -1) two_floors.first = {vec.first, mean_sqr_err(vec.second)};
        else if (two_floors.second.first == -1) two_floors.second = {vec.first, mean_sqr_err(vec.second)};
        else if (two_floors.first.second > mean_sqr_err(vec.second)) two_floors.first = {vec.first, mean_sqr_err(vec.second)};
        else if (two_floors.second.second > mean_sqr_err(vec.second)) two_floors.second = {vec.first, mean_sqr_err(vec.second)};
    }
    int floor_idx1 = two_floors.first.first;
    int floor_idx2 = two_floors.second.first;
    int smaller_floor;
    int larger_floor;
    if (map_z[floor_idx1].size() > map_z[floor_idx2].size()){
        smaller_floor = floor_idx2;
        larger_floor = floor_idx1;
    }
    else{
        smaller_floor = floor_idx1;
        larger_floor = floor_idx2;
    }

    // the height of one floor
    floor_height = abs(avg_vec_elm(map_z[floor_idx1]) - avg_vec_elm(map_z[floor_idx2]));

    // only keep the largest faces' point which are overlapped with the second largest face.
    Grid grid(pts, size);
    for (int row = 0; row < grid.get_row_num(); ++row) {
        for (int col = 0; col < grid.get_col_num(); ++col) {
            bool contain_smaller_floor = false;
            if (!grid.find_cell_by_idx(row, col)->pt_vec.empty()){
                for (auto ptr : grid.find_cell_by_idx(row, col)->pt_vec){
                    if (int(ptr->intensity) == smaller_floor) contain_smaller_floor = true;
                }
                if (contain_smaller_floor) for (auto ptr: grid.find_cell_by_idx(row, col)->pt_vec) pts_out->push_back(*ptr);
            }
        }
    }

    // the higher face is roof, the lower face is ground
    if (avg_vec_elm(map_z[floor_idx1]) < avg_vec_elm(map_z[floor_idx2])){
        roof_idx = floor_idx2;
        ground_idx = floor_idx1;
    }
    else{
        roof_idx = floor_idx1;
        ground_idx = floor_idx2;
    }
}

void extract_floors( pcl::PointCloud<pcl::PointXYZI>::Ptr& pts,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_roof_out,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_rest_out,
                    int roof_idx,
                    int ground_idx,
                    double r,
                    int min_neighbours){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_roof(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_rest(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto ptr: pts->points){
        if (int(ptr.intensity) == roof_idx) pts_roof->push_back(ptr);
        else if (int(ptr.intensity) == ground_idx) pts_ground->push_back(ptr);
        else pts_rest->push_back(ptr);
    }

    // remove the noise
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_ground_out(new pcl::PointCloud<pcl::PointXYZI>);
    remove_outliers(pts_roof, pts_roof_out, r, min_neighbours);
    remove_outliers(pts_ground, pts_ground_out, r, min_neighbours);
    remove_outliers(pts_rest, pts_rest_out, r, min_neighbours + 5);

    // output roof part
    pcl::io::savePLYFileASCII("./data/VRR_roof_A.ply", *pts_roof_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: VRR_roof_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/VRR_roof_B.ply", *pts_roof_out); // output PLY (Binary) file
    std::cout << "OUTPUT: VRR_roof_B.ply" << std::endl;
    // output ground part
    pcl::io::savePLYFileASCII("./data/VRR_ground_A.ply", *pts_ground_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: VRR_ground_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/VRR_ground_B.ply", *pts_ground_out); // output PLY (Binary) file
    std::cout << "OUTPUT: VRR_ground_B.ply" << std::endl;
}

void extract_non_archi(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_roof,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_rest,
                       double size,
                       double r,
                       int min_neighbours){
    std::vector<double> vec_roof_heights;
    for (auto &pt: pts_roof->points) vec_roof_heights.push_back(pt.z);
    double min_roof_height = min_vec_elm(vec_roof_heights);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_archi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_non_archi(new pcl::PointCloud<pcl::PointXYZI>);

    // if cell's highest z value is larger than minimum roof's height,
    // this cell's points will be consider as the architecture part
    // or it will be the non-architecture part.
    Grid grid_rest(pts_rest, size);
    for (int row = 0; row < grid_rest.get_row_num(); ++row) {
        for (int col = 0; col < grid_rest.get_col_num(); ++col) {
            std::vector<double> vec_cell_heights;
            for (auto &pt: grid_rest.find_cell_by_idx(row, col)->pt_vec) vec_cell_heights.push_back(pt->z);
            double max_cell_height = max_vec_elm(vec_cell_heights);
            if (max_cell_height >= min_roof_height - 0.2 ) for (auto &pt: grid_rest.find_cell_by_idx(row, col)->pt_vec) pts_archi->push_back(*pt);
            else for (auto &pt: grid_rest.find_cell_by_idx(row, col)->pt_vec) pts_non_archi->push_back(*pt);
        }
    }

    // remove the noise in non-architecture part
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_non_archi_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_non_archi_tmp1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_non_archi_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    remove_outliers(pts_non_archi, pts_non_archi_tmp1, r, min_neighbours);
    remove_outliers(pts_non_archi_tmp1, pts_non_archi_tmp2, r, min_neighbours);
    remove_outliers(pts_non_archi_tmp2, pts_non_archi_out, r, min_neighbours);

    // output architecture part
    pcl::io::savePLYFileASCII("./data/VRR_rest_archi_A.ply", *pts_archi); // output PLY (ASCII) file
    std::cout << "OUTPUT: VRR_rest_archi_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/VRR_rest_archi_B.ply", *pts_archi); // output PLY (Binary) file
    std::cout << "OUTPUT: VRR_rest_archi_B.ply" << std::endl;
    // output non-architecture part
    pcl::io::savePLYFileASCII("./data/VRR_rest_nonarchi_A.ply", *pts_non_archi_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: VRR_rest_nonarchi_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/VRR_rest_nonarchi_B.ply", *pts_non_archi_out); // output PLY (Binary) file
    std::cout << "OUTPUT: VRR_rest_nonarchi_B.ply" << std::endl;
};

void write_pts(std::string& path_A, std::string& path_B, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pts){
    pcl::io::savePLYFileASCII(path_A, *pts); // output PLY (ASCII) file
    pcl::io::savePLYFileBinary(path_B, *pts); // output PLY (Binary) file
}



