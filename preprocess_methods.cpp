#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <cstring>
#include <string>
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

void mls_smooth(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                pcl::PointCloud<pcl::PointNormal>::Ptr &result){
    // smooth the point clouds
    pcl::search::KdTree<pcl::PointXYZI>::Ptr search(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal> mls;
    std::cout << cloud->points.size() << std::endl;
    search->setInputCloud(cloud);
    std::cout << search->input_->size() << std::endl;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(5);
    mls.setSearchMethod(search);
    mls.setSearchRadius(0.015); // Level of smooth
    mls.process(*result);
}

void pt2mesh(const std::string& file_out, pcl::PointCloud<pcl::PointNormal>::Ptr& pts){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &pt: pts->points) cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

    // estimate normal vectors
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    // add normal vector info into point clouds
    // * cloud_with_normals = cloud + normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);


    // create kd-tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // initialize GreedyProjectionTriangulation objects，and setting parameters
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //create a PolygonMesh object to store the result
    pcl::PolygonMesh triangles;

    // set parameters of GreedyProjectionTriangulation objects
    gp3.setSearchRadius (1.5);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);

    // output triangles
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct (triangles);
    pcl::io::savePLYFile(file_out, triangles);
}

void extract_floors(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_roof_out,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_rest_out,
                    int roof_idx,
                    int ground_idx,
                    double r,
                    int min_neighbours){
    std::vector<double> vec_roof_z;
    std::vector<double> vec_ground_z;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_roof(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_rest(new pcl::PointCloud<pcl::PointXYZI>);

    for(auto ptr: pts->points) {
            if (int(ptr.intensity) == roof_idx) vec_roof_z.push_back(ptr.z);
            else if (int(ptr.intensity) == ground_idx) vec_ground_z.push_back(ptr.z);
    }
    double min_roof_z = min_vec_elm(vec_roof_z) + 0.5 * abs(max_vec_elm(vec_roof_z) - max_vec_elm(vec_roof_z));
    double max_ground_z = max_vec_elm(vec_ground_z);

    for(auto ptr: pts->points){
        if ((ptr.z) >= min_roof_z) pts_roof->push_back(ptr);
        else if ((ptr.z) <= max_ground_z) pts_ground->push_back(ptr);
        else pts_rest->push_back(ptr);
    }

    // remove the noise
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_ground_out(new pcl::PointCloud<pcl::PointXYZI>);
    remove_outliers(pts_roof, pts_roof_out, r, min_neighbours);
    remove_outliers(pts_ground, pts_ground_out, r, min_neighbours);
    remove_outliers(pts_rest, pts_rest_out, r, min_neighbours + 5);

    // output roof part
    pcl::io::savePLYFileASCII("./data/pointclouds/POINTCLOUDS_roof_A.ply", *pts_roof_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: POINTCLOUDS_roof_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/pointclouds/POINTCLOUDS_roof_B.ply", *pts_roof_out); // output PLY (Binary) file
    std::cout << "OUTPUT: POINTCLOUDS_roof_B.ply" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr result_roof (new pcl::PointCloud<pcl::PointNormal>);
    mls_smooth(pts_roof_out, result_roof);
    pt2mesh("./data/meshes/MESH_roof.ply", result_roof); // output mesh file
    std::cout << "OUTPUT: MESH_roof.ply" << std::endl;

    // output ground part
    pcl::io::savePLYFileASCII("./data/pointclouds/POINTCLOUDS_ground_A.ply", *pts_ground_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: POINTCLOUDS_ground_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/pointclouds/POINTCLOUDS_ground_B.ply", *pts_ground_out); // output PLY (Binary) file
    std::cout << "OUTPUT: POINTCLOUDS_ground_B.ply" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr result_ground (new pcl::PointCloud<pcl::PointNormal>);
    mls_smooth(pts_ground_out, result_ground);
    pt2mesh("./data/meshes/MESH_ground.ply", result_ground); // output mesh file
    std::cout << "OUTPUT: MESH_ground.ply" << std::endl;

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_non_archi_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    remove_outliers(pts_non_archi, pts_non_archi_tmp1, r, min_neighbours);
    remove_outliers(pts_non_archi_tmp1, pts_non_archi_tmp2, r, min_neighbours + 3);
    remove_outliers(pts_non_archi_tmp2, pts_non_archi_tmp3, r, min_neighbours + 6);
    remove_outliers(pts_non_archi_tmp3, pts_non_archi_out, r, min_neighbours + 9);

    // output architecture part
    pcl::io::savePLYFileASCII("./data/pointclouds/POINTCLOUDS_rest_archi_A.ply", *pts_archi); // output PLY (ASCII) file
    std::cout << "OUTPUT: POINTCLOUDS_rest_archi_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/pointclouds/POINTCLOUDS_rest_archi_B.ply", *pts_archi); // output PLY (Binary) file
    std::cout << "OUTPUT: POINTCLOUDS_rest_archi_B.ply" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr result_archi (new pcl::PointCloud<pcl::PointNormal>);
    mls_smooth(pts_archi, result_archi);
    pt2mesh("./data/meshes/MESH_rest_archi.ply", result_archi); // output mesh file
    std::cout << "OUTPUT: MESH_rest_archi.ply" << std::endl;

    // output non-architecture part
    pcl::io::savePLYFileASCII("./data/pointclouds/POINTCLOUDS_rest_nonarchi_A.ply", *pts_non_archi_out); // output PLY (ASCII) file
    std::cout << "OUTPUT: POINTCLOUDS_rest_nonarchi_A.ply" << std::endl;
    pcl::io::savePLYFileBinary("./data/pointclouds/POINTCLOUDS_rest_nonarchi_B.ply", *pts_non_archi_out); // output PLY (Binary) file
    std::cout << "OUTPUT: POINTCLOUDS_rest_nonarchi_B.ply" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr result_non_archi (new pcl::PointCloud<pcl::PointNormal>);
    mls_smooth(pts_non_archi_out, result_non_archi);
    pt2mesh("./data/meshes/MESH_rest_nonarchi.ply", result_non_archi); // output mesh file
    std::cout << "OUTPUT: MESH_rest_nonarchi.ply" << std::endl;
}

void write_pts(std::string& path_A, std::string& path_B, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pts){
    pcl::io::savePLYFileASCII(path_A, *pts); // output PLY (ASCII) file
    pcl::io::savePLYFileBinary(path_B, *pts); // output PLY (Binary) file
}



