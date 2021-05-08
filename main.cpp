#include <iostream>
#include "preprocess.cpp"

int main() {
    // -----------------------------------------------------------------------------------------------------------------
    // define global variables
    int roof_idx;
    int ground_idx;
    std::string file_in = "../data/VRR.pcd"; // file name of the input PCD file
    std::string file_out_A = "../data/VRR_out_A.ply"; // file name of the output PLY (ASCII) file
    std::string file_out_B = "../data/VRR_out_B.ply"; // file name of the output PLY (Binary) file

    // -----------------------------------------------------------------------------------------------------------------
    // preprocess the point cloud data
    preprocess_pts(file_in, roof_idx, ground_idx);

    // -----------------------------------------------------------------------------------------------------------------
    // visualize the point cloud data
    // TODO: visualize_pts();

    // -----------------------------------------------------------------------------------------------------------------
    // the other visualization methods
    // TODO: voxelization;
    // TODO: meshing;
}
