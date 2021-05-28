# include "preprocess_methods.cpp"

void preprocess_pts(std::string& file_in,
                    int& roof_idx,
                    int& ground_idx){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_raw (new pcl::PointCloud<pcl::PointXYZI>);
    // -----------------------------------------------------------------------------------------------------------------
    // read pts from dataset
    read_pts(file_in, pts_raw);
    // -----------------------------------------------------------------------------------------------------------------
    // remove the outliers
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    remove_outliers(pts_raw, pts_filtered, 0.1, 5);
    // -----------------------------------------------------------------------------------------------------------------
    // remove fake faces
    double floor_height;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_no_fake_faces (new pcl::PointCloud<pcl::PointXYZI>);
    remove_fake_faces(pts_filtered, pts_no_fake_faces, floor_height, roof_idx, ground_idx, 1.4);
    // -----------------------------------------------------------------------------------------------------------------
    // segment the roof and floor components
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_rest (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_roof (new pcl::PointCloud<pcl::PointXYZI>);
    extract_floors(pts_no_fake_faces, pts_roof, pts_rest, roof_idx, ground_idx, 0.1, 5);
    // -----------------------------------------------------------------------------------------------------------------
    // segment the non-architecture components
    extract_non_archi(pts_roof, pts_rest, 0.05, 0.1, 10);
}
