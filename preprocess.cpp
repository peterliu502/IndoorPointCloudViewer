# include "preprocess_methods.cpp"

void preprocess_pts(std::string& file_in,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& ptr_preprocessed,
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
    remove_fake_faces(pts_filtered, pts_no_fake_faces, floor_height, roof_idx, ground_idx);
    // -----------------------------------------------------------------------------------------------------------------
    // segment the architecture components
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_without_floors (new pcl::PointCloud<pcl::PointXYZI>);
    remove_floors(pts_no_fake_faces, pts_without_floors, roof_idx, ground_idx);
    ptr_preprocessed = pts_without_floors;
}
