#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <vector>
#include <map>

class Cell{
private:
    double value = 0;
public:
    std::vector<pcl::PointXYZI*> pt_vec = std::vector<pcl::PointXYZI*>();
    void set_val(double val) {value = val;}
    double get_val() const {return value;}
};

class Grid{
private:
    unsigned int col_num;
    unsigned int row_num;
    double x_min;
    double y_min;
    double size;
    std::map<std::pair<unsigned int, unsigned int>, Cell> grid_cell;

public:
    Grid(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts, double _size){
        size = _size;
        double x_max, y_max;
        for(int i = 0; i < pts->size(); ++i){
            if(i == 0){
                x_min = pts->points[i].x;
                x_max = pts->points[i].x;
                y_min = pts->points[i].y;
                y_max = pts->points[i].y;
            }
            else{
                if(pts->points[i].x > x_max) x_max = pts->points[i].x;
                else if (pts->points[i].x < x_min) x_min = pts->points[i].x;
                if(pts->points[i].y > y_max) y_max = pts->points[i].y;
                else if (pts->points[i].y < y_min) y_min = pts->points[i].y;
            }
        }

        col_num = int((x_max - x_min) / size) + 1;
        row_num = int((y_max - y_min) / size) + 1;

        for (auto &pt: pts->points){
            unsigned int col = int((pt.x - x_min) / size);
            unsigned int row = int((pt.y - y_min) / size);
            grid_cell[{row, col}].pt_vec.push_back(&pt);
        }
    }

    unsigned int get_col_num() const{return col_num;}

    unsigned int get_row_num() const{return row_num;}

    double get_x_min() const{return x_min;}

    double get_y_min() const{return y_min;}

    double get_size() const{return size;}

    Cell* find_cell_by_coord(double x, double y){
        unsigned int col = int((x - x_min) / size);
        unsigned int row = int((y - y_min) / size);
        if (col >= col_num || row >= row_num) assert(false);
        return &grid_cell[{row, col}];
    }

    Cell* find_cell_by_idx(unsigned int row, unsigned int col){
        if (col >= col_num || row >= row_num) assert(false);
        return &grid_cell[{row, col}];
    }

    std::pair<double, double> get_cell_center(unsigned int row, unsigned int col) const{
        double cc_x = x_min + 0.5 * size + col * size;
        double cc_y = y_min + 0.5 * size + row * size;
        return std::pair<double, double>{cc_x, cc_y};
    }

    std::vector<double> get_cell_center_3d(unsigned int row, unsigned int col) {
        if (col >= col_num || row >= row_num) assert(false);
        double cc_x = x_min + 0.5 * size + col * size;
        double cc_y = y_min + 0.5 * size + row * size;
        return std::vector<double>{cc_x, cc_y, grid_cell[{row, col}].get_val()};
    }
};

template<typename t>
double max_vec_elm(std::vector<t> vec){
    double max_val;
    for (int i = 0; i < vec.size(); ++i) {
        if (i == 0) max_val = vec[0];
        else if (vec[i] > max_val) max_val = vec[i];
    }
    return max_val;
}

template<typename t>
double min_vec_elm(std::vector<t> vec){
    double min_val;
    for (int i = 0; i < vec.size(); ++i) {
        if (i == 0) min_val = vec[0];
        else if (vec[i] < min_val) min_val = vec[i];
    }
    return min_val;
}

template<typename t>
double avg_vec_elm(std::vector<t> vec){
    double avg_val = 0;
    for (int i = 0; i < vec.size(); ++i) {
        avg_val += vec[i];
    }
    return avg_val / vec.size();
}

template<typename t>
double mean_sqr_err(std::vector<t> vec){
    double avg_val = avg_vec_elm(vec);
    double err_val = 0;
    for (int i = 0; i < vec.size(); ++i) {
        err_val += abs(avg_val - vec[i]);
    }
    return err_val / vec.size();
}