#ifndef TRIMESH_H
#define TRIMESH_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class trimesh {
    public:
    std::vector<std::vector<double>> vertices; 
    std::vector<std::vector<double>> tex_coords;
    std::vector<std::vector<double>> vert_color;
    std::vector<std::vector<int>> faces;
    std::vector<std::vector<int>> faces_tc; 
    std::vector<std::vector<double>> vert_normal;
    Eigen::MatrixXd face_normal;
    void load(const std::string &filename);// load .obj file from given path
    void save(const std::string &save_path, const bool with_color = true, const bool with_vt = true); // save the current mesh to the given save_path
    void cal_vert_normal(const std::vector<std::vector<double>> &vertices, const std::vector<std::vector< int>> &faces);
    void cal_face_normal(const std::vector<std::vector<double>> &vertices, const std::vector<std::vector< int>> &faces);
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> subdiv_mid();
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> del_vert_by_idx(const std::vector<int> &vert_index);
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> del_vt_by_idx(const std::vector<int> &vert_index);
};



#endif

