#include <iostream>
#include <sstream>
#include <vector>
#include "trimesh.h" 
#include "util.h"
#include <string>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{

  std::cout<< argc << std::endl;  
  std::cout<< argv[1] << std::endl;
  std::string filename = "../data/head_cleaned_align_normal.obj";
  trimesh mesh1;
  mesh1.load(filename);
  std::string save_path = "trimesh.obj";
  mesh1.save(save_path);
  Eigen::MatrixXd vert_normal(mesh1.vertices.size(),3);
  std::cout << "vert_normal计算开始" << endl;
  mesh1.cal_vert_normal(mesh1.vertices, mesh1.faces);
  mesh1.cal_face_normal(mesh1.vertices, mesh1.faces);
  std::cout << "vert_normal计算完毕" << endl;
  std::vector<std::vector<double>> vertices_remain;
  std::vector<std::vector<double>> tex_coords_remain;
  std::vector<std::vector<int>> faces_remain;
  std::vector<std::vector<int>> faces_tc_remain;

  
  std::string txt_path = "/mnt/lustre/zengxiaoxing/learning_code/face_registration-master/region/region_in_head.txt";
  auto index = read_data_from_txt(txt_path);
  tie(vertices_remain, faces_remain) = mesh1.del_vert_by_idx(index);
  tie(tex_coords_remain, faces_tc_remain) = mesh1.del_vt_by_idx(index);
  trimesh mesh2;
  mesh2.vertices = vertices_remain;
  mesh2.faces    = faces_remain;
  mesh2.tex_coords = tex_coords_remain;
  mesh2.faces_tc = faces_tc_remain;
  std::string save_path_del = "trimesh_delete.obj";
  mesh2.save(save_path_del);

  return 0;

}

