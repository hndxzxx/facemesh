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
  std::vector<std::vector<double>> vert_subdiv;
  std::vector<std::vector<int>> face_subdiv;
  tie(vert_subdiv, face_subdiv) = mesh1.subdiv_mid();
  trimesh mesh_subdiv;
  mesh_subdiv.vertices = vert_subdiv;
  mesh_subdiv.faces    = face_subdiv;
  std::string save_path_subdiv = "trimesh_subdiv.obj";
  mesh_subdiv.save(save_path_subdiv);
  return 0;

}

