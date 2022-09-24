#include <iostream>
#include <sstream>
#include <vector>
#include "trimesh.h" 
#include "util.h"
#include "octree.h"
#include "ray_triangle.h"
#include <string>
#include <Eigen/Dense>
# haha
using namespace std;
using namespace Eigen;
# 尝试merge
# 本地参考修改在同一个文件中
$ merge
# 远程修改在同一个文件中
# 远程仓库得文件名称改变了
int main()
{
  
  std::string filename = "C:/Users/xxzeng/Desktop/Anaconda/C++/mesh/data/116_18_grin_reg_scan.obj";
  std::string scan_trans_path = "C:/Users/xxzeng/Desktop/Anaconda/C++/mesh/data/116_18_grin_scan_trans_simplified.obj";
  trimesh mesh1;
  mesh1.load(filename);
  trimesh mesh2;
  mesh2.load(scan_trans_path);
  mesh2.cal_vert_normal(mesh2.vertices, mesh2.faces);
  //std::string save_path = "trimesh.obj";
  //mesh1.save(save_path);
  Eigen::MatrixXd vert_normal(mesh1.vertices.size(),3);
  mesh1.cal_vert_normal(mesh1.vertices, mesh1.faces);
  mesh1.cal_face_normal(mesh1.vertices, mesh1.faces);
  
  // the ray-triangle intersection only consider one direction
  // if we want to get all intersection, we need give the ray-triangle twice
  // one is mesh1.vert_normal, the other is mesh1.vert_normal*-1
  
  std::cout << "?????" << endl;
  int iters = 1;
  auto mesh_subdiv = subdiv(mesh1, iters); // face mesh subdivison with mid-points
  //mesh_subdiv.cal_vert_normal(mesh_subdiv.vertices, mesh_subdiv.faces);

  std::vector<std::vector<double>> inter_pts;
  std::vector<std::vector<int>> inter_face_idx;
  std::vector<std::vector<int>> inter_ray_idx;

  // do ray-triangle intersection
  std::cout << "begin ray-traingle inter" <<endl;
  tie(inter_pts, inter_face_idx, inter_ray_idx) = ray_triangle(mesh_subdiv.vertices, mesh_subdiv.vert_normal, mesh2);
  
  std::cout << "end ray-traingle inter" <<endl;

  for (int j=0; j<inter_ray_idx.size(); j++)
  {
    std::vector<double> tmp_pt;
    tmp_pt.push_back(inter_pts[j][0]);
    tmp_pt.push_back(inter_pts[j][1]);
    tmp_pt.push_back(inter_pts[j][2]);
    mesh_subdiv.vertices[inter_ray_idx[j][0]][0] = inter_pts[j][0];
    mesh_subdiv.vertices[inter_ray_idx[j][0]][1] = inter_pts[j][1];
    mesh_subdiv.vertices[inter_ray_idx[j][0]][2] = inter_pts[j][2];
    
  }
  std::string save_path_detail = "trimesh_detail.obj";
  mesh_subdiv.save(save_path_detail);
  
  return 0;

}

