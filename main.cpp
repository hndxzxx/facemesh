#include <iostream>
#include <sstream>
#include <vector>
#include "trimesh.h" 
#include "util.h"
#include "octree.h"
#include "ray_triangle.h"
#include <string>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

int main()
{

  
  std::string filename = "/mnt/lustre/zengxiaoxing/learning_code/mesh/data/116_18_grin_reg_scan.obj";
  std::string scan_trans_path = "/mnt/lustre/zengxiaoxing/learning_code/mesh/data/116_18_grin_scan_trans.obj";
  trimesh mesh1;
  mesh1.load(filename);
  trimesh mesh2;
  mesh2.load(scan_trans_path);
  mesh2.cal_vert_normal(mesh2.vertices, mesh2.faces);
  //std::string save_path = "trimesh.obj";
  //mesh1.save(save_path);
  Eigen::MatrixXd vert_normal(mesh1.vertices.size(),3);
  std::cout << "vert_normal计算开始" << endl;
  mesh1.cal_vert_normal(mesh1.vertices, mesh1.faces);
  mesh1.cal_face_normal(mesh1.vertices, mesh1.faces);
  std::cout << "vert_normal计算完毕" << endl;

  

  
  return 0;

}

