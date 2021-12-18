#include "trimesh.h"
#include <fstream>
#include <iostream> 
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>    // std::transform
#include<stack>
#include <iterator>
#include <set>
#include <map>

using namespace std;
using std::ofstream;
typedef std::map<std::string, int> BasePairMap;
typedef std::map<int, int> IndexValueMap;

void trimesh::load(const std::string &filename)
{
  std::ifstream file(filename);
  if (file.is_open()) 
  {
      std::string line;
      while (std::getline(file, line)) 
      {
          // using printf() in all tests for consistency
          //printf("%s", line.c_str());
            istringstream in(line);
            std::string type;
            in >> type;
          
            if ( type=="v")
            {
                std::vector<double> vertex={0,0,0};
                std::vector<double> vert_color_single={0,0,0};
                in >> vertex[0] >> vertex[1] >> vertex[2];                
                vertices.push_back(vertex);
                string line_s;
                vector<string> vec_line;
                while (in >> line_s)
                {
                    vec_line.push_back(line_s);
                }
                if (vec_line.size()==3)
                {
                    vert_color_single[0] = std::stod(vec_line[0]);
                    vert_color_single[1] = std::stod(vec_line[1]);
                    vert_color_single[2] = std::stod(vec_line[2]);
                    vert_color.push_back(vert_color_single);
                }
                
                
            }
            else if(type == "vt")
            {
                std::vector<double> tex_coord={0,0};
                in >> tex_coord[0] >> tex_coord[1];
                if (!in) 
                {
                    std::cerr << "Reading vt failed!\n";
                }
                tex_coords.push_back(tex_coord);
                
            }
            else if(type== "f")
            {
                char slash_a, slash_b, slash_c;
                std::vector< int> face = {0,0,0};
                std::vector< int> face_tc = {0,0,0};
                in >> face[0] >> slash_a >> face_tc[0];
                in >> face[1] >> slash_b >> face_tc[1];
                in >> face[2] >> slash_c >> face_tc[2];

                if (!in || slash_a != '/' || slash_b != '/' || slash_c != '/') 
                { 
                    std::cerr << "Reading faces failed!\n";
                }
                face[0] -= 1;
                face[1] -= 1;
                face[2] -= 1;
                faces.push_back(face);
                face_tc[0] -= 1;
                face_tc[1] -= 1;
                face_tc[2] -= 1;
                faces_tc.push_back(face_tc);
                
            }
                
           }
      
       }
    
   else
    {
        std::cerr << "Loading obj file failed: " << filename << " does not exist!\n";
    }

   if (file.eof())                      
    std::cout << "Reading " << filename << " successfully!\n";
   else
    std::cout << "Failed " << filename << " !\n";
   file.close();
 
}





void trimesh::save(const std::string &save_path, const bool with_color, const bool with_vt)
{
  ofstream outdata;
  outdata.open(save_path);
  if( !outdata ) 
  { // file couldn't be opened
      std::cout << "Error: file could not be opened" << std::endl;
      exit(1);
  }

  if (with_color && (vertices.size()==vert_color.size())) //write the vertices and colors
  {
    for (size_t i=0; i<vertices.size(); i++)
    {
    outdata << "v ";
    outdata << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << " " << vert_color[i][0] << " " << vert_color[i][1] << " " << vert_color[i][2]  << '\n';
    }  
  }
  else// only write the vertices
  {
    for (size_t i=0; i<vertices.size(); i++)
    {
      outdata << "v ";
      outdata << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2]<< '\n';
    }
    
  }

  if (with_vt)
  {
    for (size_t j=0; j<tex_coords.size(); j++)
    {
      outdata << "vt ";
      outdata << tex_coords[j][0] << " " << tex_coords[j][1] << '\n';
    }
  }

  if (faces.size()==faces_tc.size())
  {
    for (size_t k=0; k<faces.size(); k++)
    {
     int f0 = faces[k][0] + 1;
     int f1 = faces[k][1] + 1;
     int f2 = faces[k][2] + 1;

     int ftc0 = faces_tc[k][0] + 1;
     int ftc1 = faces_tc[k][1] + 1;
     int ftc2 = faces_tc[k][2] + 1;

    outdata << "f ";
    outdata << f0  << "/" << ftc0  << " " << f1  << "/" << ftc1 << " " << f2 << "/" << ftc2   << '\n';
    }  

  }
  else
  {
    for (size_t k=0; k<faces.size(); k++)
    {
    
     int f0 = faces[k][0] + 1;
     int f1 = faces[k][1] + 1;
     int f2 = faces[k][2] + 1;
    outdata << "f ";
    outdata << f0  << " " << f1 << " " << f2   <<'\n';
    }  
  }

  outdata.close();
}


Eigen::MatrixXd ConvertToEigenMatrix_double(std::vector<std::vector<double>> data)
{
    Eigen::MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

Eigen::MatrixXi ConvertToEigenMatrix_int(std::vector<std::vector< int>> data)
{
    Eigen::MatrixXi eMatrix;
    eMatrix.resize(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
    {
        eMatrix.row(i) = VectorXi::Map(&data[i][0], data[0].size());
    }
    return eMatrix;
}


void trimesh::cal_vert_normal(const std::vector<std::vector<double>> &vertices, const std::vector<std::vector< int>> &faces)
{
  Eigen::MatrixXd vert_e = ConvertToEigenMatrix_double(vertices);
  Eigen::MatrixXi face_e = ConvertToEigenMatrix_int(faces);
  
  //vert_normal.resize(vertices.size(), vertices[0].size());
  Eigen::MatrixXd vert_normal_eigen;
  vert_normal_eigen = Eigen::MatrixXd::Zero(vertices.size(), vertices[0].size());
  

  for (int i=0; i<faces.size(); i++)
  {
    Eigen::Vector3d edge10 = vert_e.row(face_e(i,1)) - vert_e.row(face_e(i,0));
    Eigen::Vector3d edge20 = vert_e.row(face_e(i,2)) - vert_e.row(face_e(i,0));
    Eigen::Vector3d per_face_normal = edge10.cross(edge20);
    vert_normal_eigen.row(face_e(i,0)) += per_face_normal;
    vert_normal_eigen.row(face_e(i,1)) += per_face_normal;
    vert_normal_eigen.row(face_e(i,2)) += per_face_normal;
  }
  
  vert_normal_eigen.rowwise().normalize();
  for (int j=0; j<vert_normal_eigen.rows(); j++)
  {
    std::vector<double> tmp;
    tmp.push_back(vert_normal_eigen(j,0));
    tmp.push_back(vert_normal_eigen(j,1));
    tmp.push_back(vert_normal_eigen(j,2));
    vert_normal.push_back(tmp);
  }

}


void trimesh::cal_face_normal(const std::vector<std::vector<double>> &vertices, const std::vector<std::vector< int>> &faces)
{
  Eigen::MatrixXd vert_e = ConvertToEigenMatrix_double(vertices);
  Eigen::MatrixXi face_e = ConvertToEigenMatrix_int(faces);
  
  //vert_normal.resize(vertices.size(), vertices[0].size());
  face_normal = Eigen::MatrixXd::Zero(faces.size(), faces[0].size());
  
  for (int i=0; i<faces.size(); i++)
  {
    Eigen::Vector3d edge10 = vert_e.row(face_e(i,1)) - vert_e.row(face_e(i,0));
    Eigen::Vector3d edge20 = vert_e.row(face_e(i,2)) - vert_e.row(face_e(i,0));
    Eigen::Vector3d per_face_normal = edge10.cross(edge20);
    face_normal.row(i) = per_face_normal;
  }
  
  face_normal.rowwise().normalize();

}

template <typename t>
std::vector<std::vector<t> > GetUniqueRows(std::vector<std::vector<t> > input)
{
    std::sort(input.begin(), input.end());
    input.erase(std::unique(input.begin(), input.end()), input.end());
    return input;
}



std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> trimesh::subdiv_mid()
{
  
    Eigen::MatrixXd vert_e = ConvertToEigenMatrix_double(vertices);
    Eigen::MatrixXi face_e = ConvertToEigenMatrix_int(faces);
    Eigen::MatrixXi edges_matrix = Eigen::MatrixXi::Zero(faces.size()*3, 2);
    Eigen::ArrayXXi edges_array;
    edges_array.resize(faces.size()*3, 2);
    edges_array.block(0, 0, faces.size(), 1) = face_e.col(0);
    edges_array.block(0, 1, faces.size(), 1) = face_e.col(1);
    edges_array.block(faces.size(), 0, faces.size(), 1) = face_e.col(1);
    edges_array.block(faces.size(), 1, faces.size(), 1) = face_e.col(2);
    edges_array.block(faces.size()*2, 0, faces.size(), 1) = face_e.col(2);
    edges_array.block(faces.size()*2, 1, faces.size(), 1) = face_e.col(0);

    
    edges_matrix = edges_array.matrix();
    for (size_t i=0; i<edges_matrix.rows(); i++)
    {
      if (edges_matrix(i, 0) > edges_matrix(i, 1))
      {
        std::swap(edges_matrix(i,0), edges_matrix(i, 1));
      }
    }
    // 通过HashMap来减少计算量。
    BasePairMap edge_map;
    std::vector<int> inverse; 
    std::vector<std::vector<int>> index_real;
    int flag = 0;
    int vert_num = vert_e.rows();
    for (size_t j=0; j<edges_matrix.rows(); j++)
    {
      std::string fir = std::to_string(edges_matrix(j,0));
      std::string sec = std::to_string(edges_matrix(j,1));
      fir = fir + "_" + sec;
      if (edge_map.count(fir)==0)
      {
        int a = flag + vert_num;
        edge_map.insert(std::pair<string, int>(fir, a)); 
        
        inverse.push_back(a);
        std::vector<int> tmp = {edges_matrix(j,0), edges_matrix(j,1)};
        index_real.push_back(tmp);
        flag+=1;
      }
      else
      { 
        int a= edge_map.find(fir)->second;
        inverse.push_back(a);
      }

    }
    
    Eigen::MatrixXd mid_points;
    mid_points.resize(index_real.size(),3);
    for (size_t m=0; m<index_real.size(); m++)
    {
      auto pt1_idx = index_real[m][0];
      auto pt2_idx = index_real[m][1];
      mid_points.row(m) = vert_e.row(pt1_idx)*0.5 + vert_e.row(pt2_idx)*0.5;

    }

    size_t inverse_row = inverse.size()/3;
    
    Eigen::MatrixXi mid_faces;
    mid_faces.resize(inverse_row,3);
    for (size_t n=0; n<inverse_row; n++)
    {
      int small_1;
      int big_1;
      if (face_e(n,0)<face_e(n,1))
      {
        small_1 = face_e(n,0);
        big_1   = face_e(n,1);
      }
      else
      {
        small_1 = face_e(n,1);
        big_1   = face_e(n,0);
      }
      std::string fir_1 = std::to_string(small_1);
      std::string sec_1 = std::to_string(big_1);
      fir_1 = fir_1 + "_" + sec_1;
      mid_faces(n,0) =  edge_map.find(fir_1)->second;

      //std::cout << "inverse可视化：" << inverse_reshape[n][0] << endl;
      int small_2;
      int big_2;
      if (face_e(n,1)<face_e(n,2))
      {
        small_2 = face_e(n,1);
        big_2   = face_e(n,2);
      }
      else
      {
        small_2 = face_e(n,2);
        big_2   = face_e(n,1);
      }
      std::string fir_2 = std::to_string(small_2);
      std::string sec_2 = std::to_string(big_2);
      fir_2 = fir_2 + "_" + sec_2;
      mid_faces(n,1) =  edge_map.find(fir_2)->second;
      
      // third
      int small_3;
      int big_3;
      if (face_e(n,2)<face_e(n,0))
      {
        small_3 = face_e(n,2);
        big_3   = face_e(n,0);
      }
      else
      {
        small_3 = face_e(n,0);
        big_3   = face_e(n,2);
      }
      std::string fir_3 = std::to_string(small_3);
      std::string sec_3 = std::to_string(big_3);
      fir_3 = fir_3 + "_" + sec_3;
      mid_faces(n,2) =  edge_map.find(fir_3)->second;

    }

    Eigen::ArrayXXd vertices_subdiv;
    int new_num_vert  = vert_e.rows() + mid_points.rows();
    vertices_subdiv.resize(new_num_vert, 3);

    vertices_subdiv.block(0,0, vert_e.rows(),3) = vert_e;
    vertices_subdiv.block(vert_e.rows(),0, mid_points.rows(),3) = mid_points;

    Eigen::ArrayXXi faces_subdiv;
    faces_subdiv.resize(face_e.rows()*4, 3);
    faces_subdiv.block(0,0, face_e.rows(), 1) = face_e.col(0);
    faces_subdiv.block(0,1, face_e.rows(), 1) = mid_faces.col(0);
    faces_subdiv.block(0,2, face_e.rows(), 1) = mid_faces.col(2);
    
    faces_subdiv.block(face_e.rows(), 0, face_e.rows(), 1) = mid_faces.col(0);
    faces_subdiv.block(face_e.rows(), 1, face_e.rows(), 1) = face_e.col(1);
    faces_subdiv.block(face_e.rows(), 2, face_e.rows(), 1) = mid_faces.col(1);
    
    faces_subdiv.block(face_e.rows()*2, 0, face_e.rows(), 1) = mid_faces.col(2);
    faces_subdiv.block(face_e.rows()*2, 1, face_e.rows(), 1) = mid_faces.col(1);
    faces_subdiv.block(face_e.rows()*2, 2, face_e.rows(), 1) = face_e.col(2);

    faces_subdiv.block(face_e.rows()*3, 0, face_e.rows(), 1) = mid_faces.col(0);
    faces_subdiv.block(face_e.rows()*3, 1, face_e.rows(), 1) = mid_faces.col(1);
    faces_subdiv.block(face_e.rows()*3, 2, face_e.rows(), 1) = mid_faces.col(2);
    
    for (size_t s=0; s<face_e.rows(); s++)
    {
      std::vector<int> tmp = {face_e(s,0), };
    }
    std::vector<std::vector<double>> new_vert;
    std::vector<std::vector<int>> new_face;
    for (size_t p=0; p<vertices_subdiv.rows(); p++)
    {
      std::vector<double> tmp;
      tmp.push_back(vertices_subdiv(p,0));
      tmp.push_back(vertices_subdiv(p,1));
      tmp.push_back(vertices_subdiv(p,2));
      new_vert.push_back(tmp);
    }

    for (size_t q=0; q<faces_subdiv.rows(); q++)
    {
      std::vector<int> tmp;
      tmp.push_back(faces_subdiv(q,0));
      tmp.push_back(faces_subdiv(q,1));
      tmp.push_back(faces_subdiv(q,2));
      new_face.push_back(tmp);
    }
  
  return std::make_tuple(new_vert, new_face);
  

}



std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> trimesh::del_vert_by_idx(const std::vector<int> & vert_index)
{
  IndexValueMap vert_idx_map;
  for (int i=0; i<vertices.size(); i++)
  {
    vert_idx_map.insert(std::pair<int, int>(i, -1));
  }
  
  std::vector<std::vector<double>> new_vertices;
  std::vector<std::vector<double>> new_vt;
  std::vector<std::vector<int>> new_faces;
  std::vector<std::vector<int>> new_faces_tc;
  for (int j=0; j< vert_index.size(); j++)
  {
    std::vector<double> tmp_vert;
    tmp_vert.push_back(vertices[vert_index[j]][0]);
    tmp_vert.push_back(vertices[vert_index[j]][1]);
    tmp_vert.push_back(vertices[vert_index[j]][2]);
    new_vertices.push_back(tmp_vert);

    vert_idx_map.find(vert_index[j])->second = j;

  }

  for (int k=0; k<faces.size(); k++)
  {
    if ((vert_idx_map.find(faces[k][0])->second!=-1 )&&(vert_idx_map.find(faces[k][1])->second!=-1)&&(vert_idx_map.find(faces[k][2])->second!=-1))
    {
      auto v_idx_1 = vert_idx_map.find(faces[k][0])->second;
      auto v_idx_2 = vert_idx_map.find(faces[k][1])->second;
      auto v_idx_3 = vert_idx_map.find(faces[k][2])->second;
      std::vector<int> tmp;
      tmp.push_back(v_idx_1);
      tmp.push_back(v_idx_2);
      tmp.push_back(v_idx_3);
      new_faces.push_back(tmp);
    }

  }

  return std::make_tuple(new_vertices, new_faces);

}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> trimesh::del_vt_by_idx(const std::vector<int> & vert_index)
{
  IndexValueMap vert_idx_map;
  for (int i=0; i<tex_coords.size(); i++)
  {
    vert_idx_map.insert(std::pair<int, int>(i, -1));
  }
  
  std::vector<std::vector<double>> new_vt;
  std::vector<std::vector<int>> new_faces_tc;
  for (int j=0; j< vert_index.size(); j++)
  {
    std::vector<double> tmp_vert;
    tmp_vert.push_back(tex_coords[vert_index[j]][0]);
    tmp_vert.push_back(tex_coords[vert_index[j]][1]);
    new_vt.push_back(tmp_vert);

    vert_idx_map.find(vert_index[j])->second = j;

  }

  for (int k=0; k<faces_tc.size(); k++)
  {
    if ((vert_idx_map.find(faces_tc[k][0])->second!=-1 )&&(vert_idx_map.find(faces_tc[k][1])->second!=-1)&&(vert_idx_map.find(faces_tc[k][2])->second!=-1))
    {
      auto v_idx_1 = vert_idx_map.find(faces_tc[k][0])->second;
      auto v_idx_2 = vert_idx_map.find(faces_tc[k][1])->second;
      auto v_idx_3 = vert_idx_map.find(faces_tc[k][2])->second;
      std::vector<int> tmp;
      tmp.push_back(v_idx_1);
      tmp.push_back(v_idx_2);
      tmp.push_back(v_idx_3);
      new_faces_tc.push_back(tmp);
    }

  }

  return std::make_tuple(new_vt, new_faces_tc);

}

/*
def del_by_vert_ind(self, vert_ind):
		indexer = np.full(self.vert_num(), -1)
		indexer[vert_ind] = np.arange(0, vert_ind.shape[0])

		self.vertices = self.vertices[vert_ind]
		face_mask = np.logical_and(np.logical_and(np.in1d(self.faces[:, 0], vert_ind), np.in1d(self.faces[:, 1], vert_ind)), np.in1d(self.faces[:, 2], vert_ind))
		new_faces = self.faces[face_mask]
		self.faces = indexer[new_faces]

		if not self.vert_color is None:
			self.vert_color = self.vert_color[vert_ind]
		if not self.vert_normal is None:
			self.vert_normal = self.vert_normal[vert_ind]
	
  */