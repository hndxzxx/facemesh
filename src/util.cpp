#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>
#include <map>
#include <algorithm>
#include "trimesh.h"

using namespace std;
typedef std::map<std::string, int> StrVecPair;

std::vector<int> read_data_from_txt(const std::string &filename)
{
    fstream newfile;
    std::vector<int> data;
    newfile.open(filename, ios::in); //open a file to perform read operation using file object
    if (newfile.is_open())
    {   //checking whether the file is open
      string tp;
      while(getline(newfile, tp))
      {  //read data from file object and put it into string.
         data.push_back(std::stod(tp));
      }
      newfile.close();   //close the file object.
    }
    
    return data;
}

std::vector<std::vector<int>> remove_duplicate_faces(const std::vector<std::vector<int>> & faces)
{
  std::vector<std::vector<int>> uniq_faces;
  StrVecPair uniq_face_map;

  for (int i=0; i<faces.size(); i++)
  {
    std::vector<int> sort_face;
    sort_face.push_back(faces[i][0]);
    sort_face.push_back(faces[i][1]);
    sort_face.push_back(faces[i][2]);
    std::sort(sort_face.begin(), sort_face.end());

    std::string fir = std::to_string(sort_face[0]);
    std::string sec = std::to_string(sort_face[1]);
    std::string thir = std::to_string(sort_face[2]);
    std::string key;
    key = fir + "_" + sec + "_" + thir;
    if (uniq_face_map.count(key)==0)
    {
      std::vector<int> tmp_face;
      uniq_face_map.insert(std::pair<string, int>(key, i));
      tmp_face.push_back(faces[i][0]);
      tmp_face.push_back(faces[i][1]);
      tmp_face.push_back(faces[i][2]);
      uniq_faces.push_back(tmp_face);
    }
    
  }

  return uniq_faces;

}


trimesh subdiv(const trimesh & mesh, const int iters)
{
  
  trimesh new_mesh;
  new_mesh.vertices = mesh.vertices;
  new_mesh.faces    = mesh.faces;

  for (int i=0; i<=iters; i++)
  {
    std::vector<std::vector<double>> vert_subdiv_tmp;
    std::vector<std::vector<int>> face_subdiv_tmp;
    tie(vert_subdiv_tmp, face_subdiv_tmp) = new_mesh.subdiv_mid();
    new_mesh.vertices = vert_subdiv_tmp;
    new_mesh.faces    = face_subdiv_tmp;
  }

  new_mesh.cal_vert_normal(new_mesh.vertices, new_mesh.faces);
  return new_mesh;

}



