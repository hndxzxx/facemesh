#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>
#include <map>
#include "trimesh.h"
using namespace std;

//std::vector<int> read_data_from_txt(const std::string &filename);

std::vector<int> read_data_from_txt(const std::string &filename);
std::vector<std::vector<int>> remove_duplicate_faces(const std::vector<std::vector<int>> & faces);
trimesh subdiv(const trimesh & mesh, const int iters );