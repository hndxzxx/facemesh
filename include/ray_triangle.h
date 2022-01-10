#include "octree.h"
#include "trimesh.h"
#include "util.h"
#include <omp.h>
#include<time.h>

template <typename T>
std::vector<std::array<T,3>> Vector2Array(const std::vector<std::vector<T>> & mat) {
	/*
	This function serves as a bridge between Eigen and this C++ header project.
	Input: an Eigen matrix, that NEEDS to be dynamic rows and with 3 cols. The type is templated.
	Output: a C++ vector of arrays with 3 columns. The type is templated.
	*/
	
	vector<array<T, 3>> out;
	for (size_t i = 0; i < mat.size(); i++) 
	{
		out.push_back({mat[i][0],mat[i][1],mat[i][2]});
	}
	return out;
}

std::vector<std::vector<double>> centroid(const std::vector<std::vector<double>> & verts, const std::vector<std::vector<int>> & faces)
{
	std::vector<std::vector<double>> centroid;

	for (int i=0; i<faces.size(); i++)
	{
		auto idx_v1 = faces[i][0];
		auto idx_v2 = faces[i][1];
		auto idx_v3 = faces[i][2];
		auto cent_x = (verts[idx_v1][0] + verts[idx_v2][0] + verts[idx_v3][0])/3;
		auto cent_y = (verts[idx_v1][1] + verts[idx_v2][1] + verts[idx_v3][1])/3;
		auto cent_z = (verts[idx_v1][2] + verts[idx_v2][2] + verts[idx_v3][2])/3;
		std::vector<double> tmp;
		tmp.push_back(cent_x);
		tmp.push_back(cent_y);
		tmp.push_back(cent_z);
		centroid.push_back(tmp);

	}

	return centroid;

}





std::tuple< std::vector<std::vector<double>>, std::vector<std::vector<int>>, std::vector<std::vector<int>> > ray_triangle(const std::vector<std::vector<double>> & ray_pts, const std::vector<std::vector<double>> & ray_nor, const trimesh & target_mesh)
{
	/*
	do ray-triangle intersection with octree fast structure.

	*/
	auto uniq_faces = remove_duplicate_faces(target_mesh.faces);
	auto ray_points = Vector2Array(ray_pts);
	auto ray_direction = Vector2Array(ray_nor);

	auto target_vert = Vector2Array(target_mesh.vertices);
	auto target_face = Vector2Array(uniq_faces);
	auto target_centroid = centroid(target_mesh.vertices, uniq_faces);
	auto target_cent = Vector2Array(target_centroid);

	SDF tree(target_vert, target_face, target_cent);
	tree.init();
	tree.build();
	std::vector<std::vector<double>> inter_pts(ray_pts.size(), vector<double>(3));
	std::vector<std::vector<int>> inter_idx_ray_vert(ray_pts.size(), vector<int>(1));
	std::vector<std::vector<int>> inter_idx_target_face(ray_pts.size(), vector<int>(1));
	
	int flag=0;
	clock_t startTime,endTime;
	startTime = clock();
	omp_set_num_threads(8);
	//#pragma omp parallel default(none) shared(ray_pts, ray_points,ray_direction, inter_pts,inter_idx_ray_vert,inter_idx_target_face, tree) 
	#pragma omp parallel
	{

	#pragma omp for
	for (int i=0; i<ray_pts.size(); i++)
	{
		std::array<double,3> ray_point = ray_points[i];
		std::array<double,3> ray_dir   = ray_direction[i];
		std::array<double,3> ray_dir_n = ray_direction[i];
		ray_dir_n.at(0) *=-1;
		ray_dir_n.at(1) *=-1;
		ray_dir_n.at(2) *=-1;

		std::vector<std::array<double, 3>> inter_pt;
		std::vector<array<int, 1>> inter_face_idx;
		std::vector<std::array<double, 3>> inter_pt_n;
		std::vector<array<int, 1>> inter_face_idx_n;
		tie(inter_pt, inter_face_idx) = tree.query(ray_point, ray_dir);
		tie(inter_pt_n, inter_face_idx_n) = tree.query(ray_point, ray_dir_n);
		
		//std::vector< std::array<int, 5> > vecs;
		//vecs.reserve(10);
		
		double dist = std::numeric_limits<double>::max();
		if (inter_pt.size()!=0){
			auto inter_pt_x = inter_pt[0].at(0);
			auto inter_pt_y = inter_pt[0].at(1);
			auto inter_pt_z = inter_pt[0].at(2);
			dist = std::sqrt((inter_pt_x-ray_pts[i][0])*(inter_pt_x-ray_pts[i][0])+ (inter_pt_y-ray_pts[i][1])*(inter_pt_y-ray_pts[i][1])+ (inter_pt_z-ray_pts[i][2])*(inter_pt_z-ray_pts[i][2]));
		}
		double dist_n = std::numeric_limits<double>::max();
		if (inter_pt_n.size()!=0){
			auto inter_pt_x_n = inter_pt_n[0].at(0);
			auto inter_pt_y_n = inter_pt_n[0].at(1);
			auto inter_pt_z_n = inter_pt_n[0].at(2);
			dist_n = std::sqrt((inter_pt_x_n-ray_pts[i][0])*(inter_pt_x_n-ray_pts[i][0])+ (inter_pt_y_n-ray_pts[i][1])*(inter_pt_y_n-ray_pts[i][1])+ (inter_pt_z_n-ray_pts[i][2])*(inter_pt_z_n-ray_pts[i][2]));
		
		}
		
		if (dist < 0.02&&dist<dist_n)
			{
			inter_pts[i][0] = inter_pt[0].at(0);
			inter_pts[i][1] = inter_pt[0].at(1);
			inter_pts[i][2] = inter_pt[0].at(2);
			inter_idx_target_face[i][0] = inter_face_idx[0].at(0);
			inter_idx_ray_vert[i][0] = i;
			}
		else if (dist_n < 0.02&&dist_n<=dist)
			{
			inter_pts[i][0] = inter_pt_n[0].at(0);
			inter_pts[i][1] = inter_pt_n[0].at(1);
			inter_pts[i][2] = inter_pt_n[0].at(2);
			inter_idx_target_face[i][0] = inter_face_idx_n[0].at(0);
			inter_idx_ray_vert[i][0] = i;
			}
		
		
	}
	}
	endTime = clock();
    cout << "The run time is: " <<(double)(endTime - startTime)/CLOCKS_PER_SEC << "s" << endl;
	

	return std::make_tuple(inter_pts, inter_idx_target_face, inter_idx_ray_vert);

	// inter_pts: the intersection points in target mesh
	// inter_idx_target_face: the idx of target faces
	// inter_idx_ray_vert: the idx of source points
}

