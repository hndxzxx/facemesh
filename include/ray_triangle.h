#include "octree.h"
#include "trimesh.h"
#include "util.h"

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
	std::vector<std::vector<double>> inter_pts;
	std::vector<std::vector<int>> inter_idx_ray_vert;
	std::vector<std::vector<int>> inter_idx_target_face;
	
	int flag=0;
	for (int i=0; i<ray_pts.size(); i++)
	{
		std::array<double,3> ray_point = ray_points[i];
		std::array<double,3> ray_dir   = ray_direction[i];
		std::vector<std::array<double, 3>> inter_pt;
		std::vector<array<int, 1>> inter_face_idx;
		tie(inter_pt, inter_face_idx) = tree.query(ray_point, ray_dir);
		
		if (inter_pt.size()!=0)
		{
			
			auto inter_pt_x = inter_pt[0].at(0);
			auto inter_pt_y = inter_pt[0].at(1);
			auto inter_pt_z = inter_pt[0].at(2);
			auto dist = std::sqrt((inter_pt_x-ray_pts[i][0])*(inter_pt_x-ray_pts[i][0])+ (inter_pt_y-ray_pts[i][1])*(inter_pt_y-ray_pts[i][1])+ (inter_pt_z-ray_pts[i][2])*(inter_pt_z-ray_pts[i][2]));
			if (dist < 0.02)
			{
			std::vector<double> inter_pt_cor;
			inter_pt_cor.push_back(inter_pt_x);
			inter_pt_cor.push_back(inter_pt_y);
			inter_pt_cor.push_back(inter_pt_z);
			inter_pts.push_back(inter_pt_cor);
			std::vector<int> inter_idx_per_face;
			inter_idx_per_face.push_back(inter_face_idx[0].at(0));
			std::vector<int> inter_idx_per_ray;
			inter_idx_per_ray.push_back(i);
			inter_idx_target_face.push_back(inter_idx_per_face);
			inter_idx_ray_vert.push_back(inter_idx_per_ray);
			}
		}
		
		
		
	}



	

	return std::make_tuple(inter_pts, inter_idx_target_face, inter_idx_ray_vert);

	// inter_pts: the intersection points in target mesh
	// inter_idx_target_face: the idx of target faces
	// inter_idx_ray_vert: the idx of source points
}

