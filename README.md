# this repo mainly focus on the mesh processing algorithms of faces.

## mesh.h/mesh.cpp
1.load(), save(): mesh load/save function<br>
2.cal_vert_normal(): calculating vertices normal just with averaging(non area weighting)<br>
3.subdiv_mid(): middle point subdivision method<br>
4.del_vert_by_idx(): delete mesh's vertices by its index<br>
5.del_vt_by_idx(): delete mesh's texture coordinates by its index<br>

## ray-triangle.h
1.ray_triangle(): do ray triangle intersecting<br>
2. adding openmp supporting. It gives 6 times faster, when the ray size is 200k, and the target triangle size is 1000k.


## octree.h
1. class of fast data structure: octree. borrow from https://github.com/szat/Octree_SDF<br>

## utils.h
1.read_data_from_txt(): read index from .txt files.<br>
2.remove_duplicate_faces(): remove the duplicates faces, it is a must taken preprocess in ray-triangle.<br>
