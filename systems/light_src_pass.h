#ifndef __LIGHT_SRC_PASS_H__
#define __LIGHT_SRC_PASS_H__

#include "../app/camera.h"

bool light_src_pass_init(float* positions, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes, dm_ecs_id id, view_camera* camera);

#endif //LIGHT_SRC_PASS_H
