#ifndef __DEFAULT_PASS_H__
#define __DEFAULT_PASS_H__

#include "../app/camera.h"

bool default_pass_init(float* positions, float* normals, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes, dm_ecs_id light_component_id, view_camera* camera);
void default_pass_set_light_pos(dm_vec3 pos);

#endif //DEFAULT_PASS_H
