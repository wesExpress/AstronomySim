#ifndef __DEFAULT_PASS_H__
#define __DEFAULT_PASS_H__

#include "../app/camera.h"

bool default_pass_init(float* positions, float* normals, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes, dm_ecs_id* exclude_ids, uint32_t num_excludes, view_camera* camera);

void default_pass_add_point_light(dm_entity entity);
void default_pass_add_blackbody(dm_entity entity);

void default_pass_set_light_component_id(dm_ecs_id id);
void default_pass_set_blackbody_component_id(dm_ecs_id id);

#endif //DEFAULT_PASS_H
