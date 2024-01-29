#ifndef __DEBUG_RENDER_PASS_H__
#define __DEBUG_RENDER_PASS_H__

#include "dm.h"

bool debug_render_pass_init(void** data, dm_context* context);
void debug_render_pass_shutdown(void** data, dm_context* context);

bool debug_render_pass_render(void** data, const dm_mat4 view_proj, dm_context* context);
void debug_render_line(const dm_vec3 pos_0, const dm_vec3 pos_1, const dm_vec4 color, void** data, dm_context* context);
void debug_render_bilboard(const dm_vec3 pos, const float width, const float height, const dm_vec4 color, const dm_mat4 inv_view, void** data, dm_context* context);
void debug_render_arrow(const dm_vec3 pos_0, const dm_vec3 pos_1, const dm_vec4 color, const dm_mat4 inv_view, void** data, dm_context* context);
void debug_render_aabb(const dm_vec3 pos, const dm_vec3 dim, const dm_vec4 color, void** data, dm_context* context);
void debug_render_cube(const dm_vec3 pos, const dm_vec3 dim, const dm_quat orientation, const dm_vec4 color, void** data, dm_context* context);


#endif
