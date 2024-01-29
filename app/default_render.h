#ifndef default_render_h
#define default_render_h

#include "dm.h"

bool default_render_init(const uint32_t array_length, void** data, dm_context* context);
void default_render_shutdown(void** data, dm_context* context);
bool default_render_render(const uint32_t array_length, const dm_mat4 view_proj, const dm_mat4 inv_view, const float* pos_x, const float* pos_y, const float* pos_z, void** data, dm_context* context);

#endif
