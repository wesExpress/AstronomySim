#ifndef DEBUG_RENDER_PASS_H
#define DEBUG_RENDER_PASS_H

#include "dm.h"

bool debug_render_pass_init(dm_context* context);
void debug_render_pass_shutdown(dm_context* context);
bool debug_render_pass_render(dm_context* context);

// rendering funcs
void debug_render_line(const float pos_0[3], const float pos_1[3], const float color[4], dm_context* context);
void debug_render_arrow(const float pos_0[3], const float pos_1[3], const float color[4], dm_context* context);
void debug_render_bilboard(const float pos[3], const float width, const float height, const float color[4], dm_context* context);
void debug_render_aabb(const float pos[3], const float dim[3], const float color[4], dm_context* context);
void debug_render_cube(const float pos[3], const float dim[3], const float orientation[4], const float color[4], dm_context* context);

#endif //DEBUG_RENDER_PASS_H