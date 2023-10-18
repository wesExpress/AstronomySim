#ifndef RENDER_PASS_H
#define RENDER_PASS_H

#define NUM_PLANETS 50
#include "dm.h"

bool render_pass_init(dm_context* context);
void render_pass_shutdown(dm_context* context);
void render_pass_submit_entity(dm_entity entity, uint32_t mesh_id, dm_context* context);
bool render_pass_render(dm_context* context);

#endif //RENDER_PASS_H
