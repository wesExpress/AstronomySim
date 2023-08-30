#ifndef RENDER_PASS_H
#define RENDER_PASS_H

#include "dm.h"

bool render_pass_init(dm_context* context);
void render_pass_shutdown(dm_context* context);
void render_pass_submit_entity(dm_entity entity, dm_context* context);
bool render_pass_render(dm_context* context);

#endif //RENDER_PASS_H
