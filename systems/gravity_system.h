#ifndef GRAVITY_SYSTEM_H
#define GRAVITY_SYSTEM_H

#include "dm.h"

bool gravity_system_init(dm_ecs_id t_id, dm_ecs_id p_id, dm_context* context);
void gravity_system_shutdown(void* s, void* c);
bool gravity_system_run(void* s, void* c);

#endif //GRAVITY_SYSTEM_H
