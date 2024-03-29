#ifndef GRAVITY_SYSTEM_H
#define GRAVITY_SYSTEM_H

#include "dm.h"

bool gravity_system_init(dm_ecs_id t_id, dm_ecs_id p_id, dm_ecs_system_timing timing, dm_ecs_id* sys_id, dm_context* context);
void gravity_system_shutdown(void* s, void* c);
bool gravity_system_run(void* s, void* c);
void gravity_system_insert(const uint32_t entity_index, void* s, void* c);

double gravity_system_get_timing(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);

#endif //GRAVITY_SYSTEM_H
