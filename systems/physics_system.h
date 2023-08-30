#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "../app/app_defines.h"
#include "dm.h"

bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_ecs_id r_id, dm_context* context);
void physics_system_shutdown(void* s, void* c);
bool physics_system_run(void* s, void* c);
void physics_system_insert(uint32_t* component_indices, uint32_t* block_indices, void* s, void* c);

#endif //PHYSICS_SYSTEM_H
