#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "dm.h"

#define PHYSICS_SYSTEM_MAX_SUB_STEPS      10
#define PHYSICS_SYSTEM_CONSTRAINT_ITER    10
#define PHYSICS_SYSTEM_MAX_MANIFOLD_COUNT 2000

#ifdef DM_SIMD_x86
#define PHYSICS_SIMD_N DM_SIMD256_FLOAT_N
#elif defined(DM_SIMD_ARM)
#define PHYSICS_SIMD_N DM_SIMD_FLOAT_N
#endif

bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_ecs_id r_id, dm_context* context);
void physics_system_shutdown(void* s, void* c);
bool physics_system_run(void* s, void* c);
void physics_system_insert(const uint32_t entity_index, void* s, void* c);

#endif //PHYSICS_SYSTEM_H
