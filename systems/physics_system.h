#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "dm.h"

#define PHYSICS_SYSTEM_MAX_SUB_STEPS      10
#define PHYSICS_SYSTEM_CONSTRAINT_ITER    15

#ifdef DM_SIMD_x86
#define PHYSICS_SIMD_N DM_SIMD256_FLOAT_N
#elif defined(DM_SIMD_ARM)
#define PHYSICS_SIMD_N DM_SIMD_FLOAT_N
#endif

bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_ecs_id r_id, dm_ecs_system_timing timing, dm_ecs_id* sys_id, dm_context* context);
void physics_system_shutdown(void* s, void* c);
bool physics_system_run(void* s, void* c);
void physics_system_insert(const uint32_t entity_index, void* s, void* c);

// timings
double physics_system_get_broadphase_average(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);
double physics_system_get_narrowphase_average(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);
double physics_system_get_constraints_average(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);
double physics_system_get_update_average(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);
double physics_system_get_total_time(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);
uint32_t physics_system_get_num_iterations(dm_ecs_system_timing timing, dm_ecs_id sys_id, dm_context* context);

#endif //PHYSICS_SYSTEM_H
