#ifndef __GRAVITY_H__
#define __GRAVITY_H__

#include "../dm.h"

#if 1
#define SIMD_GRAVITY
#endif

dm_ecs_id gravity_system_init();

#ifdef SIMD_GRAVITY
void simd_gravity(dm_entity* entities, uint32_t entity_count);
#else
void naive_gravity(dm_entity* entities, uint32_t entity_count);
#endif

#endif //GRAVITY_H
