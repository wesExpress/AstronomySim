#include "gravity.h"
#include <float.h>
#include <string.h>

#define G 6.673e-11f

#ifdef DM_SIMD_256
#define N 8
#else
#define N 4
#endif

typedef struct gravity_cache_t
{
    float* pos_x, *pos_y, *pos_z;
    float* force_x, *force_y, *force_z;
    float* mass;
} gravity_cache;

static gravity_cache grav_cache = { 0 };

/**************
GRAVITY SYSTEM
****************/
bool gravity_system_func(dm_entity* entities, uint32_t entity_count)
{
    dm_imgui_text_fmt(10,250, 1,0,1,1, "Number of bodies: %u", entity_count);
    
    if(dm_physics_is_paused()) return true;
    
    grav_cache.pos_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    grav_cache.pos_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    grav_cache.pos_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    grav_cache.force_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_X);
    grav_cache.force_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Y);
    grav_cache.force_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Z);
    
    grav_cache.mass = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_MASS);
    
#ifdef SIMD_GRAVITY
    simd_gravity(entities, entity_count);
#else // naive approach
    naive_gravity(entities, entity_count);
#endif // SIMD_GRAVITY
    
    return true;
}

dm_ecs_id gravity_system_init()
{
    dm_ecs_id gravity_system_ids[] = { DM_COMPONENT_TRANSFORM, DM_COMPONENT_PHYSICS };
    dm_ecs_id gravity_system;
    DM_ECS_REGISTER_SYSTEM(DM_ECS_SYSTEM_TIMING_BEGIN, gravity_system_ids, gravity_system_func, gravity_system);
    
    return gravity_system;
}

/**************************
GRAVITATIONAL CALCULATIONS
****************************/
void calculate_gravitational_force(dm_entity entity_a, dm_entity entity_b)
{
    float mass_a = grav_cache.mass[entity_a];
    float mass_b = grav_cache.mass[entity_b];
    
    dm_vec3 pos_a = dm_vec3_set(grav_cache.pos_x[entity_a], grav_cache.pos_y[entity_a], grav_cache.pos_z[entity_a]);
    dm_vec3 pos_b = dm_vec3_set(grav_cache.pos_x[entity_b], grav_cache.pos_y[entity_b], grav_cache.pos_z[entity_b]);
    
    dm_vec3 separation = dm_vec3_sub_vec3(pos_b, pos_a);
    float distance2 = dm_vec3_len2(separation);
    float gravity = (G * mass_a * mass_b / distance2);
    
    dm_vec3 local_force = dm_vec3_scale(dm_vec3_norm(separation), gravity);
    
    grav_cache.force_x[entity_a] += local_force.x;
    grav_cache.force_y[entity_a] += local_force.y;
    grav_cache.force_z[entity_a] += local_force.z;
    
    grav_cache.force_x[entity_b] -= local_force.x;
    grav_cache.force_y[entity_b] -= local_force.y;
    grav_cache.force_z[entity_b] -= local_force.z;
}

/************
SIMD GRAVITY
**************/
#ifdef SIMD_GRAVITY
void simd_gravity(dm_entity* entities, uint32_t entity_count)
{
    dm_timer timer = { 0 };
    dm_timer_start(&timer);
    
    dm_mm_float grav_const = dm_mm_set1_ps(G);
    
    dm_mm_float mass_i;
    dm_mm_float pos_i_x, pos_i_y, pos_i_z;
    dm_mm_float local_x, local_y, local_z;
    dm_mm_float force_i_x, force_i_y, force_i_z;
    dm_mm_float direction_x, direction_y, direction_z;
    dm_mm_float gravity;
    
    dm_entity entity_i;
    
    for(uint32_t i=0; i<entity_count; i++)
    {
        entity_i = entities[i];
        
        pos_i_x = dm_mm_set1_ps(grav_cache.pos_x[entity_i]);
        pos_i_y = dm_mm_set1_ps(grav_cache.pos_y[entity_i]);
        pos_i_z = dm_mm_set1_ps(grav_cache.pos_z[entity_i]);
        
        force_i_x = dm_mm_set1_ps(grav_cache.force_x[entity_i]);
        force_i_y = dm_mm_set1_ps(grav_cache.force_y[entity_i]);
        force_i_z = dm_mm_set1_ps(grav_cache.force_z[entity_i]);
        mass_i  = dm_mm_set1_ps(grav_cache.mass[entity_i]);
        
        // iterate until we are N from the end
        uint32_t j = i + 1;
        for(; (entity_count-j)>=N; )
        {
            // rij = pos_j - pos_i
            direction_x = dm_mm_sub_ps(dm_mm_load_ps(pos_x+j), pos_i_x);
            direction_y = dm_mm_sub_ps(dm_mm_load_ps(pos_y+j), pos_i_y);
            direction_z = dm_mm_sub_ps(dm_mm_load_ps(pos_z+j), pos_i_z);
            
            // Calculate constant of gavity force
            // d^2 = sep_x * sep_x + sep_y * sep_y + sep_z * sep_z
            gravity = dm_mm_mul_ps(direction_x, direction_x);
            gravity = dm_mm_add_ps(gravity, dm_mm_mul_ps(direction_y, direction_y));
            gravity = dm_mm_add_ps(gravity, dm_mm_mul_ps(direction_z, direction_z));
            
            // d^(5/2)
            gravity = dm_mm_mul_ps(gravity, dm_mm_sqrt_ps(gravity));
            
            // G / d^(5/2)
            gravity = dm_mm_div_ps(grav_const, gravity);
            
            // G mi*mj / d^(5/2)
            gravity = dm_mm_mul_ps(gravity, dm_mm_load_ps(mass+j));
            gravity = dm_mm_mul_ps(gravity, mass_i);
            
            // local force
            local_x = dm_mm_mul_ps(gravity, direction_x);
            local_y = dm_mm_mul_ps(gravity, direction_y);
            local_z = dm_mm_mul_ps(gravity, direction_z);
            
            // j entities have negative of local force
            dm_mm_store_ps(force_x+j, dm_mm_sub_ps(dm_mm_load_ps(force_x+j), local_x));
            dm_mm_store_ps(force_y+j, dm_mm_sub_ps(dm_mm_load_ps(force_y+j), local_y));
            dm_mm_store_ps(force_z+j, dm_mm_sub_ps(dm_mm_load_ps(force_z+j), local_z));
            
            // entity i has all j forces acting upon it
            local_x = dm_mm_hadd_ps(local_x, local_x);
            local_x = dm_mm_hadd_ps(local_x, local_x);
            local_y = dm_mm_hadd_ps(local_y, local_y);
            local_y = dm_mm_hadd_ps(local_y, local_y);
            local_z = dm_mm_hadd_ps(local_z, local_z);
            local_z = dm_mm_hadd_ps(local_z, local_z);
            
            force_i_x = dm_mm_add_ps(force_i_x, local_x);
            force_i_y = dm_mm_add_ps(force_i_y, local_y);
            force_i_z = dm_mm_add_ps(force_i_z, local_z);
            
            // iterate
            j += N;
        }
        
        if(j==i+1)
        {
            //finish last 0-N entities with a standard brute force
            for(; j<entity_count; j++)
            {
                dm_entity entity_j = entities[j];
                calculate_gravitational_force(entity_i, entity_j);
            }
        }
        // otherwise copy over what we just calculated in the j loop
        else
        {
            dm_memcpy(grav_cache.force_x+i, &force_i_x, sizeof(float));
            dm_memcpy(grav_cache.force_y+i, &force_i_y, sizeof(float));
            dm_memcpy(grav_cache.force_z+i, &force_i_z, sizeof(float));
        }
    }
    
    double t = dm_timer_elapsed_ms(&timer);
    char buffer[512];
    
    if(N == 4) strcpy(buffer, "128");
    else       strcpy(buffer, "256");
    
    dm_imgui_text_fmt(10,275, 1,0,1,1, "N-body (SIMD%s) took: %0.2lf ms", buffer, t);
}

#else
/*******************
BRUTE FORCE GRAVITY
*********************/
void naive_gravity(dm_entity* entities, uint32_t entity_count)
{
    dm_timer timer = { 0 };
    dm_timer_start(&timer);
    
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity_i = entities[i];
        
        for(uint32_t j=i+1; j<entity_count; j++)
        {
            dm_entity entity_j = entities[j];
            calculate_gravitational_force(entity_i, entity_j);
        }
    }
    dm_imgui_text_fmt(10,275, 1,0,1,1, "N-body (naive brute force) took: %0.2lf ms", dm_timer_elapsed_ms(&timer));
}

#endif // SIMD_GRAVITY