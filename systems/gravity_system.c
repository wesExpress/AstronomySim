#include "gravity_system.h"

#include "app/components.h"

#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#define G 6.6743e-11f

typedef struct gravity_data_cache_t
{
    float* pos_x, *pos_y, *pos_z;
    float* force_x, *force_y, *force_z;
    float* mass;
} gravity_data_cache;

typedef struct gravity_entity_cache_t
{
    uint32_t (*c_indices)[DM_ECS_MAX];
    uint32_t (*b_indices)[DM_ECS_MAX];
} gravity_entity_cache;

typedef struct gravity_manager_t
{
    dm_ecs_id transform, physics;
    bool      simd;
    
    gravity_data_cache   data_cache;
    gravity_entity_cache entity_cache;
    uint32_t             entity_count;
} gravity_manager;

void naive_gravity(dm_ecs_system_manager* system, dm_context* context);
void simd_gravity(dm_ecs_system_manager* system, dm_context* context);

/************
SYSTEM FUNCS
**************/
bool gravity_system_init(dm_ecs_id t_id, dm_ecs_id p_id, dm_context* context)
{
    dm_ecs_id comps[] = { t_id, p_id };
    
    dm_ecs_system_timing timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    dm_ecs_id id = dm_ecs_register_system(comps, DM_ARRAY_LEN(comps), timing, gravity_system_run, gravity_system_shutdown, context);
    if(id==DM_ECS_INVALID_ID) { DM_LOG_FATAL("Could not initialize graity system."); return false; }
    
    dm_ecs_system_manager* gravity_system = &context->ecs_manager.systems[timing][id];
    
    gravity_system->system_data = dm_alloc(sizeof(gravity_manager));
    gravity_manager* manager = gravity_system->system_data;
    
    manager->transform = t_id;
    manager->physics = p_id;
    
    manager->simd = true;
    
    return true;
}

void gravity_system_shutdown(void* s, void* c)
{
    dm_ecs_system_manager* system = s;
    gravity_manager* manager = system->system_data;
    
    dm_free(manager->data_cache.pos_x);
    dm_free(manager->data_cache.pos_y);
    dm_free(manager->data_cache.pos_z);
    
    dm_free(manager->data_cache.force_x);
    dm_free(manager->data_cache.force_y);
    dm_free(manager->data_cache.force_z);
    
    dm_free(manager->data_cache.mass);
    
    dm_free(manager->entity_cache.c_indices);
    dm_free(manager->entity_cache.b_indices);
}

void gravity_populate_arrays(gravity_manager* manager, dm_ecs_system_manager* system, dm_context* context)
{
    component_transform_block* t_block = dm_ecs_get_component_block(manager->transform, context);
    component_physics_block*   p_block = dm_ecs_get_component_block(manager->physics, context);
    
    uint32_t t_c_index, p_c_index;
    uint32_t t_b_index, p_b_index;
    
    dm_ecs_system_entity_container entity;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        entity = system->entity_containers[i];
        
        t_c_index = entity.component_indices[manager->transform];
        p_c_index = entity.component_indices[manager->physics];
        
        t_b_index = entity.block_indices[manager->transform];
        p_b_index = entity.block_indices[manager->physics];
        
        manager->data_cache.pos_x[i] = (t_block + t_b_index)->pos_x[t_c_index];
        manager->data_cache.pos_y[i] = (t_block + t_b_index)->pos_y[t_c_index];
        manager->data_cache.pos_z[i] = (t_block + t_b_index)->pos_z[t_c_index];
        
        manager->data_cache.force_x[i] = (p_block + p_b_index)->force_x[p_c_index];
        manager->data_cache.force_y[i] = (p_block + p_b_index)->force_y[p_c_index];
        manager->data_cache.force_z[i] = (p_block + p_b_index)->force_z[p_c_index];
        
        manager->data_cache.mass[i] = (p_block + p_b_index)->mass[p_c_index];
        
        dm_memcpy(manager->entity_cache.c_indices[i], entity.component_indices, sizeof(entity.component_indices));
        dm_memcpy(manager->entity_cache.b_indices[i], entity.block_indices, sizeof(entity.block_indices));
    }
}

bool gravity_system_run(void* s, void* c)
{
    dm_context* context = c;
    dm_ecs_system_manager* system = s;
    gravity_manager* manager = system->system_data;
    
    if(manager->entity_count==0) 
    {
        manager->data_cache.pos_x = dm_alloc(sizeof(float) * system->entity_count);
        manager->data_cache.pos_y = dm_alloc(sizeof(float) * system->entity_count);
        manager->data_cache.pos_z = dm_alloc(sizeof(float) * system->entity_count);
        
        manager->data_cache.force_x = dm_alloc(sizeof(float) * system->entity_count);
        manager->data_cache.force_y = dm_alloc(sizeof(float) * system->entity_count);
        manager->data_cache.force_z = dm_alloc(sizeof(float) * system->entity_count);
        
        manager->data_cache.mass = dm_alloc(sizeof(float) * system->entity_count);
        
        manager->entity_cache.c_indices = dm_alloc(sizeof(uint32_t) * DM_ECS_MAX * system->entity_count);
        manager->entity_cache.b_indices = dm_alloc(sizeof(uint32_t) * DM_ECS_MAX * system->entity_count);
        
        manager->entity_count = system->entity_count;
    }
    else if(manager->entity_count != system->entity_count)
    {
        manager->data_cache.pos_x = dm_realloc(manager->data_cache.pos_x, sizeof(float) * system->entity_count);
        manager->data_cache.pos_y = dm_realloc(manager->data_cache.pos_y, sizeof(float) * system->entity_count);
        manager->data_cache.pos_z = dm_realloc(manager->data_cache.pos_z, sizeof(float) * system->entity_count);
        
        manager->data_cache.force_x = dm_realloc(manager->data_cache.force_x, sizeof(float) * system->entity_count);
        manager->data_cache.force_y = dm_realloc(manager->data_cache.force_y, sizeof(float) * system->entity_count);
        manager->data_cache.force_z = dm_realloc(manager->data_cache.force_z, sizeof(float) * system->entity_count);
        
        manager->data_cache.mass = dm_realloc(manager->data_cache.mass, sizeof(float) * system->entity_count);
        
        manager->entity_cache.c_indices = dm_realloc(manager->entity_cache.c_indices, sizeof(uint32_t) * DM_ECS_MAX * system->entity_count);
        manager->entity_cache.b_indices = dm_realloc(manager->entity_cache.b_indices, sizeof(uint32_t) * DM_ECS_MAX * system->entity_count);
        
        manager->entity_count = system->entity_count;
    }
    
    dm_timer t = { 0 };
    dm_timer_start(&t, context);
    gravity_populate_arrays(manager, system, context);
    if(dm_input_key_just_pressed(DM_KEY_P, context)) 
    {
        DM_LOG_INFO("Switching gravity method");
        manager->simd = !manager->simd;
    }
    
    if(manager->simd) simd_gravity(system, context);
    else naive_gravity(system, context);
    
    imgui_draw_text_fmt(20,120, 0,1,0,1, context, "Gravity took: %0.3lf ms (%u entities)", dm_timer_elapsed_ms(&t, context), system->entity_count);
    
    return true;
}

/************
NAIVE GRAITY
**************/
void naive_gravity(dm_ecs_system_manager* system, dm_context* context)
{
    gravity_manager* manager = system->system_data;
    
    const dm_ecs_id p_id = manager->physics;
    
    float mass_a, mass_b;
    float pos_a_x, pos_a_y, pos_a_z;
    float pos_b_x, pos_b_y, pos_b_z;
    
    float dir_x, dir_y, dir_z;
    float dis2, grav;
    float local_f_x, local_f_y, local_f_z;
    
    uint32_t i=0,j=0;
    for(; i<system->entity_count; i++)
    {
        pos_a_x = manager->data_cache.pos_x[i];
        pos_a_y = manager->data_cache.pos_y[i];
        pos_a_z = manager->data_cache.pos_z[i];
        
        mass_a = manager->data_cache.mass[i];
        
        j = i + 1;
        for(; j<system->entity_count; j++)
        {
            pos_b_x = manager->data_cache.pos_x[j];
            pos_b_y = manager->data_cache.pos_y[j];
            pos_b_z = manager->data_cache.pos_z[j];
            
            mass_b = manager->data_cache.mass[j];
            
            // force calculation
            dir_x = pos_b_x - pos_a_x;
            dir_y = pos_b_y - pos_a_y;
            dir_z = pos_b_z - pos_a_z;
            
            dis2  = dir_x * dir_x;
            dis2 += dir_y * dir_y;
            dis2 += dir_z * dir_z;
            
            grav  = G * mass_a;
            grav *= mass_b;
            grav /= dis2;
            
            dis2 = dm_sqrtf(dis2);
            dis2 = 1.0f / dis2;
            
            dir_x *= dis2;
            dir_y *= dis2;
            dir_z *= dis2;
            
            local_f_x = dir_x * grav;
            local_f_y = dir_y * grav;
            local_f_z = dir_z * grav;
            
            // a gets force, b gets negative!
            manager->data_cache.force_x[i] += local_f_x;
            manager->data_cache.force_y[i] += local_f_y;
            manager->data_cache.force_z[i] += local_f_z;
            
            manager->data_cache.force_x[j] -= local_f_x;
            manager->data_cache.force_y[j] -= local_f_y;
            manager->data_cache.force_z[j] -= local_f_z;
        }
    }
    
    // now update data
    component_physics_block* p_block = dm_ecs_get_component_block(p_id, context);
    uint32_t p_index, b_index;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        p_index = manager->entity_cache.c_indices[i][p_id];
        b_index = manager->entity_cache.b_indices[i][p_id];
        
        (p_block + b_index)->force_x[p_index] = manager->data_cache.force_x[i];
        (p_block + b_index)->force_y[p_index] = manager->data_cache.force_y[i];
        (p_block + b_index)->force_z[p_index] = manager->data_cache.force_z[i];
    }
}

/************
SIMD GRAVITY
**************/
void simd_gravity(dm_ecs_system_manager* system, dm_context* context)
{
    gravity_manager* manager = system->system_data;
    
    const dm_ecs_id p_id = manager->physics;
    
    dm_mm_float mass_i, mass_j;
    dm_mm_float pos_i_x, pos_i_y, pos_i_z;
    dm_mm_float pos_j_x, pos_j_y, pos_j_z;
    
    dm_mm_float local_x, local_y, local_z;
    dm_mm_float force_j_x, force_j_y, force_j_z;
    dm_mm_float dir_x, dir_y, dir_z;
    dm_mm_float grav, dis2;
    
    const dm_mm_float grav_const = dm_mm_set1_ps(G);
    const dm_mm_float ones = dm_mm_set1_ps(1.0f);
    
    uint32_t i=0, j=0;
    uint32_t leftovers = 0;
    float loader_x[DM_SIMD_N], loader_y[DM_SIMD_N], loader_z[DM_SIMD_N];
    
    for(; i<system->entity_count; i++)
    {
        // load in entity_i data
        pos_i_x = dm_mm_set1_ps(manager->data_cache.pos_x[i]);
        pos_i_y = dm_mm_set1_ps(manager->data_cache.pos_y[i]);
        pos_i_z = dm_mm_set1_ps(manager->data_cache.pos_z[i]);
        
        mass_i = dm_mm_set1_ps(manager->data_cache.mass[i]);
        
        j = i+1;
        for(; (system->entity_count-j)>DM_SIMD_N; j+=DM_SIMD_N)
        {
            pos_j_x = dm_mm_load_ps(&manager->data_cache.pos_x[j]);
            pos_j_y = dm_mm_load_ps(&manager->data_cache.pos_y[j]);
            pos_j_z = dm_mm_load_ps(&manager->data_cache.pos_z[j]);
            
            mass_j = dm_mm_load_ps(manager->data_cache.mass + j);
            
            force_j_x = dm_mm_load_ps(manager->data_cache.force_x + j);
            force_j_y = dm_mm_load_ps(manager->data_cache.force_y + j);
            force_j_z = dm_mm_load_ps(manager->data_cache.force_z + j);
            
            // rij = pos_j - pos_i
            dir_x = dm_mm_sub_ps(pos_j_x, pos_i_x);
            dir_y = dm_mm_sub_ps(pos_j_y, pos_i_y);
            dir_z = dm_mm_sub_ps(pos_j_z, pos_i_z);
            
            // r^2 = sep_x * sep_x + sep_y * sep_y + sep_z * sep_z
            dis2 = dm_mm_mul_ps(dir_x, dir_x);
            dis2 = dm_mm_fmadd_ps(dir_y, dir_y, dis2);
            dis2 = dm_mm_fmadd_ps(dir_z, dir_z, dis2);
            
            // G mi * mj 
            grav = dm_mm_mul_ps(grav_const, mass_i);
            grav = dm_mm_mul_ps(grav, mass_j);
            grav = dm_mm_div_ps(grav, dis2);
            
            dis2 = dm_mm_sqrt_ps(dis2);
            dis2 = dm_mm_div_ps(ones, dis2);
            
            dir_x = dm_mm_mul_ps(dir_x, dis2);
            dir_y = dm_mm_mul_ps(dir_y, dis2);
            dir_z = dm_mm_mul_ps(dir_z, dis2);
            
            // local force
            local_x = dm_mm_mul_ps(grav, dir_x);
            local_y = dm_mm_mul_ps(grav, dir_y);
            local_z = dm_mm_mul_ps(grav, dir_z);
            
            // j entities have negative local force
            force_j_x = dm_mm_sub_ps(force_j_x, local_x);
            force_j_y = dm_mm_sub_ps(force_j_y, local_y);
            force_j_z = dm_mm_sub_ps(force_j_z, local_z);
            
            dm_mm_store_ps(manager->data_cache.force_x+j, force_j_x);
            dm_mm_store_ps(manager->data_cache.force_y+j, force_j_y);
            dm_mm_store_ps(manager->data_cache.force_z+j, force_j_z);
            
            // entity i has all j forces acting on it
            manager->data_cache.force_x[i] += dm_mm_sum_elements(local_x);
            manager->data_cache.force_y[i] += dm_mm_sum_elements(local_y);
            manager->data_cache.force_z[i] += dm_mm_sum_elements(local_z);
        }
        
        // we probably have a partial pass to do
        leftovers = system->entity_count - j;
        if(leftovers>=DM_SIMD_N) continue;
        if(leftovers==0) continue;
        
        dm_memzero(&pos_j_x, sizeof(pos_j_x));
        dm_memzero(&pos_j_y, sizeof(pos_j_y));
        dm_memzero(&pos_j_z, sizeof(pos_j_z));
        
        dm_memzero(&force_j_x, sizeof(force_j_x));
        dm_memzero(&force_j_y, sizeof(force_j_y));
        dm_memzero(&force_j_z, sizeof(force_j_z));
        
        dm_memzero(&mass_j, sizeof(mass_j));
        
        dm_memcpy(&pos_j_x, manager->data_cache.pos_x+j, sizeof(float) * leftovers);
        dm_memcpy(&pos_j_y, manager->data_cache.pos_y+j, sizeof(float) * leftovers);
        dm_memcpy(&pos_j_z, manager->data_cache.pos_z+j, sizeof(float) * leftovers);
        
        dm_memcpy(&force_j_x, manager->data_cache.force_x+j, sizeof(float) * leftovers);
        dm_memcpy(&force_j_y, manager->data_cache.force_y+j, sizeof(float) * leftovers);
        dm_memcpy(&force_j_z, manager->data_cache.force_z+j, sizeof(float) * leftovers);
        
        dm_memcpy(&mass_j, manager->data_cache.mass+j, sizeof(float) * leftovers);
        
        // rij = pos_j - pos_i
        dir_x = dm_mm_sub_ps(pos_j_x, pos_i_x);
        dir_y = dm_mm_sub_ps(pos_j_y, pos_i_y);
        dir_z = dm_mm_sub_ps(pos_j_z, pos_i_z);
        
        // r^2 = sep_x * sep_x + sep_y * sep_y + sep_z * sep_z
        dis2 = dm_mm_mul_ps(dir_x, dir_x);
        dis2 = dm_mm_fmadd_ps(dir_y, dir_y, dis2);
        dis2 = dm_mm_fmadd_ps(dir_z, dir_z, dis2);
        
        // G mi * mj / (r^2)^(3/2)
        grav = dm_mm_mul_ps(grav_const, mass_i);
        grav = dm_mm_mul_ps(grav, mass_j);
        grav = dm_mm_div_ps(grav, dis2);
        
        dis2 = dm_mm_sqrt_ps(dis2);
        dis2 = dm_mm_div_ps(ones, dis2);
        
        dir_x = dm_mm_mul_ps(dir_x, dis2);
        dir_y = dm_mm_mul_ps(dir_y, dis2);
        dir_z = dm_mm_mul_ps(dir_z, dis2);
        
        // local force
        local_x = dm_mm_mul_ps(grav, dir_x);
        local_y = dm_mm_mul_ps(grav, dir_y);
        local_z = dm_mm_mul_ps(grav, dir_z);
        
        // j entities have negative local force
        force_j_x = dm_mm_sub_ps(force_j_x, local_x);
        force_j_y = dm_mm_sub_ps(force_j_y, local_y);
        force_j_z = dm_mm_sub_ps(force_j_z, local_z);
        
        dm_mm_store_ps(loader_x, force_j_x);
        dm_mm_store_ps(loader_y, force_j_y);
        dm_mm_store_ps(loader_z, force_j_z);
        
        dm_memcpy(manager->data_cache.force_x+j, loader_x, sizeof(float) * leftovers);
        dm_memcpy(manager->data_cache.force_y+j, loader_y, sizeof(float) * leftovers);
        dm_memcpy(manager->data_cache.force_z+j, loader_z, sizeof(float) * leftovers);
        
        // entity i has all j forces acting on it
        manager->data_cache.force_x[i] += dm_mm_sum_elements(local_x);
        manager->data_cache.force_y[i] += dm_mm_sum_elements(local_y);
        manager->data_cache.force_z[i] += dm_mm_sum_elements(local_z);
    }
    
    // now update all forces
    component_physics_block*   p_block = dm_ecs_get_component_block(p_id, context);
    uint32_t b_index, c_index;
    
    for(uint32_t e=0; e<system->entity_count; e++)
    {
        c_index = manager->entity_cache.c_indices[e][p_id];
        b_index = manager->entity_cache.b_indices[e][p_id];
        
        (p_block + b_index)->force_x[c_index] = manager->data_cache.force_x[e];
        (p_block + b_index)->force_y[c_index] = manager->data_cache.force_y[e];
        (p_block + b_index)->force_z[c_index] = manager->data_cache.force_z[e];
    }
}