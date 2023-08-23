#include "gravity_system.h"

#include "app/components.h"

#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#define G 6.673e-11f

typedef struct gravity_manager_t
{
    bool simd;
    
    dm_ecs_id transform, physics;
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
    
    return true;
}

void gravity_system_shutdown(void* s, void* c)
{
}

bool gravity_system_run(void* s, void* c)
{
    dm_context* context = c;
    dm_ecs_system_manager* system = s;
    gravity_manager* manager = system->system_data;
    
    dm_timer t = { 0 };
    dm_timer_start(&t, context);
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE, context)) manager->simd = !manager->simd;
    
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
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    
    dm_ecs_system_entity_container entity_a, entity_b;
    
    component_transform_block* t_block = dm_ecs_get_component_block(t_id, context);
    component_physics_block*   p_block = dm_ecs_get_component_block(p_id, context);
    
    component_transform_block* a_t_block = NULL;
    component_physics_block*   a_p_block = NULL;
    component_transform_block* b_t_block = NULL;
    component_physics_block*   b_p_block = NULL;
    
    uint32_t a_t_index, a_p_index;
    uint32_t b_t_index, b_p_index;
    
    float mass_a, mass_b;
    float pos_a_x, pos_a_y, pos_a_z;
    float pos_b_x, pos_b_y, pos_b_z;
    
    float sep_x, sep_y, sep_z;
    float dis2, grav;
    float local_f_x, local_f_y, local_f_z;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        entity_a = system->entity_containers[i];
        
        a_t_block = t_block + entity_a.block_indices[t_id];
        a_p_block = p_block + entity_a.block_indices[p_id];
        
        a_t_index = entity_a.component_indices[t_id];
        a_p_index = entity_a.component_indices[p_id];
        
        pos_a_x = a_t_block->pos_x[a_t_index];
        pos_a_y = a_t_block->pos_y[a_t_index];
        pos_a_z = a_t_block->pos_z[a_t_index];
        
        mass_a = a_p_block->mass[a_p_index];
        
        for(uint32_t j=i+1; j<system->entity_count; j++)
        {
            entity_b = system->entity_containers[j];
            
            b_t_block = t_block + entity_b.block_indices[t_id];
            b_p_block = p_block + entity_b.block_indices[p_id];
            
            b_t_index = entity_b.component_indices[t_id];
            b_p_index = entity_b.component_indices[p_id];
            
            pos_b_x = b_t_block->pos_x[b_t_index];
            pos_b_y = b_t_block->pos_y[b_t_index];
            pos_b_z = b_t_block->pos_z[b_t_index];
            
            mass_b = b_p_block->mass[b_p_index];
            
            // force calculation
            sep_x = pos_b_x - pos_a_x;
            sep_y = pos_b_y - pos_a_y;
            sep_z = pos_b_z - pos_a_z;
            
            dis2  = sep_x * sep_x;
            dis2 += sep_y * sep_y;
            dis2 += sep_z * sep_z;
            
            grav  = G * mass_a;
            grav *= mass_b;
            grav /= dis2;
            
            dis2 = dm_sqrtf(dis2);
            dis2 = 1.0f / dis2;
            
            sep_x *= dis2;
            sep_y *= dis2;
            sep_z *= dis2;
            
            local_f_x = sep_x * grav;
            local_f_y = sep_y * grav;
            local_f_z = sep_z * grav;
            
            // a gets force, b gets negative!
            a_p_block->force_x[a_p_index] += local_f_x;
            a_p_block->force_y[a_p_index] += local_f_y;
            a_p_block->force_z[a_p_index] += local_f_z;
            
            b_p_block->force_x[b_p_index] -= local_f_x;
            b_p_block->force_y[b_p_index] -= local_f_y;
            b_p_block->force_z[b_p_index] -= local_f_z;
        }
    }
}

/************
SIMD GRAVITY
**************/
void simd_gravity(dm_ecs_system_manager* system, dm_context* context)
{
    gravity_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    
    dm_ecs_system_entity_container entity_a, entity_b;
    
    component_transform_block* t_block = dm_ecs_get_component_block(t_id, context);
    component_physics_block*   p_block = dm_ecs_get_component_block(p_id, context);
    
    component_transform_block* a_t_block = NULL;
    component_physics_block*   a_p_block = NULL;
    
    uint32_t a_t_index, a_p_index;
    uint32_t b_t_index[DM_SIMD_N], b_p_index[DM_SIMD_N];
    uint32_t b_t_block[DM_SIMD_N], b_p_block[DM_SIMD_N];
    
    float pos_x_loader[DM_SIMD_N], pos_y_loader[DM_SIMD_N], pos_z_loader[DM_SIMD_N];
    float mass_loader[DM_SIMD_N];
    float force_x_loader[DM_SIMD_N], force_y_loader[DM_SIMD_N], force_z_loader[DM_SIMD_N];
    
    dm_mm_float mass_i, mass_j;
    dm_mm_float pos_i_x, pos_i_y, pos_i_z;
    dm_mm_float pos_j_x, pos_j_y, pos_j_z;
    
    dm_mm_float local_x, local_y, local_z;
    dm_mm_float force_j_x, force_j_y, force_j_z;
    dm_mm_float dir_x, dir_y, dir_z;
    dm_mm_float grav;
    
    dm_mm_float grav_const = dm_mm_set1_ps(G);
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        entity_a = system->entity_containers[i];
        
        a_t_block = t_block + entity_a.block_indices[t_id];
        a_p_block = p_block + entity_a.block_indices[p_id];
        
        a_t_index = entity_a.component_indices[t_id];
        a_p_index = entity_a.component_indices[p_id];
        
        // load in entity_i data
        pos_i_x = dm_mm_set1_ps(a_t_block->pos_x[a_t_index]);
        pos_i_y = dm_mm_set1_ps(a_t_block->pos_y[a_t_index]);
        pos_i_z = dm_mm_set1_ps(a_t_block->pos_z[a_t_index]);
        
        mass_i = dm_mm_set1_ps(a_p_block->mass[a_p_index]);
        
        for(uint32_t j=i+1; j<system->entity_count; j+=DM_SIMD_N)
        {
            for(uint32_t k=0; k<DM_SIMD_N; k++)
            {
                if(j+k>=system->entity_count) break;
                
                entity_b  = system->entity_containers[j+k];
                
                b_t_index[k] = entity_b.component_indices[t_id];
                b_t_block[k] = entity_b.block_indices[t_id];
                
                b_p_index[k] = entity_b.component_indices[p_id];
                b_p_block[k] = entity_b.block_indices[p_id];
                
                pos_x_loader[k] = (t_block + b_t_block[k])->pos_x[b_t_index[k]];
                pos_y_loader[k] = (t_block + b_t_block[k])->pos_y[b_t_index[k]];
                pos_z_loader[k] = (t_block + b_t_block[k])->pos_z[b_t_index[k]];
                
                force_x_loader[k] = (p_block + b_p_block[k])->force_x[b_p_index[k]];
                force_y_loader[k] = (p_block + b_p_block[k])->force_y[b_p_index[k]];
                force_z_loader[k] = (p_block + b_p_block[k])->force_z[b_p_index[k]];
                
                mass_loader[k] = (p_block + b_p_block[k])->mass[b_p_index[k]];
            }
            
            pos_j_x = dm_mm_load_ps(pos_x_loader);
            pos_j_y = dm_mm_load_ps(pos_y_loader);
            pos_j_z = dm_mm_load_ps(pos_z_loader);
            
            mass_j = dm_mm_load_ps(mass_loader);
            
            force_j_x = dm_mm_load_ps(force_x_loader);
            force_j_y = dm_mm_load_ps(force_y_loader);
            force_j_z = dm_mm_load_ps(force_z_loader);
            
            // rij = pos_j - pos_i
            dir_x = dm_mm_sub_ps(pos_j_x, pos_i_x);
            dir_y = dm_mm_sub_ps(pos_j_y, pos_i_y);
            dir_z = dm_mm_sub_ps(pos_j_z, pos_i_z);
            
            // r^2 = sep_x * sep_x + sep_y * sep_y + sep_z * sep_z
            grav = dm_mm_fmadd_ps(dir_x, dir_x, dm_mm_mul_ps(dir_y, dir_y));
            grav = dm_mm_fmadd_ps(dir_z, dir_z, grav);
            
            // r^(5/2)
            grav = dm_mm_mul_ps(grav, dm_mm_sqrt_ps(grav));
            
            // G / r^(5/2)
            grav = dm_mm_div_ps(grav_const, grav);
            
            // G mi * mj / r^(5/2)
            grav = dm_mm_mul_ps(grav, mass_i);
            grav = dm_mm_mul_ps(grav, mass_j);
            
            // local force
            local_x = dm_mm_mul_ps(grav, dir_x);
            local_y = dm_mm_mul_ps(grav, dir_y);
            local_z = dm_mm_mul_ps(grav, dir_z);
            
            // j entities have negative local force
            force_j_x = dm_mm_sub_ps(force_j_x, local_x);
            force_j_y = dm_mm_sub_ps(force_j_y, local_y);
            force_j_z = dm_mm_sub_ps(force_j_z, local_z);
            
            dm_mm_store_ps(force_x_loader, force_j_x);
            dm_mm_store_ps(force_y_loader, force_j_y);
            dm_mm_store_ps(force_z_loader, force_j_z);
            
            for(uint32_t k=0; k<DM_SIMD_N; k++)
            {
                (p_block + b_p_block[k])->force_x[b_p_index[k]] = force_x_loader[k];
                (p_block + b_p_block[k])->force_y[b_p_index[k]] = force_y_loader[k];
                (p_block + b_p_block[k])->force_z[b_p_index[k]] = force_z_loader[k];
            }
            
            // entity i has all j forces acting on it
            a_p_block->force_x[a_p_index] += dm_mm_sum_elements(local_x);
            a_p_block->force_y[a_p_index] += dm_mm_sum_elements(local_y);
            a_p_block->force_z[a_p_index] += dm_mm_sum_elements(local_z);
        }
    }
}