#include "gravity_system.h"

#include "app/components.h"

#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#define G 6.6743e-11f

typedef struct gravity_system_manager_t
{
    dm_ecs_id transform, physics;
    bool      simd;
} gravity_system_manager;

void naive_gravity(dm_ecs_system_manager* system, dm_context* context);
void simd_gravity(dm_ecs_system_manager* system, dm_context* context);

/************
SYSTEM FUNCS
**************/
bool gravity_system_init(dm_ecs_id t_id, dm_ecs_id p_id, dm_context* context)
{
    dm_ecs_id comps[] = { t_id, p_id };
    
    dm_ecs_system_timing timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    // 3 position floats, 3 force floats, 1 mass float
    size_t cache_elem_size = sizeof(float) * 3 + sizeof(float);
    
    dm_ecs_id id;
    id = dm_ecs_register_system(comps, DM_ARRAY_LEN(comps), timing, gravity_system_run, gravity_system_shutdown, context);
    if(id==DM_ECS_INVALID_ID) { DM_LOG_FATAL("Could not initialize graity system."); return false; }
    
    dm_ecs_system_manager* gravity_system = &context->ecs_manager.systems[timing][id];
    
    gravity_system->system_data = dm_alloc(sizeof(gravity_system_manager));
    gravity_system_manager* manager = gravity_system->system_data;
    
    manager->transform = t_id;
    manager->physics = p_id;
    
    manager->simd = true;
    
    return true;
}

void gravity_system_shutdown(void* s, void* c)
{
}

bool gravity_system_run(void* s, void* c)
{
    dm_context* context = c;
    dm_ecs_system_manager* system = s;
    gravity_system_manager* manager = system->system_data;
    
    dm_timer t = { 0 };
    dm_timer_start(&t, context);
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
    gravity_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    
    component_transform_block*  transform = dm_ecs_get_component_block(t_id, context);
    component_physics_block*    physics   = dm_ecs_get_component_block(p_id, context);
    
    uint32_t t_a, p_a;
    uint32_t t_b, p_b;
    
    float mass_a, mass_b;
    float pos_a_x, pos_a_y, pos_a_z;
    float pos_b_x, pos_b_y, pos_b_z;
    
    float dir_x, dir_y, dir_z;
    float dis2, grav;
    float local_f_x, local_f_y, local_f_z;
    
    uint32_t i=0,j=0;
    for(; i<system->entity_count; i++)
    {
        t_a = system->entity_indices[i][t_id];
        p_a = system->entity_indices[i][p_id];
        
        pos_a_x = transform->pos_x[t_a];
        pos_a_y = transform->pos_y[t_a];
        pos_a_z = transform->pos_z[t_a];
        
        mass_a = physics->mass[p_a];
        
        j = i + 1;
        for(; j<system->entity_count; j++)
        {
            t_b = system->entity_indices[j][t_id];
            p_b = system->entity_indices[j][p_id];
            
            pos_b_x = transform->pos_x[t_b];
            pos_b_y = transform->pos_y[t_b];
            pos_b_z = transform->pos_z[t_b];
            
            mass_b = physics->mass[p_b];
            
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
            physics->force_x[p_a] += local_f_x;
            physics->force_x[p_b] -= local_f_x;
            
            physics->force_y[p_a] += local_f_y;
            physics->force_y[p_b] -= local_f_y;
            
            physics->force_z[p_a] += local_f_z;
            physics->force_z[p_b] -= local_f_z;
        }
    }
}

/************
SIMD GRAVITY
**************/
void simd_gravity(dm_ecs_system_manager* system, dm_context* context)
{
    gravity_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    
    component_transform_block*  transform = dm_ecs_get_component_block(t_id, context);
    component_physics_block*    physics   = dm_ecs_get_component_block(p_id, context);
    
    uint32_t t_a, p_a;
    uint32_t t_b[DM_SIMD_N], p_b[DM_SIMD_N];
    
    dm_mm_float mass_i, mass_j;
    dm_mm_float pos_i_x, pos_i_y, pos_i_z;
    dm_mm_float pos_j_x, pos_j_y, pos_j_z;
    
    dm_mm_float local_x, local_y, local_z;
    dm_mm_float force_j_x, force_j_y, force_j_z;
    dm_mm_float dir_x, dir_y, dir_z;
    dm_mm_float grav, dis2;
    
    const dm_mm_float grav_const = dm_mm_set1_ps(G);
    const dm_mm_float ones   = dm_mm_set1_ps(1.0f);
    const dm_mm_float zeroes = dm_mm_set1_ps(0);
    
    uint32_t i=0, j=0, k=0;
    uint32_t leftovers = 0;
    float loader_x[DM_SIMD_N], loader_y[DM_SIMD_N], loader_z[DM_SIMD_N];
    float pos_x_loader[DM_SIMD_N], pos_y_loader[DM_SIMD_N], pos_z_loader[DM_SIMD_N];
    float force_x_loader[DM_SIMD_N], force_y_loader[DM_SIMD_N], force_z_loader[DM_SIMD_N];
    float mass_loader[DM_SIMD_N];
    
    for(; i<system->entity_count; i++)
    {
        t_a = system->entity_indices[i][t_id];
        p_a = system->entity_indices[i][p_id];
        
        // load in entity_i data
        pos_i_x = dm_mm_set1_ps(transform->pos_x[t_a]);
        pos_i_y = dm_mm_set1_ps(transform->pos_y[t_a]);
        pos_i_z = dm_mm_set1_ps(transform->pos_z[t_a]);
        
        mass_i = dm_mm_set1_ps(physics->mass[p_a]);
        
        j = i+1;
        for(; (system->entity_count-j)>=DM_SIMD_N; j+=DM_SIMD_N)
        {
            k = 0;
            for(;k<DM_SIMD_N; k++)
            {
                t_b[k] = system->entity_indices[j+k][t_id];
                p_b[k] = system->entity_indices[j+k][p_id];
                
                pos_x_loader[k] = transform->pos_x[t_b[k]];
                pos_y_loader[k] = transform->pos_y[t_b[k]];
                pos_z_loader[k] = transform->pos_z[t_b[k]];
                
                mass_loader[k] = physics->mass[t_b[k]];
            }
            
            pos_j_x = dm_mm_load_ps(pos_x_loader);
            pos_j_y = dm_mm_load_ps(pos_y_loader);
            pos_j_z = dm_mm_load_ps(pos_z_loader);
            
            mass_j = dm_mm_load_ps(mass_loader);
            
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
            
            // entity i has all j forces acting on it
            // j entities have negative local force
            force_j_x = dm_mm_sub_ps(zeroes, local_x);
            force_j_y = dm_mm_sub_ps(zeroes, local_y);
            force_j_z = dm_mm_sub_ps(zeroes, local_z);
            
            physics->force_x[t_a] += dm_mm_sum_elements(local_x);
            physics->force_y[t_a] += dm_mm_sum_elements(local_y);
            physics->force_z[t_a] += dm_mm_sum_elements(local_z);
            
            dm_mm_store_ps(force_x_loader, force_j_x);
            dm_mm_store_ps(force_y_loader, force_j_y);
            dm_mm_store_ps(force_z_loader, force_j_z);
            
            k=0;
            for(;k<DM_SIMD_N;k++)
            {
                physics->force_x[p_b[k]] += force_x_loader[k];
                physics->force_y[p_b[k]] += force_y_loader[k];
                physics->force_z[p_b[k]] += force_z_loader[k];
            }
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
        
        k=0;
        for(; k<leftovers; k++)
        {
            t_b[k] = system->entity_indices[j+k][t_id];
            p_b[k] = system->entity_indices[j+k][p_id];
            
            pos_x_loader[k] = transform->pos_x[t_b[k]];
            pos_y_loader[k] = transform->pos_y[t_b[k]];
            pos_z_loader[k] = transform->pos_z[t_b[k]];
            
            mass_loader[k] = physics->mass[p_b[k]];
        }
        
        pos_j_x = dm_mm_load_ps(pos_x_loader);
        pos_j_y = dm_mm_load_ps(pos_y_loader);
        pos_j_z = dm_mm_load_ps(pos_z_loader);
        
        mass_j = dm_mm_load_ps(mass_loader);
        
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
        
        // entity i has all j forces acting on it
        // j entities have negative local force
        force_j_x = dm_mm_sub_ps(zeroes, local_x);
        force_j_y = dm_mm_sub_ps(zeroes, local_y);
        force_j_z = dm_mm_sub_ps(zeroes, local_z);
        
        dm_mm_store_ps(loader_x, force_j_x);
        dm_mm_store_ps(loader_y, force_j_y);
        dm_mm_store_ps(loader_z, force_j_z);
        
        physics->force_x[p_a] += dm_mm_sum_elements(local_x);
        physics->force_y[p_a] += dm_mm_sum_elements(local_y);
        physics->force_z[p_a] += dm_mm_sum_elements(local_z);
        
        dm_mm_store_ps(force_x_loader, force_j_x);
        dm_mm_store_ps(force_y_loader, force_j_y);
        dm_mm_store_ps(force_z_loader, force_j_z);
        
        k=0;
        for(; k<leftovers;k++)
        {
            physics->force_x[p_b[k]] += force_x_loader[k];
            physics->force_y[p_b[k]] += force_y_loader[k];
            physics->force_z[p_b[k]] += force_z_loader[k];
        }
    }
}