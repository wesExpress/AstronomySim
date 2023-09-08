#include "physics_system.h"

#include "app/components.h"

#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#include <limits.h>
#include <float.h>
#include <assert.h>

#define PHYSICS_SYSTEM_CONSTRAINT_ITER 10

typedef struct physics_system_aabb_sort_t
{
    float min_x[DM_ECS_MAX_ENTITIES];
    float min_y[DM_ECS_MAX_ENTITIES];
    float min_z[DM_ECS_MAX_ENTITIES];
    
    float max_x[DM_ECS_MAX_ENTITIES];
    float max_y[DM_ECS_MAX_ENTITIES];
    float max_z[DM_ECS_MAX_ENTITIES];
} physics_system_aabb_sort;

typedef struct physics_system_cache_t
{
    component_transform  transform;
    component_physics    physics;
    component_collision  collision;
    component_rigid_body rigid_body;
} physics_system_cache;

typedef struct physics_system_collision_pair_t
{
    uint32_t entity_a, entity_b;
} physics_system_collision_pair;

typedef struct physics_system_broadphase_multi_th_data_t
{
    uint32_t  max_count, thread_id, start_id;
    size_t    thread_count;
    
    uint32_t*                 indices;
    physics_system_aabb_sort* aabbs_sorted;
    
    float* min_array;
    float* max_array;
    collision_flag* flag;
    
    uint32_t                      possible_collision_count;
    physics_system_collision_pair possible_collisions[DM_ECS_MAX_ENTITIES];
} physics_system_broadphase_multi_th_data;

typedef struct physics_system_broadphase_data_t
{
    float center_sum[3];
    float center_sq_sum[3];
    
    uint32_t sort_axis;
    uint32_t num_checks, num_possible_collisions;
    
    physics_system_broadphase_multi_th_data sweep_mt_data[DM_MAX_THREAD_COUNT];
    
    uint32_t sweep_indices[DM_ECS_MAX_ENTITIES];
    uint32_t sphere_indices[DM_ECS_MAX_ENTITIES], box_indices[DM_ECS_MAX_ENTITIES];
    
    physics_system_aabb_sort aabbs_sorted;
    
    physics_system_collision_pair possible_collisions[DM_ECS_MAX_ENTITIES];
} physics_system_broadphase_data;

typedef struct physics_system_narrowphase_data_t
{
    uint32_t            manifold_count;
    dm_contact_manifold manifolds[DM_ECS_MAX_ENTITIES];
} physics_system_narrowphase_data;

typedef enum physics_system_flag_t
{
    DM_PHYSICS_FLAG_PAUSED = 1 << 0,
    DM_PHYSICS_FLAG_NO_COLLISIONS = 1 << 1,
} physics_system_flag;

typedef struct physics_system_manager_t
{
    double              accum_time, simulation_time;
    physics_system_flag flags;
    
    dm_ecs_id transform, collision, physics, rigid_body;
    uint32_t entity_count;
    
    physics_system_broadphase_data  broadphase_data;
    physics_system_narrowphase_data narrowphase_data;
    
    physics_system_cache cache;
    
    dm_threadpool threadpool;
} physics_system_manager;

bool physics_system_broadphase(dm_ecs_system* system, dm_context* context);
bool physics_system_narrowphase(dm_ecs_system* system);
void physics_system_solve_constraints(physics_system_manager* manager);
void physics_system_update_entities(dm_ecs_system* system);
void physics_system_update_entities_simd(dm_ecs_system* system);

/************
SYSTEM FUNCS
**************/
bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_ecs_id r_id, dm_context* context)
{
    dm_ecs_id comps[] = { t_id, c_id, p_id, r_id };
    
    dm_ecs_system_timing timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    
    dm_ecs_id id;
    id = dm_ecs_register_system(comps, DM_ARRAY_LEN(comps), timing, physics_system_run, physics_system_shutdown, physics_system_insert, context);
    
    dm_ecs_system* system = &context->ecs_manager.systems[timing][id];
    system->system_data   = dm_alloc(sizeof(physics_system_manager));
    
    physics_system_manager* manager = system->system_data;
    
    manager->transform  = t_id;
    manager->collision  = c_id;
    manager->physics    = p_id;
    manager->rigid_body = r_id;
    
    if(!dm_threadpool_create("physics_system", 4, &manager->threadpool)) return false;
    
    return true;
}

void physics_system_shutdown(void* s, void* c)
{
    dm_ecs_system* system = s;
    physics_system_manager* manager = system->system_data;
    
    dm_threadpool_destroy(&manager->threadpool);
}

void physics_system_insert(const uint32_t entity_index, void* s, void* c)
{
    dm_context*             context = c;
    dm_ecs_system*          system  = s;   
    physics_system_manager* manager = system->system_data;
    
    const uint32_t i = system->entity_count;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    const dm_ecs_id r_id = manager->rigid_body;
    const dm_ecs_id c_id = manager->collision;
    
    const component_transform*  transform  = dm_ecs_get_component_block(t_id, context);
    const component_physics*    physics    = dm_ecs_get_component_block(p_id, context);
    const component_collision*  collision  = dm_ecs_get_component_block(c_id, context);
    const component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    
    const uint32_t t_index = system->entity_indices[i][t_id];
    const uint32_t p_index = system->entity_indices[i][p_id]; 
    const uint32_t c_index = system->entity_indices[i][c_id]; 
    const uint32_t r_index = system->entity_indices[i][r_id];
    
    // transform
    manager->cache.transform.pos_x[i] = transform->pos_x[t_index];
    manager->cache.transform.pos_y[i] = transform->pos_y[t_index];
    manager->cache.transform.pos_z[i] = transform->pos_z[t_index];
    
    manager->cache.transform.scale_x[i] = transform->scale_x[t_index];
    manager->cache.transform.scale_y[i] = transform->scale_y[t_index];
    manager->cache.transform.scale_z[i] = transform->scale_z[t_index];
    
    manager->cache.transform.rot_i[i] = transform->rot_i[t_index];
    manager->cache.transform.rot_j[i] = transform->rot_j[t_index];
    manager->cache.transform.rot_k[i] = transform->rot_k[t_index];
    manager->cache.transform.rot_r[i] = transform->rot_r[t_index];
    
    // physics
    manager->cache.physics.vel_x[i] = physics->vel_x[p_index];
    manager->cache.physics.vel_y[i] = physics->vel_y[p_index];
    manager->cache.physics.vel_z[i] = physics->vel_z[p_index];
    
    manager->cache.physics.w_x[i] = physics->w_x[p_index];
    manager->cache.physics.w_y[i] = physics->w_y[p_index];
    manager->cache.physics.w_z[i] = physics->w_z[p_index];
    
    manager->cache.physics.l_x[i] = physics->l_x[p_index];
    manager->cache.physics.l_y[i] = physics->l_y[p_index];
    manager->cache.physics.l_z[i] = physics->l_z[p_index];
    
    manager->cache.physics.force_x[i] = physics->force_x[p_index];
    manager->cache.physics.force_y[i] = physics->force_y[p_index];
    manager->cache.physics.force_z[i] = physics->force_z[p_index];
    
    manager->cache.physics.torque_x[i] = physics->torque_x[p_index];
    manager->cache.physics.torque_y[i] = physics->torque_y[p_index];
    manager->cache.physics.torque_z[i] = physics->torque_z[p_index];
    
    manager->cache.physics.mass[i] = physics->mass[p_index];
    manager->cache.physics.inv_mass[i] = physics->inv_mass[p_index];
    
    manager->cache.physics.damping_v[i] = physics->damping_v[p_index];
    manager->cache.physics.damping_w[i] = physics->damping_w[p_index];
    
    manager->cache.physics.movement_type[i] = physics->movement_type[p_index];
    
    // collision
    manager->cache.collision.aabb_local_min_x[i] = collision->aabb_local_min_x[c_index];
    manager->cache.collision.aabb_local_min_y[i] = collision->aabb_local_min_y[c_index];
    manager->cache.collision.aabb_local_min_z[i] = collision->aabb_local_min_z[c_index];
    
    manager->cache.collision.aabb_local_max_x[i] = collision->aabb_local_max_x[c_index];
    manager->cache.collision.aabb_local_max_y[i] = collision->aabb_local_max_y[c_index];
    manager->cache.collision.aabb_local_max_z[i] = collision->aabb_local_max_z[c_index];
    
    manager->cache.collision.aabb_global_min_x[i] = collision->aabb_global_min_x[c_index];
    manager->cache.collision.aabb_global_min_y[i] = collision->aabb_global_min_y[c_index];
    manager->cache.collision.aabb_global_min_z[i] = collision->aabb_global_min_z[c_index];
    
    manager->cache.collision.aabb_global_max_x[i] = collision->aabb_global_max_x[c_index];
    manager->cache.collision.aabb_global_max_y[i] = collision->aabb_global_max_y[c_index];
    manager->cache.collision.aabb_global_max_z[i] = collision->aabb_global_max_z[c_index];
    
    manager->cache.collision.center_x[i] = collision->center_x[c_index];
    manager->cache.collision.center_y[i] = collision->center_y[c_index];
    manager->cache.collision.center_z[i] = collision->center_z[c_index];
    
    manager->cache.collision.internal_0[i] = collision->internal_0[c_index];
    manager->cache.collision.internal_1[i] = collision->internal_1[c_index];
    manager->cache.collision.internal_2[i] = collision->internal_2[c_index];
    manager->cache.collision.internal_3[i] = collision->internal_3[c_index];
    manager->cache.collision.internal_4[i] = collision->internal_4[c_index];
    manager->cache.collision.internal_5[i] = collision->internal_5[c_index];
    
    manager->cache.collision.shape[i] = collision->shape[c_index];
    manager->cache.collision.flag[i] = collision->flag[c_index];
    
    // rigid body
    manager->cache.rigid_body.i_body_00[i] = rigid_body->i_body_00[r_index];
    manager->cache.rigid_body.i_body_11[i] = rigid_body->i_body_11[r_index];
    manager->cache.rigid_body.i_body_22[i] = rigid_body->i_body_22[r_index];
    
    manager->cache.rigid_body.i_body_inv_00[i] = rigid_body->i_body_inv_00[r_index];
    manager->cache.rigid_body.i_body_inv_11[i] = rigid_body->i_body_inv_11[r_index];
    manager->cache.rigid_body.i_body_inv_22[i] = rigid_body->i_body_inv_22[r_index];
    
    manager->cache.rigid_body.i_inv_00[i] = rigid_body->i_inv_00[r_index];
    manager->cache.rigid_body.i_inv_01[i] = rigid_body->i_inv_01[r_index];
    manager->cache.rigid_body.i_inv_02[i] = rigid_body->i_inv_02[r_index];
    
    manager->cache.rigid_body.i_inv_10[i] = rigid_body->i_inv_10[r_index];
    manager->cache.rigid_body.i_inv_11[i] = rigid_body->i_inv_11[r_index];
    manager->cache.rigid_body.i_inv_12[i] = rigid_body->i_inv_12[r_index];
    
    manager->cache.rigid_body.i_inv_20[i] = rigid_body->i_inv_20[r_index];
    manager->cache.rigid_body.i_inv_21[i] = rigid_body->i_inv_21[r_index];
    manager->cache.rigid_body.i_inv_22[i] = rigid_body->i_inv_22[r_index];
}

void physics_system_update_values(dm_ecs_system* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    const dm_ecs_id r_id = manager->rigid_body;
    const dm_ecs_id c_id = manager->collision;
    
    component_transform*  transform  = dm_ecs_get_component_block(t_id, context);
    component_physics*    physics    = dm_ecs_get_component_block(p_id, context);
    component_collision*  collision  = dm_ecs_get_component_block(c_id, context);
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    
    uint32_t t_index, p_index, c_index, r_index;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        p_index = system->entity_indices[i][p_id];
        r_index = system->entity_indices[i][r_id];
        c_index = system->entity_indices[i][c_id];
        
        // transform
        transform->pos_x[t_index] = manager->cache.transform.pos_x[i];
        transform->pos_y[t_index] = manager->cache.transform.pos_y[i];
        transform->pos_z[t_index] = manager->cache.transform.pos_z[i];
        
        transform->rot_i[t_index] = manager->cache.transform.rot_i[i];
        transform->rot_j[t_index] = manager->cache.transform.rot_j[i];
        transform->rot_k[t_index] = manager->cache.transform.rot_k[i];
        transform->rot_r[t_index] = manager->cache.transform.rot_r[i];
        
        // physics
        physics->vel_x[p_index] = manager->cache.physics.vel_x[i];
        physics->vel_y[p_index] = manager->cache.physics.vel_y[i];
        physics->vel_z[p_index] = manager->cache.physics.vel_z[i];
        
        physics->w_x[p_index] = manager->cache.physics.w_x[i];
        physics->w_y[p_index] = manager->cache.physics.w_y[i];
        physics->w_z[p_index] = manager->cache.physics.w_z[i];
        
        physics->l_x[p_index] = manager->cache.physics.l_x[i];
        physics->l_y[p_index] = manager->cache.physics.l_y[i];
        physics->l_z[p_index] = manager->cache.physics.l_z[i];
        
        physics->force_x[p_index] = 0;
        physics->force_y[p_index] = 0;
        physics->force_z[p_index] = 0;
        
        physics->torque_x[p_index] = 0;
        physics->torque_y[p_index] = 0;
        physics->torque_z[p_index] = 0;
        
        // collision
        collision->aabb_global_min_x[c_index] = manager->cache.collision.aabb_global_min_x[i];
        collision->aabb_global_min_y[c_index] = manager->cache.collision.aabb_global_min_y[i];
        collision->aabb_global_min_z[c_index] = manager->cache.collision.aabb_global_min_z[i];
        
        collision->aabb_global_max_x[c_index] = manager->cache.collision.aabb_global_max_x[i];
        collision->aabb_global_max_y[c_index] = manager->cache.collision.aabb_global_max_y[i];
        collision->aabb_global_max_z[c_index] = manager->cache.collision.aabb_global_max_z[i];
        
        collision->flag[c_index] = manager->cache.collision.flag[i];
        
        // rigid body
        rigid_body->i_inv_00[r_index] = manager->cache.rigid_body.i_inv_00[i];
        rigid_body->i_inv_01[r_index] = manager->cache.rigid_body.i_inv_01[i];
        rigid_body->i_inv_02[r_index] = manager->cache.rigid_body.i_inv_02[i];
        
        rigid_body->i_inv_10[r_index] = manager->cache.rigid_body.i_inv_10[i];
        rigid_body->i_inv_11[r_index] = manager->cache.rigid_body.i_inv_11[i];
        rigid_body->i_inv_12[r_index] = manager->cache.rigid_body.i_inv_12[i];
        
        rigid_body->i_inv_20[r_index] = manager->cache.rigid_body.i_inv_20[i];
        rigid_body->i_inv_21[r_index] = manager->cache.rigid_body.i_inv_21[i];
        rigid_body->i_inv_22[r_index] = manager->cache.rigid_body.i_inv_22[i];
    }
}

bool physics_system_run(void* s, void* d)
{
    dm_context* context = d;
    dm_ecs_system*          system = s;
    physics_system_manager* manager = system->system_data;
    
    double total_time     = 0;
    double broad_time     = 0;
    double narrow_time    = 0;
    double collision_time = 0;
    double update_time    = 0;
    
    dm_timer t = { 0 };
    dm_timer full = { 0 };
    
    manager->accum_time += context->delta;
    
    uint32_t iters = 0;
    
    if(dm_input_key_just_pressed(DM_KEY_C, context))
    {
        if(manager->flags & DM_PHYSICS_FLAG_NO_COLLISIONS) manager->flags &= ~DM_PHYSICS_FLAG_NO_COLLISIONS;
        else manager->flags |= DM_PHYSICS_FLAG_NO_COLLISIONS;
    }
    
    dm_timer_start(&full, context);
    
    while(manager->accum_time >= DM_PHYSICS_FIXED_DT)
    {
        iters++;
        
        // broadphase
        if(!(manager->flags & DM_PHYSICS_FLAG_NO_COLLISIONS))
        {
            dm_timer_start(&t, context);
            if(!physics_system_broadphase(system, context)) return false;
            broad_time += dm_timer_elapsed_ms(&t, context);
            
            // narrowphase
            dm_timer_start(&t, context);
            if(!physics_system_narrowphase(system)) return false;
            narrow_time += dm_timer_elapsed_ms(&t, context);
            
            // collision resolution
            dm_timer_start(&t, context);
            physics_system_solve_constraints(manager);
            collision_time += dm_timer_elapsed_ms(&t, context);
        }
        
        // update
        dm_timer_start(&t, context);
        physics_system_update_entities(system);
        //physics_system_update_entities_simd(system);
        update_time += dm_timer_elapsed_ms(&t, context);
        
        manager->accum_time -= DM_PHYSICS_FIXED_DT;
    }
    
    physics_system_update_values(system, context);
    
    total_time = dm_timer_elapsed_ms(&full, context);
    
    float iter_f_inv = 1 / (float)iters;
    
    imgui_draw_text_fmt(20,20, 1,1,0,1, context, "Physics broadphase average: %0.3lf ms (%u checks)", broad_time * iter_f_inv, manager->broadphase_data.num_checks);
    imgui_draw_text_fmt(20,40, 1,1,0,1, context, "Physics narrowphase average: %0.3lf ms (%u checks)", narrow_time * iter_f_inv, manager->broadphase_data.num_possible_collisions);
    imgui_draw_text_fmt(20,60, 1,1,0,1, context, "Physics collision resolution average: %0.3lf ms (%u manifolds)", collision_time * iter_f_inv, manager->narrowphase_data.manifold_count);
    imgui_draw_text_fmt(20,80, 1,1,0,1, context, "Updating entities average: %0.3lf ms", update_time * iter_f_inv);
    imgui_draw_text_fmt(20,100, 1,0,1,1, context, "Physics took: %0.3lf ms, %u iterations", total_time, iters);
    
    return true;
}

/**********
BROADPHASE
uses a simple sort and sweep on the highest variance axis
************/
#ifdef DM_PLATFORM_LINUX
int physics_system_broadphase_sort_cmp(const void* a, const void* b, void* c)
#else
int physics_system_broadphase_sort_cmp(void* c, const void* a, const void* b)
#endif
{
    uint32_t entity_a = *(uint32_t*)a;
    uint32_t entity_b = *(uint32_t*)b;
    
    float* min_array = c;
    
    const float a_min = min_array[entity_a];
    const float b_min = min_array[entity_b];
    
    return (a_min > b_min) - (a_min < b_min);
    //return a_min < b_min;
}

void physics_system_broadphase_update_sphere_aabbs(uint32_t sphere_count, dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    
    uint32_t t_index, c_index;
    
    float center[3] = { 0 };
    
    component_transform* transform = &manager->cache.transform;
    component_collision* collision = &manager->cache.collision;
    
    for(uint32_t i=0; i<sphere_count; i++)
    {
        t_index = system->entity_indices[manager->broadphase_data.sphere_indices[i]][t_id];
        c_index = system->entity_indices[manager->broadphase_data.sphere_indices[i]][c_id];
        
        collision->aabb_global_min_x[c_index] = collision->aabb_local_min_x[c_index] + transform->pos_x[t_index];
        collision->aabb_global_min_y[c_index] = collision->aabb_local_min_y[c_index] + transform->pos_y[t_index];
        collision->aabb_global_min_z[c_index] = collision->aabb_local_min_z[c_index] + transform->pos_z[t_index];
        
        collision->aabb_global_max_x[c_index] = collision->aabb_local_max_x[c_index] + transform->pos_x[t_index];
        collision->aabb_global_max_y[c_index] = collision->aabb_local_max_y[c_index] + transform->pos_y[t_index];
        collision->aabb_global_max_z[c_index] = collision->aabb_local_max_z[c_index] + transform->pos_z[t_index];
        
        center[0] = 0.5f * (collision->aabb_global_max_x[c_index] + collision->aabb_global_min_x[c_index]);
        center[1] = 0.5f * (collision->aabb_global_max_y[c_index] + collision->aabb_global_min_y[c_index]);
        center[2] = 0.5f * (collision->aabb_global_max_z[c_index] + collision->aabb_global_min_z[c_index]);
        
        manager->broadphase_data.center_sum[0] += center[0];
        manager->broadphase_data.center_sum[1] += center[1];
        manager->broadphase_data.center_sum[2] += center[2];
        
        manager->broadphase_data.center_sq_sum[0] += center[0] * center[0];
        manager->broadphase_data.center_sq_sum[1] += center[1] * center[0];
        manager->broadphase_data.center_sq_sum[2] += center[2] * center[0];
    }
}

void physics_system_broadphase_update_box_aabbs(uint32_t box_count,  dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    
    uint32_t t_index, c_index;
    
    component_transform* transform = &manager->cache.transform;
    component_collision* collision = &manager->cache.collision;
    
    float center[3] = { 0 };
    
    float a,b      = 0.0f;
    float quat[N4] = { 0 };
    float rot[M3]  = { 0 };
    
    float world_min[3], world_max[3];
    float min[3], max[3];
    
    float xx,yy,zz;
    float xy,xz,xw,yz,yw,zw;
    
    for(uint32_t i=0; i<box_count; i++)
    {
        t_index = system->entity_indices[manager->broadphase_data.box_indices[i]][t_id];
        c_index = system->entity_indices[manager->broadphase_data.box_indices[i]][c_id];
        
        quat[0] = transform->rot_i[t_index];
        quat[1] = transform->rot_j[t_index];
        quat[2] = transform->rot_k[t_index];
        quat[3] = transform->rot_r[t_index];
        
        min[0] = collision->aabb_local_min_x[c_index];
        min[1] = collision->aabb_local_min_y[c_index];
        min[2] = collision->aabb_local_min_z[c_index];
        
        max[0] = collision->aabb_local_max_x[c_index];
        max[1] = collision->aabb_local_max_y[c_index];
        max[2] = collision->aabb_local_max_z[c_index];
        
        world_min[0] = world_max[0] = transform->pos_x[t_index];
        world_min[1] = world_max[1] = transform->pos_y[t_index];
        world_min[2] = world_max[2] = transform->pos_z[t_index];
        
        xx = quat[0] * quat[0];
        yy = quat[1] * quat[1];
        zz = quat[2] * quat[2];
        xy = quat[0] * quat[1];
        xz = quat[0] * quat[2];
        xw = quat[0] * quat[3];
        yz = quat[1] * quat[2];
        yw = quat[1] * quat[3];
        zw = quat[2] * quat[3];
        
        rot[0] = 2 * (yy + zz);
        rot[0] = 1 - rot[0];
        rot[1] = 2 * (xy + zw);
        rot[2] = 2 * (xz - yw);
        
        rot[3] = 2 * (xy - zw);
        rot[4] = 1 - 2 * (xx + zz);
        rot[5] = 2 * (yz + xw);
        
        rot[6] = 2 * (xz + yw);
        rot[7] = 2 * (yz - xw);
        rot[8] = 1 - 2 * (xx + yy);
        
        // x
        a = rot[0 * 3 + 0] * min[0];
        b = rot[0 * 3 + 0] * max[0];
        world_min[0] += DM_MIN(a,b);
        world_max[0] += DM_MAX(a,b);
        
        a = rot[1 * 3 + 0] * min[1];
        b = rot[1 * 3 + 0] * max[1];
        world_min[0] += DM_MIN(a,b);
        world_max[0] += DM_MAX(a,b);
        
        a = rot[2 * 3 + 0] * min[2];
        b = rot[2 * 3 + 0] * max[2];
        world_min[0] += DM_MIN(a,b);
        world_max[0] += DM_MAX(a,b);
        
        // y
        a = rot[0 * 3 + 1] * min[0];
        b = rot[0 * 3 + 1] * max[0];
        world_min[1] += DM_MIN(a,b);
        world_max[1] += DM_MAX(a,b);
        
        a = rot[1 * 3 + 1] * min[1];
        b = rot[1 * 3 + 1] * max[1];
        world_min[1] += DM_MIN(a,b);
        world_max[1] += DM_MAX(a,b);
        
        a = rot[2 * 3 + 1] * min[2];
        b = rot[2 * 3 + 1] * max[2];
        world_min[1] += DM_MIN(a,b);
        world_max[1] += DM_MAX(a,b);
        
        // z
        a = rot[0 * 3 + 2] * min[0];
        b = rot[0 * 3 + 2] * max[0];
        world_min[2] += DM_MIN(a,b);
        world_max[2] += DM_MAX(a,b);
        
        a = rot[1 * 3 + 2] * min[1];
        b = rot[1 * 3 + 2] * max[1];
        world_min[2] += DM_MIN(a,b);
        world_max[2] += DM_MAX(a,b);
        
        a = rot[2 * 3 + 2] * min[2];
        b = rot[2 * 3 + 2] * max[2];
        world_min[2] += DM_MIN(a,b);
        world_max[2] += DM_MAX(a,b);
        
        // assign
        collision->aabb_global_min_x[c_index] = world_min[0];
        collision->aabb_global_min_y[c_index] = world_min[1];
        collision->aabb_global_min_z[c_index] = world_min[2];
        
        collision->aabb_global_max_x[c_index] = world_max[0];
        collision->aabb_global_max_y[c_index] = world_max[1];
        collision->aabb_global_max_z[c_index] = world_max[2];
        
        // centers
        center[0] = 0.5f * (world_max[0] + world_min[0]);
        center[1] = 0.5f * (world_max[1] + world_min[1]);
        center[2] = 0.5f * (world_max[2] + world_min[2]);
        
        manager->broadphase_data.center_sum[0] += center[0];
        manager->broadphase_data.center_sum[1] += center[1];
        manager->broadphase_data.center_sum[2] += center[2];
        
        manager->broadphase_data.center_sq_sum[0] += center[0] * center[0];
        manager->broadphase_data.center_sq_sum[1] += center[1] * center[1];
        manager->broadphase_data.center_sq_sum[2] += center[2] * center[2];
    }
}

int physics_system_broadphase_get_variance_axis(dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    int axis = 0;
    
    const dm_ecs_id c_id = manager->collision;
    
    component_collision* collision = &manager->cache.collision;
    
    uint32_t box_count=0,sphere_count=0;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        manager->broadphase_data.sweep_indices[i] = i;
        collision->flag[system->entity_indices[i][c_id]] = COLLISION_FLAG_NO;
        
        switch(collision->shape[system->entity_indices[i][c_id]])
        {
            case DM_COLLISION_SHAPE_SPHERE: 
            manager->broadphase_data.sphere_indices[sphere_count++] = i;
            break;
            
            case DM_COLLISION_SHAPE_BOX: 
            manager->broadphase_data.box_indices[box_count++] = i;
            break;
            
            default:
            break;
        }
    }
    
    if(sphere_count) physics_system_broadphase_update_sphere_aabbs(sphere_count, system);
    if(box_count)    physics_system_broadphase_update_box_aabbs(box_count, system);
    
    const float scale = 1.0f / system->entity_count;
    
    float var[3];
    float max_var;
    
    dm_vec3_scale(manager->broadphase_data.center_sum, scale, manager->broadphase_data.center_sum);
    dm_vec3_scale(manager->broadphase_data.center_sq_sum, scale, manager->broadphase_data.center_sq_sum);
    
    dm_vec3_mul_vec3(manager->broadphase_data.center_sum, manager->broadphase_data.center_sum, manager->broadphase_data.center_sum);
    dm_vec3_sub_vec3(manager->broadphase_data.center_sq_sum, manager->broadphase_data.center_sum, var);
    
    max_var = -FLT_MAX;
    for(uint32_t i=0; i<N3; i++)
    {
        if(var[i] <= max_var) continue;
        
        max_var = var[i];
        axis = i;
    }
    
    return axis;
}

void physics_system_broadphase_sweep_naive(uint32_t count, float* min_array, float* max_array, physics_system_manager* manager)
{
    const dm_ecs_id c_id = manager->collision;
    
    component_collision* collision = &manager->cache.collision;
    
    float max_i, min_j;
    
    bool x_check, y_check, z_check;
    
    float a_max[3], a_min[3];
    float b_max[3], b_min[3];
    
    uint32_t entity_a, entity_b;
    
    uint32_t i,j;
    
    for(i=0; i<count; i++)
    {
        entity_a = manager->broadphase_data.sweep_indices[i];
        
        a_max[0] = collision->aabb_global_max_x[entity_a];
        a_max[1] = collision->aabb_global_max_y[entity_a];
        a_max[2] = collision->aabb_global_max_z[entity_a];
        
        a_min[0] = collision->aabb_global_min_x[entity_a];
        a_min[1] = collision->aabb_global_min_y[entity_a];
        a_min[2] = collision->aabb_global_min_z[entity_a];
        
        max_i = max_array[entity_a];
        
        for(j=i+1; j<count; j++)
        {
            entity_b = manager->broadphase_data.sweep_indices[j];
            
            b_max[0] = collision->aabb_global_max_x[entity_b];
            b_max[1] = collision->aabb_global_max_y[entity_b];
            b_max[2] = collision->aabb_global_max_z[entity_b];
            
            b_min[0] = collision->aabb_global_min_x[entity_b];
            b_min[1] = collision->aabb_global_min_y[entity_b];
            b_min[2] = collision->aabb_global_min_z[entity_b];
            
            min_j = min_array[entity_b];
            
            if(min_j > max_i) break;
            
            manager->broadphase_data.num_checks++;
            
            x_check = (a_min[0] <= b_max[0]) && (a_max[0] >= b_min[0]);
            y_check = (a_min[1] <= b_max[1]) && (a_max[1] >= b_min[1]);
            z_check = (a_min[2] <= b_max[2]) && (a_max[2] >= b_min[2]);
            
            if(!x_check || !y_check || !z_check) continue;
            
            collision->flag[entity_a] = COLLISION_FLAG_POSSIBLE;
            collision->flag[entity_b] = COLLISION_FLAG_POSSIBLE;
            
            manager->broadphase_data.possible_collisions[manager->broadphase_data.num_possible_collisions].entity_a = entity_a;
            manager->broadphase_data.possible_collisions[manager->broadphase_data.num_possible_collisions].entity_b = entity_b;
            manager->broadphase_data.num_possible_collisions++;
        }
    }
}


void physics_system_broadphase_sweep_naive_simd(uint32_t count, float* min_array, float* max_array, physics_system_manager* manager)
{
    physics_system_broadphase_data* broadphase_data = &manager->broadphase_data;
    physics_system_aabb_sort* aabbs_sorted = &broadphase_data->aabbs_sorted;
    const dm_ecs_id c_id = manager->collision;
    
    component_collision* collision = &manager->cache.collision;
    
    uint32_t i,j;
    uint32_t entity_a, entity_b;
    
    dm_mm256_float a_min_x, a_min_y, a_min_z;
    dm_mm256_float a_max_x, a_max_y, a_max_z;
    dm_mm256_float b_min_x, b_min_y, b_min_z;
    dm_mm256_float b_max_x, b_max_y, b_max_z;
    
    dm_mm256_float max_i, min_j;
    
    dm_mm256_float x_check, y_check, z_check;
    dm_mm256_float break_cond, intersect_mask;
    
#define N DM_SIMD256_FLOAT_N
    
    bool  a_possible = false;
    float b_possible[N] = { 0 };
    
    const dm_mm256_float ones = dm_mm256_set1_ps(1);
    dm_mm256_float mask;
    
    for(i=0; i < count; i++)
    {
        entity_a = broadphase_data->sweep_indices[i];
        
        a_min_x = dm_mm256_set1_ps(aabbs_sorted->min_x[i]);
        a_min_y = dm_mm256_set1_ps(aabbs_sorted->min_y[i]);
        a_min_z = dm_mm256_set1_ps(aabbs_sorted->min_z[i]);
        
        a_max_x = dm_mm256_set1_ps(aabbs_sorted->max_x[i]);
        a_max_y = dm_mm256_set1_ps(aabbs_sorted->max_y[i]);
        a_max_z = dm_mm256_set1_ps(aabbs_sorted->max_z[i]);
        
        max_i = dm_mm256_set1_ps(max_array[i]);
        
        a_possible = false;
        
        j = i + 1;
        
        for(; (count-j)>=N; j+=N)
        {
            dm_memzero(b_possible, sizeof(b_possible));
            
            b_min_x = dm_mm256_load_ps(aabbs_sorted->min_x + j);
            b_min_y = dm_mm256_load_ps(aabbs_sorted->min_y + j);
            b_min_z = dm_mm256_load_ps(aabbs_sorted->min_z + j);
            
            b_max_x = dm_mm256_load_ps(aabbs_sorted->max_x + j);
            b_max_y = dm_mm256_load_ps(aabbs_sorted->max_y + j);
            b_max_z = dm_mm256_load_ps(aabbs_sorted->max_z + j);
            
            min_j = dm_mm256_load_ps(min_array + j);
            
            // break check
            // if (min_j > max_i)
            break_cond = dm_mm256_gt_ps(min_j, max_i);
            break_cond = dm_mm256_and_ps(break_cond, ones);
            
            // if ALL elements are 1, everything is beyond i, so break
            if(dm_mm256_any_zero(break_cond)==0) break;
            
            // intersection checks
            x_check = dm_mm256_leq_ps(a_min_x, b_max_x);
            x_check = dm_mm256_and_ps(x_check, dm_mm256_geq_ps(a_max_x, b_min_x));
            y_check = dm_mm256_leq_ps(a_min_y, b_max_y);
            y_check = dm_mm256_and_ps(y_check, dm_mm256_geq_ps(a_max_y, b_min_y));
            z_check = dm_mm256_leq_ps(a_min_z, b_max_z);
            z_check = dm_mm256_and_ps(z_check, dm_mm256_geq_ps(a_max_z, b_min_z));
            
            // each element will be:
            // 1 if intersecting
            // 0 if not
            intersect_mask = dm_mm256_and_ps(x_check, y_check);
            intersect_mask = dm_mm256_and_ps(intersect_mask, z_check);
            intersect_mask = dm_mm256_and_ps(intersect_mask, ones);
            
            // if ALL elements are 0, nothing is intersecting
            if(dm_mm256_any_non_zero(intersect_mask)==0) continue;
            
            a_possible = true;
            dm_mm256_store_ps(b_possible, intersect_mask);
            
            for(uint32_t k=0; k<N; k++)
            {
                if(b_possible[k]==0) continue;
                
                entity_b = broadphase_data->sweep_indices[j+k];
                
                collision->flag[entity_b] = COLLISION_FLAG_POSSIBLE;
                
                broadphase_data->possible_collisions[broadphase_data->num_possible_collisions].entity_a = entity_a;
                broadphase_data->possible_collisions[broadphase_data->num_possible_collisions].entity_b = entity_b;
                manager->broadphase_data.num_possible_collisions++;
            }
            
            // finally, we need to break if ANY of the j's are beyond
            if(dm_mm256_any_non_zero(break_cond)) break;
        }
        
        if(a_possible) collision->flag[entity_a] = COLLISION_FLAG_POSSIBLE;
    }
}

void* physics_system_broadphase_sweep_multi_th(void* args)
{
    //uint32_t num_threads = dm_get_available_processor_count(context);
    //for(uint32_t
    physics_system_broadphase_multi_th_data* data = args;
    
    physics_system_aabb_sort* aabbs_sorted = data->aabbs_sorted;
    uint32_t*            indices   = data->indices;
    
    uint32_t i,j;
    uint32_t entity_a, entity_b;
    
    const uint32_t end_id = (data->thread_id + 1) * data->thread_count;
    
    float max_i, min_j;
    
    bool x_check, y_check, z_check;
    
    float a_max[3], a_min[3];
    float b_max[3], b_min[3];
    
    bool a_possible = false;
    
    for(i=data->start_id; i < end_id; i++)
    {
        entity_a = indices[i];
        
        a_max[0] = aabbs_sorted->max_x[i];
        a_max[1] = aabbs_sorted->max_y[i];
        a_max[2] = aabbs_sorted->max_z[i];
        
        a_min[0] = aabbs_sorted->min_x[i];
        a_min[1] = aabbs_sorted->min_y[i];
        a_min[2] = aabbs_sorted->min_z[i];
        
        max_i = data->max_array[i];
        
        a_possible = false;
        
        j = i + 1;
        for(; j<data->max_count; j++)
        {
            entity_b = indices[j];
            
            b_max[0] = aabbs_sorted->max_x[j];
            b_max[1] = aabbs_sorted->max_y[j];
            b_max[2] = aabbs_sorted->max_z[j];
            
            b_min[0] = aabbs_sorted->min_x[j];
            b_min[1] = aabbs_sorted->min_y[j];
            b_min[2] = aabbs_sorted->min_z[j];
            
            min_j = data->min_array[j];
            
            if(min_j > max_i) break;
            
            x_check = (a_min[0] <= b_max[0]) && (a_max[0] >= b_min[0]);
            y_check = (a_min[1] <= b_max[1]) && (a_max[1] >= b_min[1]);
            z_check = (a_min[2] <= b_max[2]) && (a_max[2] >= b_min[2]);
            
            if(!x_check || !y_check || !z_check) continue;
            
            a_possible = true;
            data->flag[entity_b] = COLLISION_FLAG_POSSIBLE;
            
            data->possible_collisions[data->possible_collision_count].entity_a = entity_a;
            data->possible_collisions[data->possible_collision_count].entity_b = entity_b;
            data->possible_collision_count++;
        }
        
        data->flag[entity_a] = COLLISION_FLAG_POSSIBLE;
    }
    
    return NULL;
}

void* physics_system_broadphase_sweep_multi_th_simd(void* args)
{
    physics_system_broadphase_multi_th_data* data = args;
    
    physics_system_aabb_sort* aabbs_sorted = data->aabbs_sorted;
    uint32_t*            indices   = data->indices;
    
    uint32_t i,j;
    uint32_t entity_a;
    
    const uint32_t end_id = (data->thread_id + 1) * data->thread_count;
    
    dm_mm256_float a_min_x, a_min_y, a_min_z;
    dm_mm256_float a_max_x, a_max_y, a_max_z;
    dm_mm256_float b_min_x, b_min_y, b_min_z;
    dm_mm256_float b_max_x, b_max_y, b_max_z;
    
    dm_mm256_float max_i, min_j;
    
    dm_mm256_float x_check, y_check, z_check;
    dm_mm256_float break_cond, intersect_mask;
    
#define N DM_SIMD256_FLOAT_N
    
    uint32_t entity_b[N];
    
    bool a_possible = false;
    float b_possible[N] = { 0 };
    
    const dm_mm256_float ones = dm_mm256_set1_ps(1);
    dm_mm256_float mask;
    
    for(i=data->start_id; i < end_id; i++)
    {
        entity_a = indices[i];
        
        a_min_x = dm_mm256_set1_ps(aabbs_sorted->min_x[i]);
        a_min_y = dm_mm256_set1_ps(aabbs_sorted->min_y[i]);
        a_min_z = dm_mm256_set1_ps(aabbs_sorted->min_z[i]);
        
        a_max_x = dm_mm256_set1_ps(aabbs_sorted->max_x[i]);
        a_max_y = dm_mm256_set1_ps(aabbs_sorted->max_y[i]);
        a_max_z = dm_mm256_set1_ps(aabbs_sorted->max_z[i]);
        
        max_i = dm_mm256_set1_ps(data->max_array[i]);
        
        a_possible = false;
        
        j = i + 1;
        
        for(; (data->max_count-j)>=N; j+=N)
        {
            dm_memzero(b_possible, sizeof(b_possible));
            
            b_min_x = dm_mm256_load_ps(aabbs_sorted->min_x + j);
            b_min_y = dm_mm256_load_ps(aabbs_sorted->min_y + j);
            b_min_z = dm_mm256_load_ps(aabbs_sorted->min_z + j);
            
            b_max_x = dm_mm256_load_ps(aabbs_sorted->max_x + j);
            b_max_y = dm_mm256_load_ps(aabbs_sorted->max_y + j);
            b_max_z = dm_mm256_load_ps(aabbs_sorted->max_z + j);
            
            min_j = dm_mm256_load_ps(data->min_array + j);
            
            // break check
            // if (min_j > max_i)
            break_cond = dm_mm256_gt_ps(min_j, max_i);
            break_cond = dm_mm256_and_ps(break_cond, ones);
            
            // if ALL elements are 1, everything is beyond i, so break
            if(dm_mm256_any_zero(break_cond)==0) break;
            
            // intersection checks
            x_check = dm_mm256_leq_ps(a_min_x, b_max_x);
            x_check = dm_mm256_and_ps(x_check, dm_mm256_geq_ps(a_max_x, b_min_x));
            y_check = dm_mm256_leq_ps(a_min_y, b_max_y);
            y_check = dm_mm256_and_ps(y_check, dm_mm256_geq_ps(a_max_y, b_min_y));
            z_check = dm_mm256_leq_ps(a_min_z, b_max_z);
            z_check = dm_mm256_and_ps(z_check, dm_mm256_geq_ps(a_max_z, b_min_z));
            
            // each element will be:
            // 1 if intersecting
            // 0 if not
            intersect_mask = dm_mm256_and_ps(x_check, y_check);
            intersect_mask = dm_mm256_and_ps(intersect_mask, z_check);
            intersect_mask = dm_mm256_and_ps(intersect_mask, ones);
            
            // if ALL elements are 0, nothing is intersecting
            if(dm_mm256_any_non_zero(intersect_mask)==0) continue;
            
            a_possible = true;
            dm_mm256_store_ps(b_possible, intersect_mask);
            
            for(uint32_t k=0; k<N; k++)
            {
                if(b_possible[k]==0) continue;
                
                data->flag[data->indices[j+k]] = COLLISION_FLAG_POSSIBLE;
                
                data->possible_collisions[data->possible_collision_count].entity_a = entity_a;
                data->possible_collisions[data->possible_collision_count].entity_b = indices[j+k];
                data->possible_collision_count++;
            }
            
            // finally, we need to break if ANY of the j's are beyond
            if(dm_mm256_any_non_zero(break_cond)) break;
        }
        
        if(a_possible) data->flag[entity_a] = COLLISION_FLAG_POSSIBLE;
    }
    
    return NULL;
}

void physics_system_broadphase_sort(uint32_t count, float* min_array, physics_system_manager* manager)
{
    // sort
#ifdef DM_PLATFORM_WIN32
    qsort_s(manager->broadphase_data.sweep_indices, count, sizeof(uint32_t), physics_system_broadphase_sort_cmp, min_array);
#elif defined(DM_PLATFORM_LINUX)
    qsort_r(manager->broadphase_data.sweep_indices, count, sizeof(uint32_t), physics_system_broadphase_sort_cmp, min_array);
#elif defined(DM_PLATFORM_APPLE)
    qsort_r(manager->broadphase_data.sweep_indices, count, sizeof(uint32_t), min_array, physics_system_broadphase_sort_cmp);
#endif
    
    // sort the aabb data
    component_collision* collision = &manager->cache.collision;
    
    uint32_t index;
    for(uint32_t i=0; i<count; i++)
    {
        index = manager->broadphase_data.sweep_indices[i];
        
        manager->broadphase_data.aabbs_sorted.min_x[i] = collision->aabb_global_min_x[index];
        manager->broadphase_data.aabbs_sorted.min_y[i] = collision->aabb_global_min_y[index];
        manager->broadphase_data.aabbs_sorted.min_z[i] = collision->aabb_global_min_z[index];
        
        manager->broadphase_data.aabbs_sorted.max_x[i] = collision->aabb_global_max_x[index];
        manager->broadphase_data.aabbs_sorted.max_y[i] = collision->aabb_global_max_y[index];
        manager->broadphase_data.aabbs_sorted.max_z[i] = collision->aabb_global_max_z[index];
    }
}

bool physics_system_broadphase_sweep(uint32_t count, float* min_array, float* max_array, dm_ecs_system* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    manager->broadphase_data.num_checks = 0;
    manager->broadphase_data.num_possible_collisions = 0;
    
    // sweep
    // if we don't have that many objects, no need to engage multiple threads
    if(0)
    {
        //physics_system_broadphase_sweep_naive(count, min_array, max_array, manager);
        physics_system_broadphase_sweep_naive_simd(count, min_array, max_array, manager);
    }
    else
    {
        physics_system_broadphase_multi_th_data* data = NULL;
        
        for(uint32_t i=0; i<manager->threadpool.thread_count; i++)
        {
            data = &manager->broadphase_data.sweep_mt_data[i];
            
            data->max_count = count;
            data->thread_count = count / manager->threadpool.thread_count;
            data->thread_id = i;
            data->start_id = i * data->thread_count;
            
            data->possible_collision_count = 0;
            
            data->aabbs_sorted = &manager->broadphase_data.aabbs_sorted;
            data->min_array = min_array;
            data->max_array = max_array;
            
            data->indices = manager->broadphase_data.sweep_indices;
            data->flag = manager->cache.collision.flag;
            
            dm_thread_task task = {
                //.func=physics_system_broadphase_sweep_multi_th,
                .func=physics_system_broadphase_sweep_multi_th_simd,
                .args=&manager->broadphase_data.sweep_mt_data[i]
            };
            
            manager->threadpool.total_task_count++;
            
            dm_threadpool_submit_task(&task, &manager->threadpool);
        }
        
        dm_threadpool_wait_for_completion(&manager->threadpool);
        
        for(uint32_t i=0; i<manager->threadpool.thread_count; i++)
        {
            physics_system_broadphase_multi_th_data* data = &manager->broadphase_data.sweep_mt_data[i];
            for(uint32_t j=0; j<data->possible_collision_count; j++)
            {
                manager->broadphase_data.possible_collisions[manager->broadphase_data.num_possible_collisions++] = data->possible_collisions[j]; 
            }
        }
    }
    
    return true;
}

// uses simple sort and sweep based on objects' aabbs
bool physics_system_broadphase(dm_ecs_system* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    // sort axis
    manager->broadphase_data.sort_axis = physics_system_broadphase_get_variance_axis(system);
    component_collision* collision = &manager->cache.collision;
    
    float* min, *max;
    
    switch(manager->broadphase_data.sort_axis)
    {
        case 0:
        min = collision->aabb_global_min_x;
        break;
        
        case 1: 
        min = collision->aabb_global_min_y;
        break;
        
        case 2: 
        min = collision->aabb_global_min_z;
        break;
    }
    
    // sort indices and aabbs
    physics_system_broadphase_sort(system->entity_count, min, manager);
    
    switch(manager->broadphase_data.sort_axis)
    {
        case 0:
        min = manager->broadphase_data.aabbs_sorted.min_x;
        max = manager->broadphase_data.aabbs_sorted.max_x;
        break;
        
        case 1: 
        min = manager->broadphase_data.aabbs_sorted.min_y;
        max = manager->broadphase_data.aabbs_sorted.max_y;
        break;
        
        case 2: 
        min = manager->broadphase_data.aabbs_sorted.min_z;
        max = manager->broadphase_data.aabbs_sorted.max_z;
        break;
    }
    
    // sweep
    physics_system_broadphase_sweep(system->entity_count, min, max, system, context);
    
    // reset our variance sums
    dm_memzero(manager->broadphase_data.center_sum, sizeof(float) * 3);
    dm_memzero(manager->broadphase_data.center_sq_sum, sizeof(float) * 3);
    
    return true;
}

/***********
NARROWPHASE
uses GJK to determine collisions
then EPA to determine penetration vector
then generates contact manifolds
*************/
bool physics_system_narrowphase(dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    manager->narrowphase_data.manifold_count = 0;
    
    float              pos[2][3], rots[2][4], cens[2][3], internals[2][6], vels[2][3], ws[2][3];
    dm_collision_shape shapes[2];
    
    physics_system_collision_pair collision_pair;
    uint32_t                      entity_a, entity_b;
    
    dm_contact_manifold* manifold;
    
    dm_simplex simplex;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    const dm_ecs_id p_id = manager->physics;
    const dm_ecs_id r_id = manager->rigid_body;
    
    component_transform*  transform  = &manager->cache.transform;
    component_collision*  collision  = &manager->cache.collision;
    component_physics*    physics    = &manager->cache.physics;
    component_rigid_body* rigid_body = &manager->cache.rigid_body;
    
    for(uint32_t i=0; i<manager->broadphase_data.num_possible_collisions; i++)
    {
        collision_pair = manager->broadphase_data.possible_collisions[i];
        
        entity_a = collision_pair.entity_a;
        entity_b = collision_pair.entity_b;
        
        // entity a
        pos[0][0]       = transform->pos_x[entity_a];
        pos[0][1]       = transform->pos_y[entity_a];
        pos[0][2]       = transform->pos_z[entity_a];
        rots[0][0]      = transform->rot_i[entity_a];
        rots[0][1]      = transform->rot_j[entity_a];
        rots[0][2]      = transform->rot_k[entity_a];
        rots[0][3]      = transform->rot_r[entity_a];
        cens[0][0]      = collision->center_x[entity_a];
        cens[0][1]      = collision->center_y[entity_a];
        cens[0][2]      = collision->center_z[entity_a];
        internals[0][0] = collision->internal_0[entity_a];
        internals[0][1] = collision->internal_1[entity_a];
        internals[0][2] = collision->internal_2[entity_a];
        internals[0][3] = collision->internal_3[entity_a];
        internals[0][4] = collision->internal_4[entity_a];
        internals[0][5] = collision->internal_5[entity_a];
        shapes[0]       = collision->shape[entity_a];
        vels[0][0]      = physics->vel_x[entity_a];
        vels[0][1]      = physics->vel_y[entity_a];
        vels[0][2]      = physics->vel_z[entity_a];
        ws[0][0]        = physics->w_x[entity_a];
        ws[0][1]        = physics->w_y[entity_a];
        ws[0][2]        = physics->w_z[entity_a];
        
        // entity b
        pos[1][0]       = transform->pos_x[entity_b];
        pos[1][1]       = transform->pos_y[entity_b];
        pos[1][2]       = transform->pos_z[entity_b];
        rots[1][0]      = transform->rot_i[entity_b];
        rots[1][1]      = transform->rot_j[entity_b];
        rots[1][2]      = transform->rot_k[entity_b];
        rots[1][3]      = transform->rot_r[entity_b];
        cens[1][0]      = collision->center_x[entity_b];
        cens[1][1]      = collision->center_y[entity_b];
        cens[1][2]      = collision->center_z[entity_b];
        internals[1][0] = collision->internal_0[entity_b];
        internals[1][1] = collision->internal_1[entity_b];
        internals[1][2] = collision->internal_2[entity_b];
        internals[1][3] = collision->internal_3[entity_b];
        internals[1][4] = collision->internal_4[entity_b];
        internals[1][5] = collision->internal_5[entity_b];
        shapes[1]       = collision->shape[entity_b];
        vels[1][0]      = physics->vel_x[entity_b];
        vels[1][1]      = physics->vel_y[entity_b];
        vels[1][2]      = physics->vel_z[entity_b];
        ws[1][0]        = physics->w_x[entity_b];
        ws[1][1]        = physics->w_y[entity_b];
        ws[1][2]        = physics->w_z[entity_b];
        
        //////
        simplex = (dm_simplex){ 0 };
        
        float supports[2][3];
        if(!dm_physics_gjk(pos, rots, cens, internals, shapes, supports, &simplex)) continue;
        
        assert(simplex.size==4);
        
        manifold = &manager->narrowphase_data.manifolds[manager->narrowphase_data.manifold_count++];
        *manifold = (dm_contact_manifold){ 0 };
        
        collision->flag[entity_a] = COLLISION_FLAG_YES;
        
        manifold->contact_data[0].vel_x         = &physics->vel_x[entity_b];
        manifold->contact_data[0].vel_y         = &physics->vel_y[entity_a];
        manifold->contact_data[0].vel_z         = &physics->vel_z[entity_a];
        manifold->contact_data[0].w_x           = &physics->w_x[entity_a];
        manifold->contact_data[0].w_y           = &physics->w_y[entity_a];
        manifold->contact_data[0].w_z           = &physics->w_z[entity_a];
        manifold->contact_data[0].mass          = physics->mass[entity_a];
        manifold->contact_data[0].inv_mass      = physics->inv_mass[entity_a];
        manifold->contact_data[0].v_damp        = physics->damping_v[entity_a];
        manifold->contact_data[0].w_damp        = physics->damping_w[entity_a];
        manifold->contact_data[0].i_body_inv_00 = rigid_body->i_body_inv_00[entity_a];
        manifold->contact_data[0].i_body_inv_11 = rigid_body->i_body_inv_11[entity_a];
        manifold->contact_data[0].i_body_inv_22 = rigid_body->i_body_inv_22[entity_a];
        
        collision->flag[entity_b] = COLLISION_FLAG_YES;
        
        manifold->contact_data[1].vel_x         = &physics->vel_x[entity_b];
        manifold->contact_data[1].vel_y         = &physics->vel_y[entity_b];
        manifold->contact_data[1].vel_z         = &physics->vel_z[entity_b];
        manifold->contact_data[1].w_x           = &physics->w_x[entity_b];
        manifold->contact_data[1].w_y           = &physics->w_y[entity_b];
        manifold->contact_data[1].w_z           = &physics->w_z[entity_b];
        manifold->contact_data[1].mass          = physics->mass[entity_b];
        manifold->contact_data[1].inv_mass      = physics->inv_mass[entity_b];
        manifold->contact_data[1].v_damp        = physics->damping_v[entity_b];
        manifold->contact_data[1].w_damp        = physics->damping_w[entity_b];
        manifold->contact_data[1].i_body_inv_00 = rigid_body->i_body_inv_00[entity_b];
        manifold->contact_data[1].i_body_inv_11 = rigid_body->i_body_inv_11[entity_b];
        manifold->contact_data[1].i_body_inv_22 = rigid_body->i_body_inv_22[entity_b];
        
        if(!dm_physics_collide_entities(pos, rots, cens, internals, vels, ws, shapes, &simplex, manifold)) return false;
    }
    
    return true;
}

/********************
COLLISION RESOLUTION
solves constraints generated in narrowphase
using impulse solver
**********************/
void physics_system_solve_constraints(physics_system_manager* manager)
{
    for(uint32_t iter=0; iter<PHYSICS_SYSTEM_CONSTRAINT_ITER; iter++)
    {
        for(uint32_t m=0; m<manager->narrowphase_data.manifold_count; m++)
        {
            dm_physics_apply_constraints(&manager->narrowphase_data.manifolds[m]);
        }
    }
}

/********
UPDATING
**********/
// this is a straight run through of the entities
// this is done as explicitly as possible, with as few steps per line as possible
// so that one can more easily convert this to a SIMD version.
// thus we aren't using the built-in vector/matrix functions
// but hardcoding it in
void physics_system_update_entities(dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    component_transform*  transform  = &manager->cache.transform;
    component_physics*    physics    = &manager->cache.physics;
    component_rigid_body* rigid_body = &manager->cache.rigid_body;
    
    float dt_mass;
    
    float new_rot_i, new_rot_j, new_rot_k, new_rot_r;
    float new_rot_mag;
    
    float orientation_00, orientation_01, orientation_02;
    float orientation_10, orientation_11, orientation_12;
    float orientation_20, orientation_21, orientation_22;
    
    float body_inv_00, body_inv_01, body_inv_02;
    float body_inv_10, body_inv_11, body_inv_12;
    float body_inv_20, body_inv_21, body_inv_22;
    
    const static float half_dt = 0.5f * DM_PHYSICS_FIXED_DT;
    
    uint32_t i=0;
    
    i=0;
    for(i=0; i<system->entity_count; i++)
    {
        // integrate position
        transform->pos_x[i] += physics->vel_x[i] * DM_PHYSICS_FIXED_DT;
        transform->pos_y[i] += physics->vel_y[i] * DM_PHYSICS_FIXED_DT;
        transform->pos_z[i] += physics->vel_z[i] * DM_PHYSICS_FIXED_DT;
        
        // integrate velocity
        dt_mass = physics->inv_mass[i] * DM_PHYSICS_FIXED_DT;
        
        physics->vel_x[i] += physics->force_x[i] * dt_mass;
        physics->vel_y[i] += physics->force_y[i] * dt_mass;
        physics->vel_z[i] += physics->force_z[i] * dt_mass;
        
        // integrate angular momentum
        physics->l_x[i] += physics->torque_x[i] * DM_PHYSICS_FIXED_DT;
        physics->l_y[i] += physics->torque_y[i] * DM_PHYSICS_FIXED_DT;
        physics->l_z[i] += physics->torque_z[i] * DM_PHYSICS_FIXED_DT;
        
        // integrate angular velocity
        physics->w_x[i] += rigid_body->i_inv_00[i] * physics->l_x[i];
        physics->w_x[i] += rigid_body->i_inv_01[i] * physics->l_y[i];
        physics->w_x[i] += rigid_body->i_inv_02[i] * physics->l_z[i];
        
        physics->w_y[i] += rigid_body->i_inv_10[i] * physics->l_x[i];
        physics->w_y[i] += rigid_body->i_inv_11[i] * physics->l_y[i];
        physics->w_y[i] += rigid_body->i_inv_12[i] * physics->l_z[i];
        
        physics->w_z[i] += rigid_body->i_inv_20[i] * physics->l_x[i];
        physics->w_z[i] += rigid_body->i_inv_21[i] * physics->l_y[i];
        physics->w_z[i] += rigid_body->i_inv_22[i] * physics->l_z[i];
        
        assert(physics->w_x[i]==physics->w_x[i] && physics->w_y[i]==physics->w_y[i] && physics->w_z[i]==physics->w_z[i]);
        
        // integrate rotation
        new_rot_i  = physics->w_x[i] * transform->rot_r[i];
        new_rot_i += physics->w_y[i] * transform->rot_k[i];
        new_rot_i -= physics->w_z[i] * transform->rot_j[i];
        new_rot_i *= half_dt;
        new_rot_i += transform->rot_i[i];
        
        new_rot_j  = -physics->w_x[i] * transform->rot_k[i];
        new_rot_j +=  physics->w_y[i] * transform->rot_r[i];
        new_rot_j +=  physics->w_z[i] * transform->rot_i[i];
        new_rot_j *= half_dt;
        new_rot_j += transform->rot_j[i];
        
        new_rot_k  = physics->w_x[i] * transform->rot_j[i];
        new_rot_k -= physics->w_y[i] * transform->rot_i[i];
        new_rot_k += physics->w_z[i] * transform->rot_r[i];
        new_rot_k *= half_dt;
        new_rot_k += transform->rot_k[i];
        
        new_rot_r  = -physics->w_x[i] * transform->rot_i[i];
        new_rot_r -=  physics->w_y[i] * transform->rot_j[i];
        new_rot_r -=  physics->w_z[i] * transform->rot_k[i];
        new_rot_r *= half_dt;
        new_rot_r += transform->rot_r[i];
        
        new_rot_mag  = new_rot_i * new_rot_i;
        new_rot_mag += new_rot_j * new_rot_j;
        new_rot_mag += new_rot_k * new_rot_k;
        new_rot_mag += new_rot_r * new_rot_r;
        new_rot_mag  = dm_sqrtf(new_rot_mag);
        
        new_rot_i /= new_rot_mag;
        new_rot_j /= new_rot_mag;
        new_rot_k /= new_rot_mag;
        new_rot_r /= new_rot_mag;
        
        transform->rot_i[i] = new_rot_i;
        transform->rot_j[i] = new_rot_j;
        transform->rot_k[i] = new_rot_k;
        transform->rot_r[i] = new_rot_r;
        
        // update i_inv
        orientation_00  = new_rot_j * new_rot_j;
        orientation_00 += new_rot_k * new_rot_k;
        orientation_00 *= 2;
        orientation_00  = 1 - orientation_00;
        
        orientation_01  = new_rot_i * new_rot_j;
        orientation_01 += new_rot_k * new_rot_r;
        orientation_01 *= 2;
        
        orientation_02  = new_rot_i * new_rot_k;
        orientation_02 -= new_rot_j * new_rot_r;
        orientation_02 *= 2;
        
        orientation_10  = new_rot_i * new_rot_j;
        orientation_10 -= new_rot_k * new_rot_r;
        orientation_10 *= 2;
        
        orientation_11  = new_rot_i * new_rot_i;
        orientation_11 += new_rot_k * new_rot_k;
        orientation_11 *= 2;
        orientation_11  = 1 - orientation_11;
        
        orientation_12  = new_rot_j * new_rot_k;
        orientation_12 += new_rot_i * new_rot_r;
        orientation_12 *= 2;
        
        orientation_20  = new_rot_i * new_rot_k;
        orientation_20 += new_rot_j * new_rot_r;
        orientation_20 *= 2;
        
        orientation_21  = new_rot_j * new_rot_k;
        orientation_21 -= new_rot_i * new_rot_r;
        orientation_21 *= 2;
        
        orientation_22  = new_rot_i * new_rot_i;
        orientation_22 += new_rot_j * new_rot_j;
        orientation_22 *= 2;
        orientation_22  = 1 - orientation_22;
        
        // orientation is transposed here
        body_inv_00 = orientation_00 * rigid_body->i_inv_00[i];
        body_inv_01 = orientation_10 * rigid_body->i_inv_11[i];
        body_inv_02 = orientation_20 * rigid_body->i_inv_22[i];
        
        body_inv_10 = orientation_01 * rigid_body->i_inv_00[i];
        body_inv_11 = orientation_11 * rigid_body->i_inv_11[i];
        body_inv_12 = orientation_21 * rigid_body->i_inv_22[i];
        
        body_inv_20 = orientation_02 * rigid_body->i_inv_00[i];
        body_inv_21 = orientation_12 * rigid_body->i_inv_11[i];
        body_inv_22 = orientation_22 * rigid_body->i_inv_22[i];
        
        // final i_inv matrix
        rigid_body->i_inv_00[i]  = orientation_00 * body_inv_00;
        rigid_body->i_inv_00[i] += orientation_01 * body_inv_10;
        rigid_body->i_inv_00[i] += orientation_02 * body_inv_20;
        
        rigid_body->i_inv_01[i]  = orientation_00 * body_inv_01;
        rigid_body->i_inv_01[i] += orientation_01 * body_inv_11;
        rigid_body->i_inv_01[i] += orientation_02 * body_inv_21;
        
        rigid_body->i_inv_02[i]  = orientation_00 * body_inv_02;
        rigid_body->i_inv_02[i] += orientation_01 * body_inv_12;
        rigid_body->i_inv_02[i] += orientation_02 * body_inv_22;
        
        rigid_body->i_inv_10[i]  = orientation_10 * body_inv_00;
        rigid_body->i_inv_10[i] += orientation_11 * body_inv_10;
        rigid_body->i_inv_10[i] += orientation_12 * body_inv_20;
        
        rigid_body->i_inv_11[i]  = orientation_10 * body_inv_01;
        rigid_body->i_inv_11[i] += orientation_11 * body_inv_11;
        rigid_body->i_inv_11[i] += orientation_12 * body_inv_21;
        
        rigid_body->i_inv_12[i]  = orientation_10 * body_inv_02;
        rigid_body->i_inv_12[i] += orientation_11 * body_inv_12;
        rigid_body->i_inv_12[i] += orientation_12 * body_inv_22;
        
        rigid_body->i_inv_20[i]  = orientation_20 * body_inv_00;
        rigid_body->i_inv_20[i] += orientation_21 * body_inv_10;
        rigid_body->i_inv_20[i] += orientation_22 * body_inv_20;
        
        rigid_body->i_inv_21[i]  = orientation_20 * body_inv_01;
        rigid_body->i_inv_21[i] += orientation_21 * body_inv_11;
        rigid_body->i_inv_21[i] += orientation_22 * body_inv_21;
        
        rigid_body->i_inv_22[i]  = orientation_20 * body_inv_02;
        rigid_body->i_inv_22[i] += orientation_21 * body_inv_12;
        rigid_body->i_inv_22[i] += orientation_22 * body_inv_22;
    }
}

void physics_system_update_entities_simd(dm_ecs_system* system)
{
    physics_system_manager* manager = system->system_data;
    
    component_transform*  transform = &manager->cache.transform;
    component_physics*    physics   = &manager->cache.physics;
    component_rigid_body* rigid_body = &manager->cache.rigid_body;
    
    dm_mm256_float pos_x, pos_y, pos_z;
    dm_mm256_float rot_i, rot_j, rot_k, rot_r;
    dm_mm256_float vel_x, vel_y, vel_z;
    dm_mm256_float w_x, w_y, w_z;
    dm_mm256_float l_x, l_y, l_z;
    dm_mm256_float force_x, force_y, force_z;
    dm_mm256_float torque_x, torque_y, torque_z;
    dm_mm256_float dt_mass;
    dm_mm256_float i_inv_00, i_inv_01, i_inv_02;
    dm_mm256_float i_inv_10, i_inv_11, i_inv_12;
    dm_mm256_float i_inv_20, i_inv_21, i_inv_22;
    
    dm_mm256_float new_rot_i, new_rot_j, new_rot_k, new_rot_r;
    dm_mm256_float new_rot_mag;
    
    dm_mm256_float orientation_00, orientation_01, orientation_02;
    dm_mm256_float orientation_10, orientation_11, orientation_12;
    dm_mm256_float orientation_20, orientation_21, orientation_22;
    
    dm_mm256_float body_inv_00, body_inv_01, body_inv_02;
    dm_mm256_float body_inv_10, body_inv_11, body_inv_12;
    dm_mm256_float body_inv_20, body_inv_21, body_inv_22;
    
    dm_mm256_float dt      = dm_mm256_set1_ps(DM_PHYSICS_FIXED_DT);
    dm_mm256_float half_dt = dm_mm256_set1_ps(0.5f * DM_PHYSICS_FIXED_DT);
    dm_mm256_float ones    = dm_mm256_set1_ps(1);
    dm_mm256_float twos    = dm_mm256_set1_ps(2);
    dm_mm256_float zeroes  = dm_mm256_set1_ps(0);
    
    uint32_t i=0;
    for(; (system->entity_count-i)>DM_SIMD256_FLOAT_N; i+=DM_SIMD256_FLOAT_N)
    {
        pos_x = dm_mm256_load_ps(transform->pos_x + i);
        pos_y = dm_mm256_load_ps(transform->pos_y + i);
        pos_z = dm_mm256_load_ps(transform->pos_z + i);
        
        rot_i = dm_mm256_load_ps(transform->rot_i + i);
        rot_j = dm_mm256_load_ps(transform->rot_j + i);
        rot_k = dm_mm256_load_ps(transform->rot_k + i);
        rot_r = dm_mm256_load_ps(transform->rot_r + i);
        
        vel_x = dm_mm256_load_ps(physics->vel_x + i);
        vel_y = dm_mm256_load_ps(physics->vel_y + i);
        vel_z = dm_mm256_load_ps(physics->vel_z + i);
        
        w_x = dm_mm256_load_ps(physics->w_x + i);
        w_y = dm_mm256_load_ps(physics->w_y + i);
        w_z = dm_mm256_load_ps(physics->w_z + i);
        
        l_x = dm_mm256_load_ps(physics->l_x + i);
        l_y = dm_mm256_load_ps(physics->l_y + i);
        l_z = dm_mm256_load_ps(physics->l_z + i);
        
        force_x = dm_mm256_load_ps(physics->force_x + i);
        force_y = dm_mm256_load_ps(physics->force_y + i);
        force_z = dm_mm256_load_ps(physics->force_z + i);
        
        torque_x = dm_mm256_load_ps(physics->torque_x + i);
        torque_y = dm_mm256_load_ps(physics->torque_y + i);
        torque_z = dm_mm256_load_ps(physics->torque_z + i);
        
        dt_mass = dm_mm256_load_ps(physics->inv_mass + i);
        dt_mass = dm_mm256_mul_ps(dt_mass, dt);
        
        i_inv_00 = dm_mm256_load_ps(rigid_body->i_inv_00 + i);
        i_inv_01 = dm_mm256_load_ps(rigid_body->i_inv_01 + i);
        i_inv_02 = dm_mm256_load_ps(rigid_body->i_inv_02 + i);
        
        i_inv_10 = dm_mm256_load_ps(rigid_body->i_inv_10 + i);
        i_inv_11 = dm_mm256_load_ps(rigid_body->i_inv_11 + i);
        i_inv_12 = dm_mm256_load_ps(rigid_body->i_inv_12 + i);
        
        i_inv_20 = dm_mm256_load_ps(rigid_body->i_inv_20 + i);
        i_inv_21 = dm_mm256_load_ps(rigid_body->i_inv_21 + i);
        i_inv_22 = dm_mm256_load_ps(rigid_body->i_inv_22 + i);
        
        // integrate position
        pos_x = dm_mm256_fmadd_ps(vel_x, dt, pos_x);
        pos_y = dm_mm256_fmadd_ps(vel_y, dt, pos_y);
        pos_z = dm_mm256_fmadd_ps(vel_z, dt, pos_z);
        
        // integrate velocity
        vel_x = dm_mm256_fmadd_ps(force_x, dt_mass, vel_x);
        vel_y = dm_mm256_fmadd_ps(force_y, dt_mass, vel_y);
        vel_z = dm_mm256_fmadd_ps(force_z, dt_mass, vel_z);
        
        // integrate angular momentum
        l_x = dm_mm256_fmadd_ps(torque_x, dt, l_x);
        l_y = dm_mm256_fmadd_ps(torque_y, dt, l_y);
        l_z = dm_mm256_fmadd_ps(torque_z, dt, l_z);
        
        // integrate angular velocity
        w_x = dm_mm256_fmadd_ps(i_inv_00, l_x, w_x);
        w_x = dm_mm256_fmadd_ps(i_inv_01, l_y, w_x);
        w_x = dm_mm256_fmadd_ps(i_inv_02, l_z, w_x);
        
        w_y = dm_mm256_fmadd_ps(i_inv_10, l_x, w_y);
        w_y = dm_mm256_fmadd_ps(i_inv_11, l_y, w_y);
        w_y = dm_mm256_fmadd_ps(i_inv_12, l_z, w_y);
        
        w_z = dm_mm256_fmadd_ps(i_inv_20, l_x, w_z);
        w_z = dm_mm256_fmadd_ps(i_inv_21, l_y, w_z);
        w_z = dm_mm256_fmadd_ps(i_inv_22, l_z, w_z);
        
        // integrate rotation
        new_rot_i = dm_mm256_mul_ps(w_x, rot_r);
        new_rot_i = dm_mm256_fmadd_ps(w_x, rot_r, new_rot_i);
        new_rot_i = dm_mm256_fmadd_ps(w_y, rot_k, new_rot_i);
        new_rot_i = dm_mm256_sub_ps(new_rot_i, dm_mm256_mul_ps(w_z, rot_j));
        new_rot_i = dm_mm256_fmadd_ps(new_rot_i, half_dt, rot_i);
        
        new_rot_j = dm_mm256_sub_ps(zeroes, dm_mm256_mul_ps(w_x, rot_k));
        new_rot_j = dm_mm256_fmadd_ps(w_y, rot_r, new_rot_j);
        new_rot_j = dm_mm256_fmadd_ps(w_z, rot_i, new_rot_j);
        new_rot_j = dm_mm256_fmadd_ps(new_rot_i, half_dt, rot_j);
        
        new_rot_k = dm_mm256_mul_ps(w_x, rot_j);
        new_rot_k = dm_mm256_sub_ps(new_rot_k, dm_mm256_mul_ps(w_x, rot_i));
        new_rot_k = dm_mm256_fmadd_ps(w_z, rot_r, new_rot_k);
        new_rot_k = dm_mm256_fmadd_ps(new_rot_k, half_dt, rot_k);
        
        new_rot_r = dm_mm256_sub_ps(zeroes, dm_mm256_mul_ps(w_x, rot_i));
        new_rot_r = dm_mm256_sub_ps(new_rot_r, dm_mm256_mul_ps(w_y, rot_j));
        new_rot_r = dm_mm256_sub_ps(new_rot_r, dm_mm256_mul_ps(w_z, rot_k));
        new_rot_r = dm_mm256_fmadd_ps(new_rot_r, half_dt, rot_r);
        
        new_rot_mag = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_j, new_rot_j, new_rot_mag);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, new_rot_mag);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_r, new_rot_r, new_rot_mag);;
        new_rot_mag = dm_mm256_sqrt_ps(new_rot_mag);
        new_rot_mag = dm_mm256_div_ps(ones, new_rot_mag);
        
        new_rot_i = dm_mm256_mul_ps(new_rot_i, new_rot_mag);
        new_rot_j = dm_mm256_mul_ps(new_rot_j, new_rot_mag);
        new_rot_k = dm_mm256_mul_ps(new_rot_k, new_rot_mag);
        new_rot_r = dm_mm256_mul_ps(new_rot_r, new_rot_mag);
        
        // update i_inv
        orientation_00 = dm_mm256_mul_ps(new_rot_j, new_rot_j);
        orientation_00 = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, orientation_00);
        orientation_00 = dm_mm256_mul_ps(twos, orientation_00);
        orientation_00 = dm_mm256_sub_ps(ones, orientation_00);
        
        orientation_01 = dm_mm256_mul_ps(new_rot_i, new_rot_j);
        orientation_01 = dm_mm256_fmadd_ps(new_rot_k, new_rot_r, orientation_01);
        orientation_01 = dm_mm256_mul_ps(twos, orientation_01);
        
        orientation_02 = dm_mm256_mul_ps(new_rot_i, new_rot_k);
        orientation_02 = dm_mm256_sub_ps(orientation_02, dm_mm256_mul_ps(new_rot_j, new_rot_r));
        orientation_02 = dm_mm256_mul_ps(twos, orientation_02);
        
        orientation_10 = dm_mm256_mul_ps(new_rot_i, new_rot_j);
        orientation_10 = dm_mm256_sub_ps(orientation_10, dm_mm256_mul_ps(new_rot_k, new_rot_r));
        orientation_10 = dm_mm256_mul_ps(twos, orientation_10);
        
        orientation_11 = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        orientation_11 = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, orientation_11);
        orientation_11 = dm_mm256_mul_ps(twos, orientation_11);
        orientation_11 = dm_mm256_sub_ps(ones, orientation_11);
        
        orientation_12 = dm_mm256_mul_ps(new_rot_j, new_rot_k);
        orientation_12 = dm_mm256_fmadd_ps(new_rot_i, new_rot_r, orientation_12);
        orientation_12 = dm_mm256_mul_ps(twos, orientation_12);
        
        orientation_20 = dm_mm256_mul_ps(new_rot_i, new_rot_k);
        orientation_20 = dm_mm256_fmadd_ps(new_rot_j, new_rot_r, orientation_20);
        orientation_20 = dm_mm256_mul_ps(twos, orientation_20);
        
        orientation_21 = dm_mm256_mul_ps(new_rot_j, new_rot_k);
        orientation_21 = dm_mm256_sub_ps(orientation_21, dm_mm256_mul_ps(new_rot_i, new_rot_r));
        orientation_21 = dm_mm256_mul_ps(twos, orientation_21);
        
        orientation_22 = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        orientation_22 = dm_mm256_fmadd_ps(new_rot_j, new_rot_j, orientation_22);
        orientation_22 = dm_mm256_mul_ps(twos, orientation_22);
        orientation_22 = dm_mm256_sub_ps(ones, orientation_22);
        
        // orientation is transposed here
        body_inv_00 = dm_mm256_mul_ps(orientation_00, i_inv_00);
        body_inv_01 = dm_mm256_mul_ps(orientation_10, i_inv_11);
        body_inv_02 = dm_mm256_mul_ps(orientation_20, i_inv_22);
        
        body_inv_10 = dm_mm256_mul_ps(orientation_01, i_inv_00);
        body_inv_11 = dm_mm256_mul_ps(orientation_11, i_inv_11);
        body_inv_12 = dm_mm256_mul_ps(orientation_21, i_inv_22);
        
        body_inv_20 = dm_mm256_mul_ps(orientation_02, i_inv_00);
        body_inv_21 = dm_mm256_mul_ps(orientation_12, i_inv_11);
        body_inv_22 = dm_mm256_mul_ps(orientation_22, i_inv_22);
        
        // final i_inv matrix
        i_inv_00 = dm_mm256_mul_ps(orientation_00, body_inv_00);
        i_inv_00 = dm_mm256_fmadd_ps(orientation_01, body_inv_10, i_inv_00);
        i_inv_00 = dm_mm256_fmadd_ps(orientation_02, body_inv_20, i_inv_00);
        
        i_inv_01 = dm_mm256_mul_ps(orientation_00, body_inv_01);
        i_inv_01 = dm_mm256_fmadd_ps(orientation_01, body_inv_11, i_inv_01);
        i_inv_01 = dm_mm256_fmadd_ps(orientation_02, body_inv_21, i_inv_01);;
        
        i_inv_02 = dm_mm256_mul_ps(orientation_00, body_inv_02);
        i_inv_02 = dm_mm256_fmadd_ps(orientation_01, body_inv_12, i_inv_02);
        i_inv_02 = dm_mm256_fmadd_ps(orientation_02, body_inv_22, i_inv_02);
        
        i_inv_10 = dm_mm256_mul_ps(orientation_10, body_inv_00);
        i_inv_10 = dm_mm256_fmadd_ps(orientation_11, body_inv_10, i_inv_10);
        i_inv_10 = dm_mm256_fmadd_ps(orientation_12, body_inv_20, i_inv_10);
        
        i_inv_11 = dm_mm256_mul_ps(orientation_10, body_inv_01);
        i_inv_11 = dm_mm256_fmadd_ps(orientation_11, body_inv_11, i_inv_11);
        i_inv_11 = dm_mm256_fmadd_ps(orientation_12, body_inv_21, i_inv_11);
        
        i_inv_12 = dm_mm256_mul_ps(orientation_10, body_inv_02);
        i_inv_12 = dm_mm256_fmadd_ps(orientation_11, body_inv_12, i_inv_12);
        i_inv_12 = dm_mm256_fmadd_ps(orientation_12, body_inv_22, i_inv_12);
        
        i_inv_20 = dm_mm256_mul_ps(orientation_20, body_inv_00);
        i_inv_20 = dm_mm256_fmadd_ps(orientation_21, body_inv_10, i_inv_20);
        i_inv_20 = dm_mm256_fmadd_ps(orientation_22, body_inv_20, i_inv_20);
        
        i_inv_21 = dm_mm256_mul_ps(orientation_20, body_inv_01);
        i_inv_21 = dm_mm256_fmadd_ps(orientation_21, body_inv_11, i_inv_21);
        i_inv_21 = dm_mm256_fmadd_ps(orientation_22, body_inv_21, i_inv_21);
        
        i_inv_22 = dm_mm256_mul_ps(orientation_20, body_inv_02);
        i_inv_22 = dm_mm256_fmadd_ps(orientation_21, body_inv_12, i_inv_22);
        i_inv_22 = dm_mm256_fmadd_ps(orientation_22, body_inv_22, i_inv_22);
        
        // store
        dm_mm256_store_ps(transform->pos_x + i, pos_x);
        dm_mm256_store_ps(transform->pos_y + i, pos_y);
        dm_mm256_store_ps(transform->pos_z + i, pos_z);
        
        dm_mm256_store_ps(transform->rot_i + i, rot_i);
        dm_mm256_store_ps(transform->rot_j + i, rot_j);
        dm_mm256_store_ps(transform->rot_k + i, rot_k);
        dm_mm256_store_ps(transform->rot_r + i, rot_r);
        
        dm_mm256_store_ps(physics->vel_x + i, vel_x);
        dm_mm256_store_ps(physics->vel_y + i, vel_y);
        dm_mm256_store_ps(physics->vel_z + i, vel_z);
        
        dm_mm256_store_ps(physics->w_x + i, w_x);
        dm_mm256_store_ps(physics->w_y + i, w_y);
        dm_mm256_store_ps(physics->w_z + i, w_z);
        
        dm_mm256_store_ps(physics->l_x + i, l_x);
        dm_mm256_store_ps(physics->l_y + i, l_y);
        dm_mm256_store_ps(physics->l_z + i, l_z);
        
        dm_mm256_store_ps(rigid_body->i_inv_00 + i, i_inv_00);
        dm_mm256_store_ps(rigid_body->i_inv_01 + i, i_inv_01);
        dm_mm256_store_ps(rigid_body->i_inv_02 + i, i_inv_02);
        
        dm_mm256_store_ps(rigid_body->i_inv_10 + i, i_inv_10);
        dm_mm256_store_ps(rigid_body->i_inv_11 + i, i_inv_11);
        dm_mm256_store_ps(rigid_body->i_inv_12 + i, i_inv_12);
        
        dm_mm256_store_ps(rigid_body->i_inv_20 + i, i_inv_20);
        dm_mm256_store_ps(rigid_body->i_inv_21 + i, i_inv_21);
        dm_mm256_store_ps(rigid_body->i_inv_22 + i, i_inv_22);
    }
    
    uint32_t leftovers = system->entity_count - i;
    if(leftovers>0 && leftovers<DM_SIMD256_FLOAT_N) 
    {
        pos_x = dm_mm256_set1_ps(0);
        pos_y = dm_mm256_set1_ps(0);
        pos_z = dm_mm256_set1_ps(0);
        
        rot_i = dm_mm256_set1_ps(0);
        rot_j = dm_mm256_set1_ps(0);
        rot_k = dm_mm256_set1_ps(0);
        rot_r = dm_mm256_set1_ps(0);
        
        vel_x = dm_mm256_set1_ps(0);
        vel_y = dm_mm256_set1_ps(0);
        vel_z = dm_mm256_set1_ps(0);
        
        w_x = dm_mm256_set1_ps(0);
        w_y = dm_mm256_set1_ps(0);
        w_z = dm_mm256_set1_ps(0);
        
        l_x = dm_mm256_set1_ps(0);
        l_y = dm_mm256_set1_ps(0);
        l_z = dm_mm256_set1_ps(0);
        
        force_x = dm_mm256_set1_ps(0);
        force_y = dm_mm256_set1_ps(0);
        force_z = dm_mm256_set1_ps(0);
        
        torque_x = dm_mm256_set1_ps(0);
        torque_y = dm_mm256_set1_ps(0);
        torque_z = dm_mm256_set1_ps(0);
        
        i_inv_00 = dm_mm256_set1_ps(0);
        i_inv_01 = dm_mm256_set1_ps(0);
        i_inv_02 = dm_mm256_set1_ps(0);
        
        i_inv_10 = dm_mm256_set1_ps(0);
        i_inv_11 = dm_mm256_set1_ps(0);
        i_inv_12 = dm_mm256_set1_ps(0);
        
        i_inv_20 = dm_mm256_set1_ps(0);
        i_inv_21 = dm_mm256_set1_ps(0);
        i_inv_22 = dm_mm256_set1_ps(0);
        
        pos_x = dm_mm256_load_ps(transform->pos_x + i);
        pos_y = dm_mm256_load_ps(transform->pos_y + i);
        pos_z = dm_mm256_load_ps(transform->pos_z + i);
        
        rot_i = dm_mm256_load_ps(transform->rot_i + i);
        rot_j = dm_mm256_load_ps(transform->rot_j + i);
        rot_k = dm_mm256_load_ps(transform->rot_k + i);
        rot_r = dm_mm256_load_ps(transform->rot_r + i);
        
        vel_x = dm_mm256_load_ps(physics->vel_x + i);
        vel_y = dm_mm256_load_ps(physics->vel_y + i);
        vel_z = dm_mm256_load_ps(physics->vel_z + i);
        
        w_x = dm_mm256_load_ps(physics->w_x + i);
        w_y = dm_mm256_load_ps(physics->w_y + i);
        w_z = dm_mm256_load_ps(physics->w_z + i);
        
        l_x = dm_mm256_load_ps(physics->l_x + i);
        l_y = dm_mm256_load_ps(physics->l_y + i);
        l_z = dm_mm256_load_ps(physics->l_z + i);
        
        force_x = dm_mm256_load_ps(physics->force_x + i);
        force_y = dm_mm256_load_ps(physics->force_y + i);
        force_z = dm_mm256_load_ps(physics->force_z + i);
        
        torque_x = dm_mm256_load_ps(physics->torque_x + i);
        torque_y = dm_mm256_load_ps(physics->torque_y + i);
        torque_z = dm_mm256_load_ps(physics->torque_z + i);
        
        i_inv_00 = dm_mm256_load_ps(rigid_body->i_inv_00 + i);
        i_inv_01 = dm_mm256_load_ps(rigid_body->i_inv_01 + i);
        i_inv_02 = dm_mm256_load_ps(rigid_body->i_inv_02 + i);
        
        i_inv_10 = dm_mm256_load_ps(rigid_body->i_inv_10 + i);
        i_inv_11 = dm_mm256_load_ps(rigid_body->i_inv_11 + i);
        i_inv_12 = dm_mm256_load_ps(rigid_body->i_inv_12 + i);
        
        i_inv_20 = dm_mm256_load_ps(rigid_body->i_inv_20 + i);
        i_inv_21 = dm_mm256_load_ps(rigid_body->i_inv_21 + i);
        i_inv_22 = dm_mm256_load_ps(rigid_body->i_inv_22 + i);
        
        dt_mass = dm_mm256_load_ps(physics->inv_mass + i);
        dt_mass = dm_mm256_mul_ps(dt_mass, dt);
        
        // integrate position
        pos_x = dm_mm256_fmadd_ps(vel_x, dt, pos_x);
        pos_y = dm_mm256_fmadd_ps(vel_y, dt, pos_y);
        pos_z = dm_mm256_fmadd_ps(vel_z, dt, pos_z);
        
        // integrate velocity
        vel_x = dm_mm256_fmadd_ps(force_x, dt_mass, vel_x);
        vel_y = dm_mm256_fmadd_ps(force_y, dt_mass, vel_y);
        vel_z = dm_mm256_fmadd_ps(force_z, dt_mass, vel_z);
        
        // integrate angular momentum
        l_x = dm_mm256_fmadd_ps(torque_x, dt, l_x);
        l_y = dm_mm256_fmadd_ps(torque_y, dt, l_y);
        l_z = dm_mm256_fmadd_ps(torque_z, dt, l_z);
        
        // integrate angular velocity
        w_x = dm_mm256_fmadd_ps(i_inv_00, l_x, w_x);
        w_x = dm_mm256_fmadd_ps(i_inv_01, l_y, w_x);
        w_x = dm_mm256_fmadd_ps(i_inv_02, l_z, w_x);
        
        w_y = dm_mm256_fmadd_ps(i_inv_10, l_x, w_y);
        w_y = dm_mm256_fmadd_ps(i_inv_11, l_y, w_y);
        w_y = dm_mm256_fmadd_ps(i_inv_12, l_z, w_y);
        
        w_z = dm_mm256_fmadd_ps(i_inv_20, l_x, w_z);
        w_z = dm_mm256_fmadd_ps(i_inv_21, l_y, w_z);
        w_z = dm_mm256_fmadd_ps(i_inv_22, l_z, w_z);
        
        // integrate rotation
        new_rot_i = dm_mm256_mul_ps(w_x, rot_r);
        new_rot_i = dm_mm256_fmadd_ps(w_x, rot_r, new_rot_i);
        new_rot_i = dm_mm256_fmadd_ps(w_y, rot_k, new_rot_i);
        new_rot_i = dm_mm256_sub_ps(new_rot_i, dm_mm256_mul_ps(w_z, rot_j));
        new_rot_i = dm_mm256_fmadd_ps(new_rot_i, half_dt, rot_i);
        
        new_rot_j = dm_mm256_sub_ps(zeroes, dm_mm256_mul_ps(w_x, rot_k));
        new_rot_j = dm_mm256_fmadd_ps(w_y, rot_r, new_rot_j);
        new_rot_j = dm_mm256_fmadd_ps(w_z, rot_i, new_rot_j);
        new_rot_j = dm_mm256_fmadd_ps(new_rot_i, half_dt, rot_j);
        
        new_rot_k = dm_mm256_mul_ps(w_x, rot_j);
        new_rot_k = dm_mm256_sub_ps(new_rot_k, dm_mm256_mul_ps(w_x, rot_i));
        new_rot_k = dm_mm256_fmadd_ps(w_z, rot_r, new_rot_k);
        new_rot_k = dm_mm256_fmadd_ps(new_rot_k, half_dt, rot_k);
        
        new_rot_r = dm_mm256_sub_ps(zeroes, dm_mm256_mul_ps(w_x, rot_i));
        new_rot_r = dm_mm256_sub_ps(new_rot_r, dm_mm256_mul_ps(w_y, rot_j));
        new_rot_r = dm_mm256_sub_ps(new_rot_r, dm_mm256_mul_ps(w_z, rot_k));
        new_rot_r = dm_mm256_fmadd_ps(new_rot_r, half_dt, rot_r);
        
        new_rot_mag = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_j, new_rot_j, new_rot_mag);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, new_rot_mag);
        new_rot_mag = dm_mm256_fmadd_ps(new_rot_r, new_rot_r, new_rot_mag);;
        new_rot_mag = dm_mm256_sqrt_ps(new_rot_mag);
        new_rot_mag = dm_mm256_div_ps(ones, new_rot_mag);
        
        new_rot_i = dm_mm256_mul_ps(new_rot_i, new_rot_mag);
        new_rot_j = dm_mm256_mul_ps(new_rot_j, new_rot_mag);
        new_rot_k = dm_mm256_mul_ps(new_rot_k, new_rot_mag);
        new_rot_r = dm_mm256_mul_ps(new_rot_r, new_rot_mag);
        
        // update i_inv
        orientation_00 = dm_mm256_mul_ps(new_rot_j, new_rot_j);
        orientation_00 = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, orientation_00);
        orientation_00 = dm_mm256_mul_ps(twos, orientation_00);
        orientation_00 = dm_mm256_sub_ps(ones, orientation_00);
        
        orientation_01 = dm_mm256_mul_ps(new_rot_i, new_rot_j);
        orientation_01 = dm_mm256_fmadd_ps(new_rot_k, new_rot_r, orientation_01);
        orientation_01 = dm_mm256_mul_ps(twos, orientation_01);
        
        orientation_02 = dm_mm256_mul_ps(new_rot_i, new_rot_k);
        orientation_02 = dm_mm256_sub_ps(orientation_02, dm_mm256_mul_ps(new_rot_j, new_rot_r));
        orientation_02 = dm_mm256_mul_ps(twos, orientation_02);
        
        orientation_10 = dm_mm256_mul_ps(new_rot_i, new_rot_j);
        orientation_10 = dm_mm256_sub_ps(orientation_10, dm_mm256_mul_ps(new_rot_k, new_rot_r));
        orientation_10 = dm_mm256_mul_ps(twos, orientation_10);
        
        orientation_11 = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        orientation_11 = dm_mm256_fmadd_ps(new_rot_k, new_rot_k, orientation_11);
        orientation_11 = dm_mm256_mul_ps(twos, orientation_11);
        orientation_11 = dm_mm256_sub_ps(ones, orientation_11);
        
        orientation_12 = dm_mm256_mul_ps(new_rot_j, new_rot_k);
        orientation_12 = dm_mm256_fmadd_ps(new_rot_i, new_rot_r, orientation_12);
        orientation_12 = dm_mm256_mul_ps(twos, orientation_12);
        
        orientation_20 = dm_mm256_mul_ps(new_rot_i, new_rot_k);
        orientation_20 = dm_mm256_fmadd_ps(new_rot_j, new_rot_r, orientation_20);
        orientation_20 = dm_mm256_mul_ps(twos, orientation_20);
        
        orientation_21 = dm_mm256_mul_ps(new_rot_j, new_rot_k);
        orientation_21 = dm_mm256_sub_ps(orientation_21, dm_mm256_mul_ps(new_rot_i, new_rot_r));
        orientation_21 = dm_mm256_mul_ps(twos, orientation_21);
        
        orientation_22 = dm_mm256_mul_ps(new_rot_i, new_rot_i);
        dm_mm256_fmadd_ps(new_rot_j, new_rot_j, orientation_22);
        dm_mm256_mul_ps(twos, orientation_22);
        dm_mm256_sub_ps(ones, orientation_22);
        
        // orientation is transposed here
        body_inv_00 = dm_mm256_mul_ps(orientation_00, i_inv_00);
        body_inv_01 = dm_mm256_mul_ps(orientation_10, i_inv_11);
        body_inv_02 = dm_mm256_mul_ps(orientation_20, i_inv_22);
        
        body_inv_10 = dm_mm256_mul_ps(orientation_01, i_inv_00);
        body_inv_11 = dm_mm256_mul_ps(orientation_11, i_inv_11);
        body_inv_12 = dm_mm256_mul_ps(orientation_21, i_inv_22);
        
        body_inv_20 = dm_mm256_mul_ps(orientation_02, i_inv_00);
        body_inv_21 = dm_mm256_mul_ps(orientation_12, i_inv_11);
        body_inv_22 = dm_mm256_mul_ps(orientation_22, i_inv_22);
        
        // final i_inv matrix
        i_inv_00 = dm_mm256_mul_ps(orientation_00, body_inv_00);
        i_inv_00 = dm_mm256_fmadd_ps(orientation_01, body_inv_10, i_inv_00);
        i_inv_00 = dm_mm256_fmadd_ps(orientation_02, body_inv_20, i_inv_00);
        
        i_inv_01 = dm_mm256_mul_ps(orientation_00, body_inv_01);
        i_inv_01 = dm_mm256_fmadd_ps(orientation_01, body_inv_11, i_inv_01);
        i_inv_01 = dm_mm256_fmadd_ps(orientation_02, body_inv_21, i_inv_01);;
        
        i_inv_02 = dm_mm256_mul_ps(orientation_00, body_inv_02);
        i_inv_02 = dm_mm256_fmadd_ps(orientation_01, body_inv_12, i_inv_02);
        i_inv_02 = dm_mm256_fmadd_ps(orientation_02, body_inv_22, i_inv_02);
        
        i_inv_10 = dm_mm256_mul_ps(orientation_10, body_inv_00);
        i_inv_10 = dm_mm256_fmadd_ps(orientation_11, body_inv_10, i_inv_10);
        i_inv_10 = dm_mm256_fmadd_ps(orientation_12, body_inv_20, i_inv_10);
        
        i_inv_11 = dm_mm256_mul_ps(orientation_10, body_inv_01);
        i_inv_11 = dm_mm256_fmadd_ps(orientation_11, body_inv_11, i_inv_11);
        i_inv_11 = dm_mm256_fmadd_ps(orientation_12, body_inv_21, i_inv_11);
        
        i_inv_12 = dm_mm256_mul_ps(orientation_10, body_inv_02);
        i_inv_12 = dm_mm256_fmadd_ps(orientation_11, body_inv_12, i_inv_12);
        i_inv_12 = dm_mm256_fmadd_ps(orientation_12, body_inv_22, i_inv_12);
        
        i_inv_20 = dm_mm256_mul_ps(orientation_20, body_inv_00);
        i_inv_20 = dm_mm256_fmadd_ps(orientation_21, body_inv_10, i_inv_20);
        i_inv_20 = dm_mm256_fmadd_ps(orientation_22, body_inv_20, i_inv_20);
        
        i_inv_21 = dm_mm256_mul_ps(orientation_20, body_inv_01);
        i_inv_21 = dm_mm256_fmadd_ps(orientation_21, body_inv_11, i_inv_21);
        i_inv_21 = dm_mm256_fmadd_ps(orientation_22, body_inv_21, i_inv_21);
        
        i_inv_22 = dm_mm256_mul_ps(orientation_20, body_inv_02);
        i_inv_22 = dm_mm256_fmadd_ps(orientation_21, body_inv_12, i_inv_22);
        i_inv_22 = dm_mm256_fmadd_ps(orientation_22, body_inv_22, i_inv_22);
        
        // store
        dm_mm256_store_ps(transform->pos_x + i, pos_x);
        dm_mm256_store_ps(transform->pos_y + i, pos_y);
        dm_mm256_store_ps(transform->pos_z + i, pos_z);
        
        dm_mm256_store_ps(transform->rot_i + i, rot_i);
        dm_mm256_store_ps(transform->rot_j + i, rot_j);
        dm_mm256_store_ps(transform->rot_k + i, rot_k);
        dm_mm256_store_ps(transform->rot_r + i, rot_r);
        
        dm_mm256_store_ps(physics->vel_x + i, vel_x);
        dm_mm256_store_ps(physics->vel_y + i, vel_y);
        dm_mm256_store_ps(physics->vel_z + i, vel_z);
        
        dm_mm256_store_ps(physics->w_x + i, w_x);
        dm_mm256_store_ps(physics->w_y + i, w_y);
        dm_mm256_store_ps(physics->w_z + i, w_z);
        
        dm_mm256_store_ps(physics->l_x + i, l_x);
        dm_mm256_store_ps(physics->l_y + i, l_y);
        dm_mm256_store_ps(physics->l_z + i, l_z);
        
        dm_mm256_store_ps(rigid_body->i_inv_00 + i, i_inv_00);
        dm_mm256_store_ps(rigid_body->i_inv_01 + i, i_inv_01);
        dm_mm256_store_ps(rigid_body->i_inv_02 + i, i_inv_02);
        
        dm_mm256_store_ps(rigid_body->i_inv_10 + i, i_inv_10);
        dm_mm256_store_ps(rigid_body->i_inv_11 + i, i_inv_11);
        dm_mm256_store_ps(rigid_body->i_inv_12 + i, i_inv_12);
        
        dm_mm256_store_ps(rigid_body->i_inv_20 + i, i_inv_20);
        dm_mm256_store_ps(rigid_body->i_inv_21 + i, i_inv_21);
        dm_mm256_store_ps(rigid_body->i_inv_22 + i, i_inv_22);
    }
}