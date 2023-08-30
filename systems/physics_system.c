#include "physics_system.h"

#include "app/components.h"

#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#include <limits.h>
#include <float.h>
#include <assert.h>

#define PHYSICS_SYSTEM_CONSTRAINT_ITER   10

typedef struct physics_system_collision_pair_t
{
    uint32_t entity_a, entity_b;
} physics_system_collision_pair;

typedef enum physics_system_flag_t
{
    DM_PHYSICS_FLAG_PAUSED = 1 << 0,
    DM_PHYSICS_FLAG_NO_COLLISIONS = 1 << 1,
} physics_system_flag;

#define PHYSICS_SYSTEM_DYNAMIC_CAPACITY      16
#define PHYSICS_SYSTEM_DYNAMIC_CAPACITY      16
#define PHYSICS_SYSTEM_DYNAMIC_LOAD_FACTOR   0.75f
#define PHYSICS_SYSTEM_DYNAMIC_RESIZE_FACTOR 2
typedef struct physics_system_manager_t
{
    double              accum_time, simulation_time;
    physics_system_flag flags;
    
    uint32_t broadphase_checks;
    
    uint32_t entity_count, entity_capacity;
    
    dm_ecs_id transform, collision, physics, rigid_body;
    
    uint32_t sweep_indices[DM_ECS_MAX_ENTITIES];
    uint32_t sphere_indices[DM_ECS_MAX_ENTITIES], box_indices[DM_ECS_MAX_ENTITIES];
    
    uint32_t num_possible_collisions, collision_capacity;
    uint32_t num_manifolds, manifold_capacity;
    
    physics_system_collision_pair* possible_collisions;
    dm_contact_manifold*           manifolds;
} physics_system_manager;

bool physics_system_broadphase(dm_ecs_system_manager* system, dm_context* context);
bool physics_system_narrowphase(dm_ecs_system_manager* system, dm_context* context);
void physics_system_solve_constraints(physics_system_manager* manager, dm_context* context);
void physics_system_update_entities(dm_ecs_system_manager* system, dm_context* context);
void physics_system_update_entities_simd(dm_ecs_system_manager* system, dm_context* context);

/************
SYSTEM FUNCS
**************/
bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_ecs_id r_id, dm_context* context)
{
    dm_ecs_id comps[] = { t_id, c_id, p_id, r_id };
    
    dm_ecs_system_timing timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    
    dm_ecs_id id;
    id = dm_ecs_register_system(comps, DM_ARRAY_LEN(comps), timing, physics_system_run, physics_system_shutdown, context);
    
    dm_ecs_system_manager* system = &context->ecs_manager.systems[timing][id];
    system->system_data = dm_alloc(sizeof(physics_system_manager));
    
    physics_system_manager* manager = system->system_data;
    
    manager->collision_capacity  = PHYSICS_SYSTEM_DYNAMIC_CAPACITY;
    manager->manifold_capacity   = PHYSICS_SYSTEM_DYNAMIC_CAPACITY;
    manager->possible_collisions = dm_alloc(sizeof(physics_system_collision_pair) * manager->collision_capacity);
    manager->manifolds           = dm_alloc(sizeof(dm_contact_manifold) * manager->manifold_capacity);
    
    manager->transform  = t_id;
    manager->collision  = c_id;
    manager->physics    = p_id;
    manager->rigid_body = r_id;
    
    return true;
}

void physics_system_shutdown(void* s, void* c)
{
    dm_ecs_system_manager* system = s;   
    physics_system_manager* manager = system->system_data;
    
    dm_free(manager->possible_collisions);
    dm_free(manager->manifolds);
}

bool physics_system_run(void* s, void* d)
{
    dm_context* context = d;
    dm_ecs_system_manager* system = s;
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
            //if(!physics_system_broadphase(system, context)) return false;
            broad_time += dm_timer_elapsed_ms(&t, context);
            
            // narrowphase
            dm_timer_start(&t, context);
            if(!physics_system_narrowphase(system, context)) return false;
            narrow_time += dm_timer_elapsed_ms(&t, context);
            
            // collision resolution
            dm_timer_start(&t, context);
            physics_system_solve_constraints(manager, context);
            collision_time += dm_timer_elapsed_ms(&t, context);
        }
        
        // update
        dm_timer_start(&t, context);
        //physics_system_update_entities(system, context);
        physics_system_update_entities_simd(system, context);
        update_time += dm_timer_elapsed_ms(&t, context);
        
        manager->accum_time -= DM_PHYSICS_FIXED_DT;
    }
    
    // reset forces
    component_physics* physics = dm_ecs_get_component_block(manager->physics, context);
    dm_memzero(physics->force_x,  sizeof(float) * DM_ECS_MAX_ENTITIES);
    dm_memzero(physics->force_y,  sizeof(float) * DM_ECS_MAX_ENTITIES);
    dm_memzero(physics->force_z,  sizeof(float) * DM_ECS_MAX_ENTITIES);
    dm_memzero(physics->torque_x, sizeof(float) * DM_ECS_MAX_ENTITIES);
    dm_memzero(physics->torque_y, sizeof(float) * DM_ECS_MAX_ENTITIES);
    dm_memzero(physics->torque_z, sizeof(float) * DM_ECS_MAX_ENTITIES);
    
    total_time = dm_timer_elapsed_ms(&full, context);
    
    float iter_f_inv = 1 / (float)iters;
    
    imgui_draw_text_fmt(20,20, 1,1,0,1, context, "Physics broadphase average: %0.3lf ms (%u checks)", broad_time * iter_f_inv, manager->broadphase_checks);
    imgui_draw_text_fmt(20,40, 1,1,0,1, context, "Physics narrowphase average: %0.3lf ms (%u checks)", narrow_time * iter_f_inv, manager->num_possible_collisions);
    imgui_draw_text_fmt(20,60, 1,1,0,1, context, "Physics collision resolution average: %0.3lf ms (%u manifolds)", collision_time * iter_f_inv, manager->num_manifolds);
    imgui_draw_text_fmt(20,80, 1,1,0,1, context, "Updating entities average: %0.3lf ms", update_time * iter_f_inv);
    imgui_draw_text_fmt(20,100, 1,0,1,1, context, "Physics took: %0.3lf ms, %u iterations", total_time, iters);
    
    return true;
}

/**********
BROADPHASE
uses a simple sort and sweep on the highest variance axis
************/
#ifdef DM_PLATFORM_LINUX
int physics_system_broadphase_sort(const void* a, const void* b, void* c)
#else
int physics_system_broadphase_sort(void* c, const void* a, const void* b)
#endif
{
    float* min = c;
    
    uint32_t entity_a = *(uint32_t*)a;
    uint32_t entity_b = *(uint32_t*)b;
    
    float a_min = min[entity_a];
    float b_min = min[entity_b];
    
    return (a_min > b_min) - (a_min < b_min);
}

void physics_system_broadphase_update_sphere_aabbs(uint32_t sphere_count, float center_sum[3], float center_sq_sum[3], dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    
    uint32_t t_index, c_index;
    
    float center[3] = { 0 };
    
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    
    for(uint32_t i=0; i<sphere_count; i++)
    {
        t_index = system->entity_indices[manager->sphere_indices[i]][t_id];
        c_index = system->entity_indices[manager->sphere_indices[i]][c_id];
        
        collision->aabb_global_min_x[c_index] = collision->aabb_local_min_x[c_index] + transform->pos_x[t_index];
        collision->aabb_global_min_y[c_index] = collision->aabb_local_min_y[c_index] + transform->pos_y[t_index];
        collision->aabb_global_min_z[c_index] = collision->aabb_local_min_z[c_index] + transform->pos_z[t_index];
        
        collision->aabb_global_max_x[c_index] = collision->aabb_local_max_x[c_index] + transform->pos_x[t_index];
        collision->aabb_global_max_y[c_index] = collision->aabb_local_max_y[c_index] + transform->pos_y[t_index];
        collision->aabb_global_max_z[c_index] = collision->aabb_local_max_z[c_index] + transform->pos_z[t_index];
        
        center[0] = 0.5f * (collision->aabb_global_max_x[c_index] - collision->aabb_global_min_x[c_index]);
        center[1] = 0.5f * (collision->aabb_global_max_y[c_index] - collision->aabb_global_min_y[c_index]);
        center[2] = 0.5f * (collision->aabb_global_max_z[c_index] - collision->aabb_global_min_z[c_index]);
        
        center_sum[0] += center[0];
        center_sum[1] += center[1];
        center_sum[2] += center[2];
        
        center[0] = center[0] * center[0];
        center[1] = center[1] * center[1];
        center[2] = center[2] * center[2];
        
        center_sq_sum[0] += center[0];
        center_sq_sum[1] += center[1];
        center_sq_sum[2] += center[2];
    }
}

void physics_system_broadphase_update_box_aabbs(uint32_t box_count, float center_sum[3], float center_sq_sum[3], dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    
    uint32_t t_index, c_index;
    
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    
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
        t_index = system->entity_indices[manager->sphere_indices[i]][t_id];
        c_index = system->entity_indices[manager->sphere_indices[i]][c_id];
        
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
        
        // start
        world_min[0] = world_max[0] = transform->pos_x[t_index];
        world_min[1] = world_max[1] = transform->pos_y[t_index];
        world_min[2] = world_max[2] = transform->pos_z[t_index];
        
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
        
        // centers
        center[0] = 0.5f * (world_max[0] - world_min[0]);
        center[1] = 0.5f * (world_max[1] - world_min[1]);
        center[2] = 0.5f * (world_max[2] - world_min[2]);
        
        center_sum[0] += center[0];
        center_sum[1] += center[1];
        center_sum[2] += center[2];
        
        center_sq_sum[0] += center[0] * center[0];
        center_sq_sum[1] += center[1] * center[1];
        center_sq_sum[2] += center[2] * center[2];
        
        // assign
        collision->aabb_global_min_x[c_index] = world_min[0];
        collision->aabb_global_min_y[c_index] = world_min[1];
        collision->aabb_global_min_z[c_index] = world_min[2];
        
        collision->aabb_global_max_x[c_index] = world_max[0];
        collision->aabb_global_max_y[c_index] = world_max[1];
        collision->aabb_global_max_z[c_index] = world_max[2];
    }
}

void physics_system_broadphase_update_box_aabbs_simd(uint32_t box_count, float center_sum[3], float center_sq_sum[3], dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    
    dm_mm_float center_x, center_y, center_z;
    dm_mm_float center_sum_x, center_sum_y, center_sum_z;
    dm_mm_float center_sq_sum_x, center_sq_sum_y, center_sq_sum_z;
    
    dm_mm_float a,b;
    dm_mm_float rot_i, rot_j, rot_k, rot_r;
    
    dm_mm_float xx,yy,zz;
    dm_mm_float xy,xz,xw,yz,yw,zw;
    
    dm_mm_float rot_00, rot_01, rot_02;
    dm_mm_float rot_10, rot_11, rot_12;
    dm_mm_float rot_20, rot_21, rot_22;
    
    dm_mm_float min_x, min_y, min_z;
    dm_mm_float max_x, max_y, max_z;
    dm_mm_float world_min_x, world_min_y, world_min_z;
    dm_mm_float world_max_x, world_max_y, world_max_z;
    
    dm_mm_float ones = dm_mm_set1_ps(1);
    dm_mm_float twos = dm_mm_set1_ps(2);
    dm_mm_float half = dm_mm_set1_ps(0.5f);
    
    center_sum_x = dm_mm_set1_ps(0);
    center_sum_y = dm_mm_set1_ps(0);
    center_sum_z = dm_mm_set1_ps(0);
    
    center_sq_sum_x = dm_mm_set1_ps(0);
    center_sq_sum_y = dm_mm_set1_ps(0);
    center_sq_sum_z = dm_mm_set1_ps(0);
    
    float pos_x[DM_ECS_MAX_ENTITIES], pos_y[DM_ECS_MAX_ENTITIES], pos_z[DM_ECS_MAX_ENTITIES];
    float r_i[DM_ECS_MAX_ENTITIES], r_j[DM_ECS_MAX_ENTITIES], r_k[DM_ECS_MAX_ENTITIES], r_r[DM_ECS_MAX_ENTITIES];
    float aabb_min_x[DM_ECS_MAX_ENTITIES], aabb_min_y[DM_ECS_MAX_ENTITIES], aabb_min_z[DM_ECS_MAX_ENTITIES];
    float aabb_max_x[DM_ECS_MAX_ENTITIES], aabb_max_y[DM_ECS_MAX_ENTITIES], aabb_max_z[DM_ECS_MAX_ENTITIES];
    
    uint32_t i = 0;
    uint32_t t_index, c_index;
    
    for(; i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        c_index = system->entity_indices[i][c_id];
        
        pos_x[i] = transform->pos_x[t_index];
        pos_y[i] = transform->pos_y[t_index];
        pos_z[i] = transform->pos_z[t_index];
        
        r_i[i] = transform->rot_i[t_index];
        r_j[i] = transform->rot_j[t_index];
        r_k[i] = transform->rot_k[t_index];
        r_r[i] = transform->rot_r[t_index];
        
        aabb_min_x[i] = collision->aabb_local_min_x[c_index];
        aabb_min_y[i] = collision->aabb_local_min_y[c_index];
        aabb_min_z[i] = collision->aabb_local_min_z[c_index];
        
        aabb_max_x[i] = collision->aabb_local_max_x[c_index];
        aabb_max_y[i] = collision->aabb_local_max_y[c_index];
        aabb_max_z[i] = collision->aabb_local_max_z[c_index];
    }
    
    i=0;
    for(; (box_count-i)>=DM_SIMD_N; i+=DM_SIMD_N)
    {
        world_min_x = dm_mm_load_ps(pos_x + i);
        world_min_y = dm_mm_load_ps(pos_y + i);
        world_min_z = dm_mm_load_ps(pos_z + i);
        
        world_max_x = dm_mm_load_ps(pos_x + i);
        world_max_y = dm_mm_load_ps(pos_y + i);
        world_max_z = dm_mm_load_ps(pos_z + i);
        
        rot_i = dm_mm_load_ps(r_i + i);
        rot_j = dm_mm_load_ps(r_j + i);
        rot_k = dm_mm_load_ps(r_k + i);
        rot_r = dm_mm_load_ps(r_r + i);
        
        min_x = dm_mm_load_ps(aabb_min_x + i);
        min_y = dm_mm_load_ps(aabb_min_z + i);
        min_z = dm_mm_load_ps(aabb_min_z + i);
        
        max_x = dm_mm_load_ps(aabb_max_x + i);
        max_y = dm_mm_load_ps(aabb_max_y + i);
        max_z = dm_mm_load_ps(aabb_max_z + i);
        
        xx = dm_mm_mul_ps(rot_i, rot_i);
        yy = dm_mm_mul_ps(rot_j, rot_j);
        zz = dm_mm_mul_ps(rot_k, rot_k);
        xy = dm_mm_mul_ps(rot_i, rot_j);
        xz = dm_mm_mul_ps(rot_i, rot_k);
        xw = dm_mm_mul_ps(rot_i, rot_r);
        yz = dm_mm_mul_ps(rot_j, rot_k);
        yw = dm_mm_mul_ps(rot_j, rot_r);
        zw = dm_mm_mul_ps(rot_k, rot_r);
        
        rot_00 = dm_mm_add_ps(yy,zz);
        rot_00 = dm_mm_mul_ps(rot_00, twos);
        rot_00 = dm_mm_sub_ps(ones, rot_00);
        rot_01 = dm_mm_add_ps(xy,zw);
        rot_01 = dm_mm_mul_ps(rot_01, twos);
        rot_02 = dm_mm_sub_ps(xz,yw);
        rot_02 = dm_mm_mul_ps(rot_02, twos);
        
        rot_10 = dm_mm_sub_ps(xy,zw);
        rot_10 = dm_mm_mul_ps(rot_10, twos);
        rot_11 = dm_mm_add_ps(xx,zz);
        rot_11 = dm_mm_mul_ps(rot_11, twos);
        rot_11 = dm_mm_sub_ps(ones, rot_11);
        rot_12 = dm_mm_add_ps(yz,xw);
        rot_12 = dm_mm_mul_ps(rot_12, twos);
        
        rot_20 = dm_mm_add_ps(xz,yw);
        rot_20 = dm_mm_mul_ps(rot_20, twos);
        rot_21 = dm_mm_sub_ps(yz,xw);
        rot_21 = dm_mm_mul_ps(rot_21, twos);
        rot_22 = dm_mm_add_ps(xx,yy);
        rot_22 = dm_mm_mul_ps(rot_22, twos);
        rot_22 = dm_mm_sub_ps(ones, rot_22);
        
        // x
        a = dm_mm_mul_ps(rot_00, min_x);
        b = dm_mm_mul_ps(rot_00, max_x);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_10, min_y);
        b = dm_mm_mul_ps(rot_10, max_y);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_20, min_z);
        b = dm_mm_mul_ps(rot_20, max_z);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        // y
        a = dm_mm_mul_ps(rot_01, min_x);
        b = dm_mm_mul_ps(rot_01, max_x);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_11, min_y);
        b = dm_mm_mul_ps(rot_11, max_y);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_21, min_z);
        b = dm_mm_mul_ps(rot_21, max_z);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        // z
        a = dm_mm_mul_ps(rot_02, min_x);
        b = dm_mm_mul_ps(rot_02, max_x);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_12, min_y);
        b = dm_mm_mul_ps(rot_12, max_y);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_22, min_z);
        b = dm_mm_mul_ps(rot_22, max_z);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        // centers
        center_x = dm_mm_sub_ps(world_max_x, world_min_x);
        center_x = dm_mm_mul_ps(center_x, half);
        center_y = dm_mm_sub_ps(world_max_y, world_min_y);
        center_y = dm_mm_mul_ps(center_y, half);
        center_z = dm_mm_sub_ps(world_max_z, world_min_z);
        center_z = dm_mm_mul_ps(center_z, half);
        
        center_sum_x = dm_mm_add_ps(center_sum_x, center_x);
        center_sum_y = dm_mm_add_ps(center_sum_y, center_y);
        center_sum_z = dm_mm_add_ps(center_sum_z, center_z);
        
        center_sq_sum_x = dm_mm_fmadd_ps(center_x,center_x, center_sq_sum_x);
        center_sq_sum_y = dm_mm_fmadd_ps(center_y,center_y, center_sq_sum_y);
        center_sq_sum_z = dm_mm_fmadd_ps(center_z,center_z, center_sq_sum_z);
        
        // assign
        dm_mm_store_ps(aabb_min_x + i, world_min_x);
        dm_mm_store_ps(aabb_min_y + i, world_min_y);
        dm_mm_store_ps(aabb_min_z + i, world_min_z);
        
        dm_mm_store_ps(aabb_max_x + i, world_max_x);
        dm_mm_store_ps(aabb_max_y + i, world_max_y);
        dm_mm_store_ps(aabb_max_z + i, world_max_z);
    }
    
    // might have one more pass
    uint32_t leftovers = box_count - i;
    if(leftovers)
    {
        world_min_x = dm_mm_set1_ps(0);
        world_min_y = dm_mm_set1_ps(0);
        world_min_z = dm_mm_set1_ps(0);
        
        world_max_x = dm_mm_set1_ps(0);
        world_max_y = dm_mm_set1_ps(0);
        world_max_z = dm_mm_set1_ps(0);
        
        rot_i = dm_mm_set1_ps(0);
        rot_j = dm_mm_set1_ps(0);
        rot_k = dm_mm_set1_ps(0);
        rot_r = dm_mm_set1_ps(0);
        
        min_x = dm_mm_set1_ps(0);
        min_y = dm_mm_set1_ps(0);
        min_z = dm_mm_set1_ps(0);
        
        max_x = dm_mm_set1_ps(0);
        max_y = dm_mm_set1_ps(0);
        max_z = dm_mm_set1_ps(0);
        
        world_min_x = dm_mm_load_ps(pos_x + i);
        world_min_y = dm_mm_load_ps(pos_y + i);
        world_min_z = dm_mm_load_ps(pos_z + i);
        
        world_max_x = dm_mm_load_ps(pos_x + i);
        world_max_y = dm_mm_load_ps(pos_y + i);
        world_max_z = dm_mm_load_ps(pos_z + i);
        
        min_x = dm_mm_load_ps(aabb_min_x + i);
        min_y = dm_mm_load_ps(aabb_min_y + i);
        min_z = dm_mm_load_ps(aabb_min_z + i);
        
        max_x = dm_mm_load_ps(aabb_max_x + i);
        max_y = dm_mm_load_ps(aabb_max_y + i);
        max_z = dm_mm_load_ps(aabb_max_z + i);
        
        rot_i = dm_mm_load_ps(r_i + i);
        rot_j = dm_mm_load_ps(r_j + i);
        rot_k = dm_mm_load_ps(r_k + i);
        rot_r = dm_mm_load_ps(r_r + i);
        
        xx = dm_mm_mul_ps(rot_i, rot_i);
        yy = dm_mm_mul_ps(rot_j, rot_j);
        zz = dm_mm_mul_ps(rot_k, rot_k);
        xy = dm_mm_mul_ps(rot_i, rot_j);
        xz = dm_mm_mul_ps(rot_i, rot_k);
        xw = dm_mm_mul_ps(rot_i, rot_r);
        yz = dm_mm_mul_ps(rot_j, rot_k);
        yw = dm_mm_mul_ps(rot_j, rot_r);
        zw = dm_mm_mul_ps(rot_k, rot_r);
        
        rot_00 = dm_mm_add_ps(yy,zz);
        rot_00 = dm_mm_mul_ps(rot_00, twos);
        rot_00 = dm_mm_sub_ps(ones, rot_00);
        rot_01 = dm_mm_add_ps(xy,zw);
        rot_01 = dm_mm_mul_ps(rot_01, twos);
        rot_02 = dm_mm_sub_ps(xz,yw);
        rot_02 = dm_mm_mul_ps(rot_02, twos);
        
        rot_10 = dm_mm_sub_ps(xy,zw);
        rot_10 = dm_mm_mul_ps(rot_10, twos);
        rot_11 = dm_mm_add_ps(xx,zz);
        rot_11 = dm_mm_mul_ps(rot_11, twos);
        rot_11 = dm_mm_sub_ps(ones, rot_11);
        rot_12 = dm_mm_add_ps(yz,xw);
        rot_12 = dm_mm_mul_ps(rot_12, twos);
        
        rot_20 = dm_mm_add_ps(xz,yw);
        rot_20 = dm_mm_mul_ps(rot_20, twos);
        rot_21 = dm_mm_sub_ps(yz,xw);
        rot_21 = dm_mm_mul_ps(rot_21, twos);
        rot_22 = dm_mm_add_ps(xx,yy);
        rot_22 = dm_mm_mul_ps(rot_22, twos);
        rot_22 = dm_mm_sub_ps(ones, rot_22);
        
        // x
        a = dm_mm_mul_ps(rot_00, min_x);
        b = dm_mm_mul_ps(rot_00, max_x);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_10, min_y);
        b = dm_mm_mul_ps(rot_10, max_y);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_20, min_z);
        b = dm_mm_mul_ps(rot_20, max_z);
        world_min_x = dm_mm_add_ps(world_min_x, dm_mm_min_ps(a,b));
        world_max_x = dm_mm_add_ps(world_max_x, dm_mm_max_ps(a,b));
        
        // y
        a = dm_mm_mul_ps(rot_01, min_x);
        b = dm_mm_mul_ps(rot_01, max_x);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_11, min_y);
        b = dm_mm_mul_ps(rot_11, max_y);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_21, min_z);
        b = dm_mm_mul_ps(rot_21, max_z);
        world_min_y = dm_mm_add_ps(world_min_y, dm_mm_min_ps(a,b));
        world_max_y = dm_mm_add_ps(world_max_y, dm_mm_max_ps(a,b));
        
        // z
        a = dm_mm_mul_ps(rot_02, min_x);
        b = dm_mm_mul_ps(rot_02, max_x);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_12, min_x);
        b = dm_mm_mul_ps(rot_12, max_x);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        a = dm_mm_mul_ps(rot_22, min_x);
        b = dm_mm_mul_ps(rot_22, max_x);
        world_min_z = dm_mm_add_ps(world_min_z, dm_mm_min_ps(a,b));
        world_max_z = dm_mm_add_ps(world_max_z, dm_mm_max_ps(a,b));
        
        // centers
        center_x = dm_mm_sub_ps(world_max_x, world_min_x);
        center_x = dm_mm_mul_ps(center_x, half);
        center_y = dm_mm_sub_ps(world_max_y, world_min_y);
        center_y = dm_mm_mul_ps(center_y, half);
        center_z = dm_mm_sub_ps(world_max_z, world_min_z);
        center_z = dm_mm_mul_ps(center_z, half);
        
        center_sum_x = dm_mm_add_ps(center_sum_x, center_x);
        center_sum_y = dm_mm_add_ps(center_sum_y, center_y);
        center_sum_z = dm_mm_add_ps(center_sum_z, center_z);
        
        center_sq_sum_x = dm_mm_fmadd_ps(center_x,center_x, center_sq_sum_x);
        center_sq_sum_y = dm_mm_fmadd_ps(center_y,center_y, center_sq_sum_y);
        center_sq_sum_z = dm_mm_fmadd_ps(center_z,center_z, center_sq_sum_z);
        
        // assign
        dm_mm_store_ps(aabb_min_x + i, world_min_x);
        dm_mm_store_ps(aabb_min_y + i, world_min_y);
        dm_mm_store_ps(aabb_min_z + i, world_min_z);
        
        dm_mm_store_ps(aabb_max_x + i, world_max_x);
        dm_mm_store_ps(aabb_max_y + i, world_max_y);
        dm_mm_store_ps(aabb_max_z + i, world_max_z);
    }
    
    i=0;
    for(;i<system->entity_count; i++)
    {
        c_index = system->entity_indices[i][c_id];
        
        collision->aabb_global_min_x[c_index] = aabb_min_x[i];
        collision->aabb_global_min_y[c_index] = aabb_min_y[i];
        collision->aabb_global_min_z[c_index] = aabb_min_z[i];
        
        collision->aabb_global_max_x[c_index] = aabb_max_x[i];
        collision->aabb_global_max_y[c_index] = aabb_max_y[i];
        collision->aabb_global_max_z[c_index] = aabb_max_z[i];
    }
    
    // add to center_sum and center_sq_sum
    center_sum[0] += dm_mm_sum_elements(center_sum_x);
    center_sum[1] += dm_mm_sum_elements(center_sum_y);
    center_sum[2] += dm_mm_sum_elements(center_sum_z);
    
    center_sq_sum[0] += dm_mm_sum_elements(center_sq_sum_x);
    center_sq_sum[1] += dm_mm_sum_elements(center_sq_sum_y);
    center_sq_sum[2] += dm_mm_sum_elements(center_sq_sum_z);
}

int physics_system_broadphase_get_variance_axis(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    float center_sum[3]    = { 0 };
    float center_sq_sum[3] = { 0 };
    
    int axis = 0;
    
    uint32_t sphere_count=0, box_count=0;
    
    const dm_ecs_id c_id = manager->collision;
    
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        manager->sweep_indices[i] = i;
        switch(collision->shape[system->entity_indices[i][c_id]])
        {
            case DM_COLLISION_SHAPE_SPHERE: manager->sphere_indices[sphere_count++] = i;
            break;
            
            case DM_COLLISION_SHAPE_BOX: manager->box_indices[box_count++] = i;
            break;
            
            default:
            break;
        }
    }
    
    if(sphere_count) physics_system_broadphase_update_sphere_aabbs(sphere_count, center_sum, center_sq_sum, system, context);
    //if(box_count) physics_system_broadphase_update_box_aabbs(box_count, manager, center_sum, center_sq_sum);
    if(box_count) physics_system_broadphase_update_box_aabbs_simd(box_count, center_sum, center_sq_sum, system, context);
    
    const float scale = 1.0f / system->entity_count;
    
    float var[3];
    float max_var;
    
    dm_vec3_scale(center_sum, scale, center_sum);
    dm_vec3_scale(center_sq_sum, scale, center_sq_sum);
    
    dm_vec3_mul_vec3(center_sum, center_sum, center_sum);
    var[0] = center_sq_sum[0] - center_sum[0];
    var[1] = center_sq_sum[1] - center_sum[1];
    var[2] = center_sq_sum[2] - center_sum[2];
    
    max_var = -FLT_MAX;
    for(uint32_t i=0; i<N3; i++)
    {
        if(var[i] <= max_var) continue;
        
        max_var = var[i];
        axis = i;
    }
    
    return axis;
}

void physics_system_broadphase_sort_sweep(uint32_t count, float* sort_min, float* max_i_array, dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    manager->broadphase_checks = 0;
    manager->num_possible_collisions = 0;
    
    const dm_ecs_id c_id = manager->collision;
    
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    
#ifdef DM_PLATFORM_WIN32
    qsort_s(manager->sweep_indices, count, sizeof(uint32_t), physics_system_broadphase_sort, sort_min);
#elif defined(DM_PLATFORM_LINUX)
    qsort_r(manager->sweep_indices, count, sizeof(uint32_t), physics_system_broadphase_sort, sort_min);
#elif defined(DM_PLATFORM_APPLE)
    qsort_r(manager->sweep_indices, count, sizeof(uint32_t), sort_min, physics_system_broadphase_sort);
#endif
    
    // sweep
    float max_i, min_j;
    
    bool x_check, y_check, z_check;
    
    float a_pos[3], a_dim[3];
    float b_pos[3], b_dim[3];
    
    uint32_t entity_a, entity_b;
    
    float x_sep, y_sep, z_sep;
    float x_dim, y_dim, z_dim;
    
    uint32_t a_index, b_index;
    
    for(uint32_t i=0; i<count; i++)
    {
        entity_a = manager->sweep_indices[i];
        
        a_index = system->entity_indices[entity_a][c_id];
        
        a_dim[0] = collision->aabb_global_max_x[a_index] - collision->aabb_global_min_x[a_index];
        a_dim[1] = collision->aabb_global_max_y[a_index] - collision->aabb_global_min_y[a_index];
        a_dim[2] = collision->aabb_global_max_z[a_index] - collision->aabb_global_min_z[a_index];
        
        a_dim[0] = a_dim[0] < 0 ? -a_dim[0] : a_dim[0];
        a_dim[1] = a_dim[1] < 0 ? -a_dim[1] : a_dim[1];
        a_dim[2] = a_dim[2] < 0 ? -a_dim[2] : a_dim[2];
        
        a_dim[0] *= 0.5f;
        a_dim[1] *= 0.5f;
        a_dim[2] *= 0.5f;
        
        a_pos[0] = collision->aabb_global_max_x[a_index] - a_dim[0];
        a_pos[1] = collision->aabb_global_max_y[a_index] - a_dim[1];
        a_pos[2] = collision->aabb_global_max_z[a_index] - a_dim[2];
        
        max_i = max_i_array[entity_a];
        
        for(uint32_t j=i+1; j<count; j++)
        {
            entity_b = manager->sweep_indices[j];
            
            b_index = system->entity_indices[entity_b][c_id];
            
            b_dim[0] = collision->aabb_global_max_x[b_index] - collision->aabb_global_min_x[b_index];
            b_dim[1] = collision->aabb_global_max_y[b_index] - collision->aabb_global_min_y[b_index];
            b_dim[2] = collision->aabb_global_max_z[b_index] - collision->aabb_global_min_z[b_index];
            
            b_dim[0] = b_dim[0] < 0 ? -b_dim[0] : b_dim[0];
            b_dim[1] = b_dim[1] < 0 ? -b_dim[1] : b_dim[1];
            b_dim[2] = b_dim[2] < 0 ? -b_dim[2] : b_dim[2];
            
            b_dim[0] *= 0.5f;
            b_dim[1] *= 0.5f;
            b_dim[2] *= 0.5f;
            
            b_pos[0] = collision->aabb_global_max_x[b_index] - b_dim[0];
            b_pos[1] = collision->aabb_global_max_y[b_index] - b_dim[1];
            b_pos[2] = collision->aabb_global_max_z[b_index] - b_dim[2];
            
            min_j = sort_min[entity_b];
            
            if(min_j > max_i) break;
            
            manager->broadphase_checks++;
            
            x_sep = a_pos[0] - b_pos[0];
            y_sep = a_pos[1] - b_pos[1];
            z_sep = a_pos[2] - b_pos[2];
            
            x_sep = x_sep < 0 ? -x_sep : x_sep;
            y_sep = y_sep < 0 ? -y_sep : y_sep;
            z_sep = z_sep < 0 ? -z_sep : z_sep;
            
            x_dim = a_dim[0] + b_dim[0];
            y_dim = a_dim[1] + b_dim[1];
            z_dim = a_dim[2] + b_dim[2];
            
            x_check = x_sep <= x_dim;
            y_check = y_sep <= y_dim;
            z_check = z_sep <= z_dim;
            
            if(!x_check || !y_check || !z_check) continue;
            
            collision->flag[a_index] = COLLISION_FLAG_POSSIBLE;
            collision->flag[b_index] = COLLISION_FLAG_POSSIBLE;
            
            manager->possible_collisions[manager->num_possible_collisions].entity_a = entity_a;
            manager->possible_collisions[manager->num_possible_collisions].entity_b = entity_b;
            manager->num_possible_collisions++;
            
            dm_grow_dyn_array((void**)&manager->possible_collisions, manager->num_possible_collisions, &manager->collision_capacity, sizeof(physics_system_collision_pair), PHYSICS_SYSTEM_DYNAMIC_LOAD_FACTOR, PHYSICS_SYSTEM_DYNAMIC_RESIZE_FACTOR);
        }
    }
    
}

// uses simple sort and sweep based on objects' aabbs
bool physics_system_broadphase(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const int axis = physics_system_broadphase_get_variance_axis(system, context);
    
    component_collision* collision = dm_ecs_get_component_block(manager->collision, context);
    
    float* min, *max;
    
    // sort
    switch(axis)
    {
        case 0:
        min = collision->aabb_global_min_x;
        max = collision->aabb_global_max_x;
        break;
        
        case 1: 
        min = collision->aabb_global_min_y;
        max = collision->aabb_global_max_y;
        break;
        
        case 2: 
        min = collision->aabb_global_min_z;
        max = collision->aabb_global_max_z;
        break;
    }
    
    physics_system_broadphase_sort_sweep(system->entity_count, min, max, system, context);
    
    return true;
}

/***********
NARROWPHASE
uses GJK to determine collisions
then EPA to determine penetration vector
then generates contact manifolds
*************/
bool physics_system_narrowphase(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    manager->num_manifolds = 0;
    
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
    
    component_transform*  transform = dm_ecs_get_component_block(t_id, context);
    component_collision*  collision = dm_ecs_get_component_block(c_id, context);
    component_physics*    physics   = dm_ecs_get_component_block(p_id, context);
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    
    uint32_t t_a, c_a, p_a, r_a;
    uint32_t t_b, c_b, p_b, r_b;
    
    for(uint32_t i=0; i<manager->num_possible_collisions; i++)
    {
        collision_pair = manager->possible_collisions[i];
        
        entity_a = collision_pair.entity_a;
        entity_b = collision_pair.entity_b;
        
        t_a = system->entity_indices[entity_a][t_id];
        c_a = system->entity_indices[entity_a][c_id];
        p_a = system->entity_indices[entity_a][p_id];
        r_a = system->entity_indices[entity_a][r_id];
        t_b = system->entity_indices[entity_b][t_id];
        c_b = system->entity_indices[entity_b][c_id];
        p_b = system->entity_indices[entity_b][p_id];
        r_b = system->entity_indices[entity_b][r_id];
        
        // get all data
        
        // entity a
        pos[0][0]       = transform->pos_x[t_a];
        pos[0][1]       = transform->pos_y[t_a];
        pos[0][2]       = transform->pos_z[t_a];
        rots[0][0]      = transform->rot_i[t_a];
        rots[0][1]      = transform->rot_j[t_a];
        rots[0][2]      = transform->rot_k[t_a];
        rots[0][3]      = transform->rot_r[t_a];
        cens[0][0]      = collision->center_x[c_a];
        cens[0][1]      = collision->center_y[c_a];
        cens[0][2]      = collision->center_z[c_a];
        internals[0][0] = collision->internal_0[c_a];
        internals[0][1] = collision->internal_1[c_a];
        internals[0][2] = collision->internal_2[c_a];
        internals[0][3] = collision->internal_3[c_a];
        internals[0][4] = collision->internal_4[c_a];
        internals[0][5] = collision->internal_5[c_a];
        shapes[0]       = collision->shape[c_a];
        vels[0][0]      = physics->vel_x[p_a];
        vels[0][1]      = physics->vel_y[p_a];
        vels[0][2]      = physics->vel_z[p_a];
        ws[0][0]        = physics->w_x[p_a];
        ws[0][1]        = physics->w_y[p_a];
        ws[0][2]        = physics->w_z[p_a];
        
        // entity b
        pos[1][0]       = transform->pos_x[t_b];
        pos[1][1]       = transform->pos_y[t_b];
        pos[1][2]       = transform->pos_z[t_b];
        rots[1][0]      = transform->rot_i[t_b];
        rots[1][1]      = transform->rot_j[t_b];
        rots[1][2]      = transform->rot_k[t_b];
        rots[1][3]      = transform->rot_r[t_b];
        cens[1][0]      = collision->center_x[c_b];
        cens[1][1]      = collision->center_y[c_b];
        cens[1][2]      = collision->center_z[c_b];
        internals[1][0] = collision->internal_0[c_b];
        internals[1][1] = collision->internal_1[c_b];
        internals[1][2] = collision->internal_2[c_b];
        internals[1][3] = collision->internal_3[c_b];
        internals[1][4] = collision->internal_4[c_b];
        internals[1][5] = collision->internal_5[c_b];
        shapes[1]       = collision->shape[c_b];
        vels[1][0]      = physics->vel_x[p_b];
        vels[1][1]      = physics->vel_y[p_b];
        vels[1][2]      = physics->vel_z[p_b];
        ws[1][0]        = physics->w_x[p_b];
        ws[1][1]        = physics->w_y[p_b];
        ws[1][2]        = physics->w_z[p_b];
        
        //////
        simplex = (dm_simplex){ 0 };
        
        float supports[2][3];
        if(!dm_physics_gjk(pos, rots, cens, internals, shapes, supports, &simplex)) continue;
        
        assert(simplex.size==4);
        
        manifold = &manager->manifolds[manager->num_manifolds++];
        *manifold = (dm_contact_manifold){ 0 };
        
        collision->flag[c_a] = COLLISION_FLAG_YES;
        
        manifold->contact_data[0].vel_x         = &physics->vel_x[p_a];
        manifold->contact_data[0].vel_y         = &physics->vel_y[p_a];
        manifold->contact_data[0].vel_z         = &physics->vel_z[p_a];
        manifold->contact_data[0].w_x           = &physics->w_x[p_a];
        manifold->contact_data[0].w_y           = &physics->w_y[p_a];
        manifold->contact_data[0].w_z           = &physics->w_z[p_a];
        manifold->contact_data[0].mass          = physics->mass[p_a];
        manifold->contact_data[0].inv_mass      = physics->inv_mass[p_a];
        manifold->contact_data[0].v_damp        = physics->damping_v[p_a];
        manifold->contact_data[0].w_damp        = physics->damping_w[p_a];
        manifold->contact_data[0].i_body_inv_00 = rigid_body->i_body_inv_00[r_a];
        manifold->contact_data[0].i_body_inv_11 = rigid_body->i_body_inv_11[r_a];
        manifold->contact_data[0].i_body_inv_22 = rigid_body->i_body_inv_22[r_a];
        
        collision->flag[c_b] = COLLISION_FLAG_YES;
        
        manifold->contact_data[1].vel_x         = &physics->vel_x[p_b];
        manifold->contact_data[1].vel_y         = &physics->vel_y[p_b];
        manifold->contact_data[1].vel_z         = &physics->vel_z[p_b];
        manifold->contact_data[1].w_x           = &physics->w_x[p_b];
        manifold->contact_data[1].w_y           = &physics->w_y[p_b];
        manifold->contact_data[1].w_z           = &physics->w_z[p_b];
        manifold->contact_data[1].mass          = physics->mass[p_b];
        manifold->contact_data[1].inv_mass      = physics->inv_mass[p_b];
        manifold->contact_data[1].v_damp        = physics->damping_v[p_b];
        manifold->contact_data[1].w_damp        = physics->damping_w[p_b];
        manifold->contact_data[1].i_body_inv_00 = rigid_body->i_body_inv_00[r_b];
        manifold->contact_data[1].i_body_inv_11 = rigid_body->i_body_inv_11[r_b];
        manifold->contact_data[1].i_body_inv_22 = rigid_body->i_body_inv_22[r_b];
        
        if(!dm_physics_collide_entities(pos, rots, cens, internals, vels, ws, shapes, &simplex, manifold)) return false;
        
        // resize manifolds
        dm_grow_dyn_array((void**)&manager->manifolds, manager->num_manifolds, &manager->manifold_capacity, sizeof(dm_contact_manifold), PHYSICS_SYSTEM_DYNAMIC_LOAD_FACTOR, PHYSICS_SYSTEM_DYNAMIC_RESIZE_FACTOR);
    }
    
    return true;
}

/********************
COLLISION RESOLUTION
solves constraints generated in narrowphase
using impulse solver
**********************/
void physics_system_solve_constraints(physics_system_manager* manager, dm_context* context)
{
    for(uint32_t iter=0; iter<PHYSICS_SYSTEM_CONSTRAINT_ITER; iter++)
    {
        for(uint32_t m=0; m<manager->num_manifolds; m++)
        {
            dm_physics_apply_constraints(&manager->manifolds[m]);
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
void physics_system_update_entities(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    const dm_ecs_id r_id = manager->rigid_body;
    
    component_transform*  transform = dm_ecs_get_component_block(t_id, context);
    component_physics*    physics   = dm_ecs_get_component_block(p_id, context);
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    
    uint32_t t_index, p_index, r_index;
    
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
    
    float pos_x[DM_ECS_MAX_ENTITIES], pos_y[DM_ECS_MAX_ENTITIES], pos_z[DM_ECS_MAX_ENTITIES];
    float rot_i[DM_ECS_MAX_ENTITIES], rot_j[DM_ECS_MAX_ENTITIES], rot_k[DM_ECS_MAX_ENTITIES], rot_r[DM_ECS_MAX_ENTITIES];
    float vel_x[DM_ECS_MAX_ENTITIES], vel_y[DM_ECS_MAX_ENTITIES], vel_z[DM_ECS_MAX_ENTITIES];
    float w_x[DM_ECS_MAX_ENTITIES], w_y[DM_ECS_MAX_ENTITIES], w_z[DM_ECS_MAX_ENTITIES];
    float l_x[DM_ECS_MAX_ENTITIES], l_y[DM_ECS_MAX_ENTITIES], l_z[DM_ECS_MAX_ENTITIES];
    float inv_mass[DM_ECS_MAX_ENTITIES];
    float force_x[DM_ECS_MAX_ENTITIES], force_y[DM_ECS_MAX_ENTITIES], force_z[DM_ECS_MAX_ENTITIES];
    float torque_x[DM_ECS_MAX_ENTITIES], torque_y[DM_ECS_MAX_ENTITIES], torque_z[DM_ECS_MAX_ENTITIES];
    float i_inv_00[DM_ECS_MAX_ENTITIES], i_inv_01[DM_ECS_MAX_ENTITIES], i_inv_02[DM_ECS_MAX_ENTITIES];
    float i_inv_10[DM_ECS_MAX_ENTITIES], i_inv_11[DM_ECS_MAX_ENTITIES], i_inv_12[DM_ECS_MAX_ENTITIES];
    float i_inv_20[DM_ECS_MAX_ENTITIES], i_inv_21[DM_ECS_MAX_ENTITIES], i_inv_22[DM_ECS_MAX_ENTITIES];
    
    uint32_t i=0;
    for(;i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        p_index = system->entity_indices[i][p_id];
        r_index = system->entity_indices[i][r_id];
        
        pos_x[i] = transform->pos_x[t_index];
        pos_y[i] = transform->pos_y[t_index];
        pos_z[i] = transform->pos_z[t_index];
        
        rot_i[i] = transform->rot_i[t_index];
        rot_j[i] = transform->rot_j[t_index];
        rot_k[i] = transform->rot_k[t_index];
        rot_r[i] = transform->rot_r[t_index];
        
        vel_x[i] = physics->vel_x[p_index];
        vel_y[i] = physics->vel_y[p_index];
        vel_z[i] = physics->vel_z[p_index];
        
        w_x[i] = physics->w_x[p_index];
        w_y[i] = physics->w_y[p_index];
        w_z[i] = physics->w_z[p_index];
        
        l_x[i] = physics->l_x[p_index];
        l_y[i] = physics->l_y[p_index];
        l_z[i] = physics->l_z[p_index];
        
        inv_mass[i] = physics->inv_mass[p_index];
        
        force_x[i] = physics->force_x[p_index];
        force_y[i] = physics->force_y[p_index];
        force_z[i] = physics->force_z[p_index];
        
        torque_x[i] = physics->torque_x[p_index];
        torque_y[i] = physics->torque_y[p_index];
        torque_z[i] = physics->torque_z[p_index];
        
        i_inv_00[i] = rigid_body->i_inv_00[r_index];
        i_inv_01[i] = rigid_body->i_inv_01[r_index];
        i_inv_02[i] = rigid_body->i_inv_02[r_index];
        
        i_inv_10[i] = rigid_body->i_inv_10[r_index];
        i_inv_11[i] = rigid_body->i_inv_11[r_index];
        i_inv_12[i] = rigid_body->i_inv_12[r_index];
        
        i_inv_20[i] = rigid_body->i_inv_20[r_index];
        i_inv_21[i] = rigid_body->i_inv_21[r_index];
        i_inv_22[i] = rigid_body->i_inv_22[r_index];
    }
    
    i=0;
    for(i=0; i<system->entity_count; i++)
    {
        // integrate position
        pos_x[i] += vel_x[i] * DM_PHYSICS_FIXED_DT;
        pos_y[i] += vel_y[i] * DM_PHYSICS_FIXED_DT;
        pos_z[i] += vel_z[i] * DM_PHYSICS_FIXED_DT;
        
        // integrate velocity
        dt_mass = inv_mass[i] * DM_PHYSICS_FIXED_DT;
        
        vel_x[i] += force_x[i] * dt_mass;
        vel_y[i] += force_y[i] * dt_mass;
        vel_z[i] += force_z[i] * dt_mass;
        
        // integrate angular momentum
        l_x[i] += torque_x[i] * DM_PHYSICS_FIXED_DT;
        l_y[i] += torque_y[i] * DM_PHYSICS_FIXED_DT;
        l_z[i] += torque_z[i] * DM_PHYSICS_FIXED_DT;
        
        // integrate angular velocity
        w_x[i] += i_inv_00[i] * l_x[i];
        w_x[i] += i_inv_01[i] * l_y[i];
        w_x[i] += i_inv_02[i] * l_z[i];
        
        w_y[i] += i_inv_10[i] * l_x[i];
        w_y[i] += i_inv_11[i] * l_y[i];
        w_y[i] += i_inv_12[i] * l_z[i];
        
        w_z[i] += i_inv_20[i] * l_x[i];
        w_z[i] += i_inv_21[i] * l_y[i];
        w_z[i] += i_inv_22[i] * l_z[i];
        
        // integrate rotation
        new_rot_i  = w_x[i] * rot_r[i];
        new_rot_i += w_y[i] * rot_k[i];
        new_rot_i -= w_z[i] * rot_j[i];
        new_rot_i *= half_dt;
        new_rot_i += rot_i[i];
        
        new_rot_j  = -w_x[i] * rot_k[i];
        new_rot_j +=  w_y[i] * rot_r[i];
        new_rot_j +=  w_z[i] * rot_i[i];
        new_rot_j *= half_dt;
        new_rot_j += rot_j[i];
        
        new_rot_k  = w_x[i] * rot_j[i];
        new_rot_k -= w_y[i] * rot_i[i];
        new_rot_k += w_z[i] * rot_r[i];
        new_rot_k *= half_dt;
        new_rot_k += rot_k[i];
        
        new_rot_r  = -w_x[i] * rot_i[i];
        new_rot_r -=  w_y[i] * rot_j[i];
        new_rot_r -=  w_z[i] * rot_k[i];
        new_rot_r *= half_dt;
        new_rot_r += rot_r[i];
        
        new_rot_mag  = new_rot_i * new_rot_i;
        new_rot_mag += new_rot_j * new_rot_j;
        new_rot_mag += new_rot_k * new_rot_k;
        new_rot_mag += new_rot_r * new_rot_r;
        new_rot_mag  = dm_sqrtf(new_rot_mag);
        
        new_rot_i /= new_rot_mag;
        new_rot_j /= new_rot_mag;
        new_rot_k /= new_rot_mag;
        new_rot_r /= new_rot_mag;
        
        rot_i[i] = new_rot_i;
        rot_j[i] = new_rot_j;
        rot_k[i] = new_rot_k;
        rot_r[i] = new_rot_r;
        
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
        body_inv_00 = orientation_00 * i_inv_00[i];
        body_inv_01 = orientation_10 * i_inv_11[i];
        body_inv_02 = orientation_20 * i_inv_22[i];
        
        body_inv_10 = orientation_01 * i_inv_00[i];
        body_inv_11 = orientation_11 * i_inv_11[i];
        body_inv_12 = orientation_21 * i_inv_22[i];
        
        body_inv_20 = orientation_02 * i_inv_00[i];
        body_inv_21 = orientation_12 * i_inv_11[i];
        body_inv_22 = orientation_22 * i_inv_22[i];
        
        // final i_inv matrix
        i_inv_00[i]  = orientation_00 * body_inv_00;
        i_inv_00[i] += orientation_01 * body_inv_10;
        i_inv_00[i] += orientation_02 * body_inv_20;
        
        i_inv_01[i]  = orientation_00 * body_inv_01;
        i_inv_01[i] += orientation_01 * body_inv_11;
        i_inv_01[i] += orientation_02 * body_inv_21;
        
        i_inv_02[i]  = orientation_00 * body_inv_02;
        i_inv_02[i] += orientation_01 * body_inv_12;
        i_inv_02[i] += orientation_02 * body_inv_22;
        
        i_inv_10[i]  = orientation_10 * body_inv_00;
        i_inv_10[i] += orientation_11 * body_inv_10;
        i_inv_10[i] += orientation_12 * body_inv_20;
        
        i_inv_11[i]  = orientation_10 * body_inv_01;
        i_inv_11[i] += orientation_11 * body_inv_11;
        i_inv_11[i] += orientation_12 * body_inv_21;
        
        i_inv_12[i]  = orientation_10 * body_inv_02;
        i_inv_12[i] += orientation_11 * body_inv_12;
        i_inv_12[i] += orientation_12 * body_inv_22;
        
        i_inv_20[i]  = orientation_20 * body_inv_00;
        i_inv_20[i] += orientation_21 * body_inv_10;
        i_inv_20[i] += orientation_22 * body_inv_20;
        
        i_inv_21[i]  = orientation_20 * body_inv_01;
        i_inv_21[i] += orientation_21 * body_inv_11;
        i_inv_21[i] += orientation_22 * body_inv_21;
        
        i_inv_22[i]  = orientation_20 * body_inv_02;
        i_inv_22[i] += orientation_21 * body_inv_12;
        i_inv_22[i] += orientation_22 * body_inv_22;
    }
    
    // assign
    i=0;
    for(;i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        p_index = system->entity_indices[i][p_id];
        r_index = system->entity_indices[i][r_id];
        
        transform->pos_x[t_index] = pos_x[i];
        transform->pos_y[t_index] = pos_y[i];
        transform->pos_z[t_index] = pos_z[i];
        
        transform->rot_i[t_index] = rot_i[i];
        transform->rot_j[t_index] = rot_j[i];
        transform->rot_k[t_index] = rot_k[i];
        transform->rot_r[t_index] = rot_r[i];
        
        physics->vel_x[p_index] = vel_x[i];
        physics->vel_y[p_index] = vel_y[i];
        physics->vel_z[p_index] = vel_z[i];
        
        physics->w_x[p_index] = w_x[i];
        physics->w_y[p_index] = w_y[i];
        physics->w_z[p_index] = w_z[i];
        
        physics->l_x[p_index] = l_x[i];
        physics->l_y[p_index] = l_y[i];
        physics->l_z[p_index] = l_z[i];
        
        rigid_body->i_inv_00[r_index] = i_inv_00[i];
        rigid_body->i_inv_01[r_index] = i_inv_01[i];
        rigid_body->i_inv_02[r_index] = i_inv_02[i];
        
        rigid_body->i_inv_10[r_index] = i_inv_10[i];
        rigid_body->i_inv_11[r_index] = i_inv_11[i];
        rigid_body->i_inv_12[r_index] = i_inv_12[i];
        
        rigid_body->i_inv_20[r_index] = i_inv_20[i];
        rigid_body->i_inv_21[r_index] = i_inv_21[i];
        rigid_body->i_inv_22[r_index] = i_inv_22[i];
    }
}

void physics_system_update_entities_simd(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id p_id = manager->physics;
    const dm_ecs_id r_id = manager->rigid_body;
    
    component_transform*  transform = dm_ecs_get_component_block(t_id, context);
    component_physics*    physics   = dm_ecs_get_component_block(p_id, context);
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    
    uint32_t t_index, p_index, r_index;
    
    dm_mm_float pos_x, pos_y, pos_z;
    dm_mm_float rot_i, rot_j, rot_k, rot_r;
    dm_mm_float vel_x, vel_y, vel_z;
    dm_mm_float w_x, w_y, w_z;
    dm_mm_float l_x, l_y, l_z;
    dm_mm_float force_x, force_y, force_z;
    dm_mm_float torque_x, torque_y, torque_z;
    dm_mm_float dt_mass;
    dm_mm_float i_inv_00, i_inv_01, i_inv_02;
    dm_mm_float i_inv_10, i_inv_11, i_inv_12;
    dm_mm_float i_inv_20, i_inv_21, i_inv_22;
    
    dm_mm_float new_rot_i, new_rot_j, new_rot_k, new_rot_r;
    dm_mm_float new_rot_mag;
    
    dm_mm_float orientation_00, orientation_01, orientation_02;
    dm_mm_float orientation_10, orientation_11, orientation_12;
    dm_mm_float orientation_20, orientation_21, orientation_22;
    
    dm_mm_float body_inv_00, body_inv_01, body_inv_02;
    dm_mm_float body_inv_10, body_inv_11, body_inv_12;
    dm_mm_float body_inv_20, body_inv_21, body_inv_22;
    
    dm_mm_float dt      = dm_mm_set1_ps(DM_PHYSICS_FIXED_DT);
    dm_mm_float half_dt = dm_mm_set1_ps(0.5f * DM_PHYSICS_FIXED_DT);
    dm_mm_float ones    = dm_mm_set1_ps(1);
    dm_mm_float twos    = dm_mm_set1_ps(2);
    dm_mm_float zeroes  = dm_mm_set1_ps(0);
    
    float pos_x_d[DM_ECS_MAX_ENTITIES], pos_y_d[DM_ECS_MAX_ENTITIES], pos_z_d[DM_ECS_MAX_ENTITIES];
    float rot_i_d[DM_ECS_MAX_ENTITIES], rot_j_d[DM_ECS_MAX_ENTITIES], rot_k_d[DM_ECS_MAX_ENTITIES], rot_r_d[DM_ECS_MAX_ENTITIES];
    float vel_x_d[DM_ECS_MAX_ENTITIES], vel_y_d[DM_ECS_MAX_ENTITIES], vel_z_d[DM_ECS_MAX_ENTITIES];
    float w_x_d[DM_ECS_MAX_ENTITIES], w_y_d[DM_ECS_MAX_ENTITIES], w_z_d[DM_ECS_MAX_ENTITIES];
    float l_x_d[DM_ECS_MAX_ENTITIES], l_y_d[DM_ECS_MAX_ENTITIES], l_z_d[DM_ECS_MAX_ENTITIES];
    float inv_mass_d[DM_ECS_MAX_ENTITIES];
    float force_x_d[DM_ECS_MAX_ENTITIES], force_y_d[DM_ECS_MAX_ENTITIES], force_z_d[DM_ECS_MAX_ENTITIES];
    float torque_x_d[DM_ECS_MAX_ENTITIES], torque_y_d[DM_ECS_MAX_ENTITIES], torque_z_d[DM_ECS_MAX_ENTITIES];
    float i_inv_00_d[DM_ECS_MAX_ENTITIES], i_inv_01_d[DM_ECS_MAX_ENTITIES], i_inv_02_d[DM_ECS_MAX_ENTITIES];
    float i_inv_10_d[DM_ECS_MAX_ENTITIES], i_inv_11_d[DM_ECS_MAX_ENTITIES], i_inv_12_d[DM_ECS_MAX_ENTITIES];
    float i_inv_20_d[DM_ECS_MAX_ENTITIES], i_inv_21_d[DM_ECS_MAX_ENTITIES], i_inv_22_d[DM_ECS_MAX_ENTITIES];
    
    uint32_t i=0;
    for(;i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        p_index = system->entity_indices[i][p_id];
        r_index = system->entity_indices[i][r_id];
        
        pos_x_d[i] = transform->pos_x[t_index];
        pos_y_d[i] = transform->pos_y[t_index];
        pos_z_d[i] = transform->pos_z[t_index];
        
        rot_i_d[i] = transform->rot_i[t_index];
        rot_j_d[i] = transform->rot_j[t_index];
        rot_k_d[i] = transform->rot_k[t_index];
        rot_r_d[i] = transform->rot_r[t_index];
        
        vel_x_d[i] = physics->vel_x[p_index];
        vel_y_d[i] = physics->vel_y[p_index];
        vel_z_d[i] = physics->vel_z[p_index];
        
        w_x_d[i] = physics->w_x[p_index];
        w_y_d[i] = physics->w_y[p_index];
        w_z_d[i] = physics->w_z[p_index];
        
        l_x_d[i] = physics->l_x[p_index];
        l_y_d[i] = physics->l_y[p_index];
        l_z_d[i] = physics->l_z[p_index];
        
        inv_mass_d[i] = physics->inv_mass[p_index];
        
        force_x_d[i] = physics->force_x[p_index];
        force_y_d[i] = physics->force_y[p_index];
        force_z_d[i] = physics->force_z[p_index];
        
        torque_x_d[i] = physics->torque_x[p_index];
        torque_y_d[i] = physics->torque_y[p_index];
        torque_z_d[i] = physics->torque_z[p_index];
        
        i_inv_00_d[i] = rigid_body->i_inv_00[r_index];
        i_inv_01_d[i] = rigid_body->i_inv_01[r_index];
        i_inv_02_d[i] = rigid_body->i_inv_02[r_index];
        
        i_inv_10_d[i] = rigid_body->i_inv_10[r_index];
        i_inv_11_d[i] = rigid_body->i_inv_11[r_index];
        i_inv_12_d[i] = rigid_body->i_inv_12[r_index];
        
        i_inv_20_d[i] = rigid_body->i_inv_20[r_index];
        i_inv_21_d[i] = rigid_body->i_inv_21[r_index];
        i_inv_22_d[i] = rigid_body->i_inv_22[r_index];
    }
    
    i=0;
    for(; (system->entity_count-i)>DM_SIMD_N; i+=DM_SIMD_N)
    {
        pos_x = dm_mm_load_ps(pos_x_d + i);
        pos_y = dm_mm_load_ps(pos_y_d + i);
        pos_z = dm_mm_load_ps(pos_z_d + i);
        
        rot_i = dm_mm_load_ps(rot_i_d + i);
        rot_j = dm_mm_load_ps(rot_j_d + i);
        rot_k = dm_mm_load_ps(rot_k_d + i);
        rot_r = dm_mm_load_ps(rot_r_d + i);
        
        vel_x = dm_mm_load_ps(vel_x_d + i);
        vel_y = dm_mm_load_ps(vel_y_d + i);
        vel_z = dm_mm_load_ps(vel_z_d + i);
        
        w_x = dm_mm_load_ps(w_x_d + i);
        w_y = dm_mm_load_ps(w_y_d + i);
        w_z = dm_mm_load_ps(w_z_d + i);
        
        l_x = dm_mm_load_ps(l_x_d + i);
        l_y = dm_mm_load_ps(l_y_d + i);
        l_z = dm_mm_load_ps(l_z_d + i);
        
        force_x = dm_mm_load_ps(force_x_d + i);
        force_y = dm_mm_load_ps(force_y_d + i);
        force_z = dm_mm_load_ps(force_z_d + i);
        
        torque_x = dm_mm_load_ps(torque_x_d + i);
        torque_y = dm_mm_load_ps(torque_y_d + i);
        torque_z = dm_mm_load_ps(torque_z_d + i);
        
        i_inv_00 = dm_mm_load_ps(i_inv_00_d + i);
        i_inv_01 = dm_mm_load_ps(i_inv_01_d + i);
        i_inv_02 = dm_mm_load_ps(i_inv_02_d + i);
        
        i_inv_10 = dm_mm_load_ps(i_inv_10_d + i);
        i_inv_11 = dm_mm_load_ps(i_inv_11_d + i);
        i_inv_12 = dm_mm_load_ps(i_inv_12_d + i);
        
        i_inv_20 = dm_mm_load_ps(i_inv_20_d + i);
        i_inv_21 = dm_mm_load_ps(i_inv_21_d + i);
        i_inv_22 = dm_mm_load_ps(i_inv_22_d + i);
        
        // integrate position
        pos_x = dm_mm_fmadd_ps(vel_x, dt, pos_x);
        pos_y = dm_mm_fmadd_ps(vel_y, dt, pos_y);
        pos_z = dm_mm_fmadd_ps(vel_z, dt, pos_z);
        
        // integrate velocity
        dt_mass = dm_mm_load_ps(inv_mass_d + i);
        dt_mass = dm_mm_mul_ps(dt_mass, dt);
        
        vel_x = dm_mm_fmadd_ps(force_x, dt_mass, vel_x);
        vel_y = dm_mm_fmadd_ps(force_y, dt_mass, vel_y);
        vel_z = dm_mm_fmadd_ps(force_z, dt_mass, vel_z);
        
        // integrate angular momentum
        l_x = dm_mm_fmadd_ps(torque_x, dt, l_x);
        l_y = dm_mm_fmadd_ps(torque_y, dt, l_y);
        l_z = dm_mm_fmadd_ps(torque_z, dt, l_z);
        
        // integrate angular velocity
        w_x = dm_mm_fmadd_ps(i_inv_00, l_x, w_x);
        w_x = dm_mm_fmadd_ps(i_inv_01, l_y, w_x);
        w_x = dm_mm_fmadd_ps(i_inv_02, l_z, w_x);
        
        w_y = dm_mm_fmadd_ps(i_inv_10, l_x, w_y);
        w_y = dm_mm_fmadd_ps(i_inv_11, l_y, w_y);
        w_y = dm_mm_fmadd_ps(i_inv_12, l_z, w_y);
        
        w_z = dm_mm_fmadd_ps(i_inv_20, l_x, w_z);
        w_z = dm_mm_fmadd_ps(i_inv_21, l_y, w_z);
        w_z = dm_mm_fmadd_ps(i_inv_22, l_z, w_z);
        
        // integrate rotation
        new_rot_i = dm_mm_mul_ps(w_x, rot_r);
        new_rot_i = dm_mm_fmadd_ps(w_x, rot_r, new_rot_i);
        new_rot_i = dm_mm_fmadd_ps(w_y, rot_k, new_rot_i);
        new_rot_i = dm_mm_sub_ps(new_rot_i, dm_mm_mul_ps(w_z, rot_j));
        new_rot_i = dm_mm_fmadd_ps(new_rot_i, half_dt, rot_i);
        
        new_rot_j = dm_mm_sub_ps(zeroes, dm_mm_mul_ps(w_x, rot_k));
        new_rot_j = dm_mm_fmadd_ps(w_y, rot_r, new_rot_j);
        new_rot_j = dm_mm_fmadd_ps(w_z, rot_i, new_rot_j);
        new_rot_j = dm_mm_fmadd_ps(new_rot_i, half_dt, rot_j);
        
        new_rot_k = dm_mm_mul_ps(w_x, rot_j);
        new_rot_k = dm_mm_sub_ps(new_rot_k, dm_mm_mul_ps(w_x, rot_i));
        new_rot_k = dm_mm_fmadd_ps(w_z, rot_r, new_rot_k);
        new_rot_k = dm_mm_fmadd_ps(new_rot_k, half_dt, rot_k);
        
        new_rot_r = dm_mm_sub_ps(zeroes, dm_mm_mul_ps(w_x, rot_i));
        new_rot_r = dm_mm_sub_ps(new_rot_r, dm_mm_mul_ps(w_y, rot_j));
        new_rot_r = dm_mm_sub_ps(new_rot_r, dm_mm_mul_ps(w_z, rot_k));
        new_rot_r = dm_mm_fmadd_ps(new_rot_r, half_dt, rot_r);
        
        new_rot_mag = dm_mm_mul_ps(new_rot_i, new_rot_i);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_j, new_rot_j, new_rot_mag);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_k, new_rot_k, new_rot_mag);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_r, new_rot_r, new_rot_mag);;
        new_rot_mag = dm_mm_sqrt_ps(new_rot_mag);
        new_rot_mag = dm_mm_div_ps(ones, new_rot_mag);
        
        new_rot_i = dm_mm_mul_ps(new_rot_i, new_rot_mag);
        new_rot_j = dm_mm_mul_ps(new_rot_j, new_rot_mag);
        new_rot_k = dm_mm_mul_ps(new_rot_k, new_rot_mag);
        new_rot_r = dm_mm_mul_ps(new_rot_r, new_rot_mag);
        
        // update i_inv
        orientation_00 = dm_mm_mul_ps(new_rot_j, new_rot_j);
        orientation_00 = dm_mm_fmadd_ps(new_rot_k, new_rot_k, orientation_00);
        orientation_00 = dm_mm_mul_ps(twos, orientation_00);
        orientation_00 = dm_mm_sub_ps(ones, orientation_00);
        
        orientation_01 = dm_mm_mul_ps(new_rot_i, new_rot_j);
        orientation_01 = dm_mm_fmadd_ps(new_rot_k, new_rot_r, orientation_01);
        orientation_01 = dm_mm_mul_ps(twos, orientation_01);
        
        orientation_02 = dm_mm_mul_ps(new_rot_i, new_rot_k);
        orientation_02 = dm_mm_sub_ps(orientation_02, dm_mm_mul_ps(new_rot_j, new_rot_r));
        orientation_02 = dm_mm_mul_ps(twos, orientation_02);
        
        orientation_10 = dm_mm_mul_ps(new_rot_i, new_rot_j);
        orientation_10 = dm_mm_sub_ps(orientation_10, dm_mm_mul_ps(new_rot_k, new_rot_r));
        orientation_10 = dm_mm_mul_ps(twos, orientation_10);
        
        orientation_11 = dm_mm_mul_ps(new_rot_i, new_rot_i);
        orientation_11 = dm_mm_fmadd_ps(new_rot_k, new_rot_k, orientation_11);
        orientation_11 = dm_mm_mul_ps(twos, orientation_11);
        orientation_11 = dm_mm_sub_ps(ones, orientation_11);
        
        orientation_12 = dm_mm_mul_ps(new_rot_j, new_rot_k);
        orientation_12 = dm_mm_fmadd_ps(new_rot_i, new_rot_r, orientation_12);
        orientation_12 = dm_mm_mul_ps(twos, orientation_12);
        
        orientation_20 = dm_mm_mul_ps(new_rot_i, new_rot_k);
        orientation_20 = dm_mm_fmadd_ps(new_rot_j, new_rot_r, orientation_20);
        orientation_20 = dm_mm_mul_ps(twos, orientation_20);
        
        orientation_21 = dm_mm_mul_ps(new_rot_j, new_rot_k);
        orientation_21 = dm_mm_sub_ps(orientation_21, dm_mm_mul_ps(new_rot_i, new_rot_r));
        orientation_21 = dm_mm_mul_ps(twos, orientation_21);
        
        orientation_22 = dm_mm_mul_ps(new_rot_i, new_rot_i);
        dm_mm_fmadd_ps(new_rot_j, new_rot_j, orientation_22);
        dm_mm_mul_ps(twos, orientation_22);
        dm_mm_sub_ps(ones, orientation_22);
        
        // orientation is transposed here
        body_inv_00 = dm_mm_mul_ps(orientation_00, i_inv_00);
        body_inv_01 = dm_mm_mul_ps(orientation_10, i_inv_11);
        body_inv_02 = dm_mm_mul_ps(orientation_20, i_inv_22);
        
        body_inv_10 = dm_mm_mul_ps(orientation_01, i_inv_00);
        body_inv_11 = dm_mm_mul_ps(orientation_11, i_inv_11);
        body_inv_12 = dm_mm_mul_ps(orientation_21, i_inv_22);
        
        body_inv_20 = dm_mm_mul_ps(orientation_02, i_inv_00);
        body_inv_21 = dm_mm_mul_ps(orientation_12, i_inv_11);
        body_inv_22 = dm_mm_mul_ps(orientation_22, i_inv_22);
        
        // final i_inv matrix
        i_inv_00 = dm_mm_mul_ps(orientation_00, body_inv_00);
        i_inv_00 = dm_mm_fmadd_ps(orientation_01, body_inv_10, i_inv_00);
        i_inv_00 = dm_mm_fmadd_ps(orientation_02, body_inv_20, i_inv_00);
        
        i_inv_01 = dm_mm_mul_ps(orientation_00, body_inv_01);
        i_inv_01 = dm_mm_fmadd_ps(orientation_01, body_inv_11, i_inv_01);
        i_inv_01 = dm_mm_fmadd_ps(orientation_02, body_inv_21, i_inv_01);;
        
        i_inv_02 = dm_mm_mul_ps(orientation_00, body_inv_02);
        i_inv_02 = dm_mm_fmadd_ps(orientation_01, body_inv_12, i_inv_02);
        i_inv_02 = dm_mm_fmadd_ps(orientation_02, body_inv_22, i_inv_02);
        
        i_inv_10 = dm_mm_mul_ps(orientation_10, body_inv_00);
        i_inv_10 = dm_mm_fmadd_ps(orientation_11, body_inv_10, i_inv_10);
        i_inv_10 = dm_mm_fmadd_ps(orientation_12, body_inv_20, i_inv_10);
        
        i_inv_11 = dm_mm_mul_ps(orientation_10, body_inv_01);
        i_inv_11 = dm_mm_fmadd_ps(orientation_11, body_inv_11, i_inv_11);
        i_inv_11 = dm_mm_fmadd_ps(orientation_12, body_inv_21, i_inv_11);
        
        i_inv_12 = dm_mm_mul_ps(orientation_10, body_inv_02);
        i_inv_12 = dm_mm_fmadd_ps(orientation_11, body_inv_12, i_inv_12);
        i_inv_12 = dm_mm_fmadd_ps(orientation_12, body_inv_22, i_inv_12);
        
        i_inv_20 = dm_mm_mul_ps(orientation_20, body_inv_00);
        i_inv_20 = dm_mm_fmadd_ps(orientation_21, body_inv_10, i_inv_20);
        i_inv_20 = dm_mm_fmadd_ps(orientation_22, body_inv_20, i_inv_20);
        
        i_inv_21 = dm_mm_mul_ps(orientation_20, body_inv_01);
        i_inv_21 = dm_mm_fmadd_ps(orientation_21, body_inv_11, i_inv_21);
        i_inv_21 = dm_mm_fmadd_ps(orientation_22, body_inv_21, i_inv_21);
        
        i_inv_22 = dm_mm_mul_ps(orientation_20, body_inv_02);
        i_inv_22 = dm_mm_fmadd_ps(orientation_21, body_inv_12, i_inv_22);
        i_inv_22 = dm_mm_fmadd_ps(orientation_22, body_inv_22, i_inv_22);
        
        // store
        dm_mm_store_ps(pos_x_d + i, pos_x);
        dm_mm_store_ps(pos_y_d + i, pos_y);
        dm_mm_store_ps(pos_z_d + i, pos_z);
        
        dm_mm_store_ps(rot_i_d + i, rot_i);
        dm_mm_store_ps(rot_j_d + i, rot_j);
        dm_mm_store_ps(rot_k_d + i, rot_k);
        dm_mm_store_ps(rot_r_d + i, rot_r);
        
        dm_mm_store_ps(vel_x_d + i, vel_x);
        dm_mm_store_ps(vel_y_d + i, vel_y);
        dm_mm_store_ps(vel_z_d + i, vel_z);
        
        dm_mm_store_ps(w_x_d + i, w_x);
        dm_mm_store_ps(w_y_d + i, w_y);
        dm_mm_store_ps(w_z_d + i, w_z);
        
        dm_mm_store_ps(l_x_d + i, l_x);
        dm_mm_store_ps(l_y_d + i, l_y);
        dm_mm_store_ps(l_z_d + i, l_z);
        
        dm_mm_store_ps(i_inv_00_d + i, i_inv_00);
        dm_mm_store_ps(i_inv_01_d + i, i_inv_01);
        dm_mm_store_ps(i_inv_02_d + i, i_inv_02);
        
        dm_mm_store_ps(i_inv_10_d + i, i_inv_10);
        dm_mm_store_ps(i_inv_11_d + i, i_inv_11);
        dm_mm_store_ps(i_inv_12_d + i, i_inv_12);
        
        dm_mm_store_ps(i_inv_20_d + i, i_inv_20);
        dm_mm_store_ps(i_inv_21_d + i, i_inv_21);
        dm_mm_store_ps(i_inv_22_d + i, i_inv_22);
    }
    
    uint32_t leftovers = system->entity_count - i;
    if(leftovers>0 && leftovers<DM_SIMD_N) 
    {
        pos_x = dm_mm_set1_ps(0);
        pos_y = dm_mm_set1_ps(0);
        pos_z = dm_mm_set1_ps(0);
        
        rot_i = dm_mm_set1_ps(0);
        rot_j = dm_mm_set1_ps(0);
        rot_k = dm_mm_set1_ps(0);
        rot_r = dm_mm_set1_ps(0);
        
        vel_x = dm_mm_set1_ps(0);
        vel_y = dm_mm_set1_ps(0);
        vel_z = dm_mm_set1_ps(0);
        
        w_x = dm_mm_set1_ps(0);
        w_y = dm_mm_set1_ps(0);
        w_z = dm_mm_set1_ps(0);
        
        l_x = dm_mm_set1_ps(0);
        l_y = dm_mm_set1_ps(0);
        l_z = dm_mm_set1_ps(0);
        
        force_x = dm_mm_set1_ps(0);
        force_y = dm_mm_set1_ps(0);
        force_z = dm_mm_set1_ps(0);
        
        torque_x = dm_mm_set1_ps(0);
        torque_y = dm_mm_set1_ps(0);
        torque_z = dm_mm_set1_ps(0);
        
        i_inv_00 = dm_mm_set1_ps(0);
        i_inv_01 = dm_mm_set1_ps(0);
        i_inv_02 = dm_mm_set1_ps(0);
        
        i_inv_10 = dm_mm_set1_ps(0);
        i_inv_11 = dm_mm_set1_ps(0);
        i_inv_12 = dm_mm_set1_ps(0);
        
        i_inv_20 = dm_mm_set1_ps(0);
        i_inv_21 = dm_mm_set1_ps(0);
        i_inv_22 = dm_mm_set1_ps(0);
        
        pos_x = dm_mm_load_ps(pos_x_d + i);
        pos_y = dm_mm_load_ps(pos_y_d + i);
        pos_z = dm_mm_load_ps(pos_z_d + i);
        
        rot_i = dm_mm_load_ps(rot_i_d + i);
        rot_j = dm_mm_load_ps(rot_j_d + i);
        rot_k = dm_mm_load_ps(rot_k_d + i);
        rot_r = dm_mm_load_ps(rot_r_d + i);
        
        vel_x = dm_mm_load_ps(vel_x_d + i);
        vel_y = dm_mm_load_ps(vel_y_d + i);
        vel_z = dm_mm_load_ps(vel_z_d + i);
        
        w_x = dm_mm_load_ps(w_x_d + i);
        w_y = dm_mm_load_ps(w_y_d + i);
        w_z = dm_mm_load_ps(w_z_d + i);
        
        l_x = dm_mm_load_ps(l_x_d + i);
        l_y = dm_mm_load_ps(l_y_d + i);
        l_z = dm_mm_load_ps(l_z_d + i);
        
        force_x = dm_mm_load_ps(force_x_d + i);
        force_y = dm_mm_load_ps(force_y_d + i);
        force_z = dm_mm_load_ps(force_z_d + i);
        
        torque_x = dm_mm_load_ps(torque_x_d + i);
        torque_y = dm_mm_load_ps(torque_y_d + i);
        torque_z = dm_mm_load_ps(torque_z_d + i);
        
        i_inv_00 = dm_mm_load_ps(i_inv_00_d + i);
        i_inv_01 = dm_mm_load_ps(i_inv_01_d + i);
        i_inv_02 = dm_mm_load_ps(i_inv_02_d + i);
        
        i_inv_10 = dm_mm_load_ps(i_inv_10_d + i);
        i_inv_11 = dm_mm_load_ps(i_inv_11_d + i);
        i_inv_12 = dm_mm_load_ps(i_inv_12_d + i);
        
        i_inv_20 = dm_mm_load_ps(i_inv_20_d + i);
        i_inv_21 = dm_mm_load_ps(i_inv_21_d + i);
        i_inv_22 = dm_mm_load_ps(i_inv_22_d + i);
        
        // integrate position
        pos_x = dm_mm_fmadd_ps(vel_x, dt, pos_x);
        pos_y = dm_mm_fmadd_ps(vel_y, dt, pos_y);
        pos_z = dm_mm_fmadd_ps(vel_z, dt, pos_z);
        
        // integrate velocity
        dt_mass = dm_mm_load_ps(inv_mass_d + i);
        dt_mass = dm_mm_mul_ps(dt_mass, dt);
        
        vel_x = dm_mm_fmadd_ps(force_x, dt_mass, vel_x);
        vel_y = dm_mm_fmadd_ps(force_y, dt_mass, vel_y);
        vel_z = dm_mm_fmadd_ps(force_z, dt_mass, vel_z);
        
        // integrate angular momentum
        l_x = dm_mm_fmadd_ps(torque_x, dt, l_x);
        l_y = dm_mm_fmadd_ps(torque_y, dt, l_y);
        l_z = dm_mm_fmadd_ps(torque_z, dt, l_z);
        
        // integrate angular velocity
        w_x = dm_mm_fmadd_ps(i_inv_00, l_x, w_x);
        w_x = dm_mm_fmadd_ps(i_inv_01, l_y, w_x);
        w_x = dm_mm_fmadd_ps(i_inv_02, l_z, w_x);
        
        w_y = dm_mm_fmadd_ps(i_inv_10, l_x, w_y);
        w_y = dm_mm_fmadd_ps(i_inv_11, l_y, w_y);
        w_y = dm_mm_fmadd_ps(i_inv_12, l_z, w_y);
        
        w_z = dm_mm_fmadd_ps(i_inv_20, l_x, w_z);
        w_z = dm_mm_fmadd_ps(i_inv_21, l_y, w_z);
        w_z = dm_mm_fmadd_ps(i_inv_22, l_z, w_z);
        
        // integrate rotation
        new_rot_i = dm_mm_mul_ps(w_x, rot_r);
        new_rot_i = dm_mm_fmadd_ps(w_x, rot_r, new_rot_i);
        new_rot_i = dm_mm_fmadd_ps(w_y, rot_k, new_rot_i);
        new_rot_i = dm_mm_sub_ps(new_rot_i, dm_mm_mul_ps(w_z, rot_j));
        new_rot_i = dm_mm_fmadd_ps(new_rot_i, half_dt, rot_i);
        
        new_rot_j = dm_mm_sub_ps(zeroes, dm_mm_mul_ps(w_x, rot_k));
        new_rot_j = dm_mm_fmadd_ps(w_y, rot_r, new_rot_j);
        new_rot_j = dm_mm_fmadd_ps(w_z, rot_i, new_rot_j);
        new_rot_j = dm_mm_fmadd_ps(new_rot_i, half_dt, rot_j);
        
        new_rot_k = dm_mm_mul_ps(w_x, rot_j);
        new_rot_k = dm_mm_sub_ps(new_rot_k, dm_mm_mul_ps(w_x, rot_i));
        new_rot_k = dm_mm_fmadd_ps(w_z, rot_r, new_rot_k);
        new_rot_k = dm_mm_fmadd_ps(new_rot_k, half_dt, rot_k);
        
        new_rot_r = dm_mm_sub_ps(zeroes, dm_mm_mul_ps(w_x, rot_i));
        new_rot_r = dm_mm_sub_ps(new_rot_r, dm_mm_mul_ps(w_y, rot_j));
        new_rot_r = dm_mm_sub_ps(new_rot_r, dm_mm_mul_ps(w_z, rot_k));
        new_rot_r = dm_mm_fmadd_ps(new_rot_r, half_dt, rot_r);
        
        new_rot_mag = dm_mm_mul_ps(new_rot_i, new_rot_i);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_j, new_rot_j, new_rot_mag);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_k, new_rot_k, new_rot_mag);
        new_rot_mag = dm_mm_fmadd_ps(new_rot_r, new_rot_r, new_rot_mag);;
        new_rot_mag = dm_mm_sqrt_ps(new_rot_mag);
        new_rot_mag = dm_mm_div_ps(ones, new_rot_mag);
        
        new_rot_i = dm_mm_mul_ps(new_rot_i, new_rot_mag);
        new_rot_j = dm_mm_mul_ps(new_rot_j, new_rot_mag);
        new_rot_k = dm_mm_mul_ps(new_rot_k, new_rot_mag);
        new_rot_r = dm_mm_mul_ps(new_rot_r, new_rot_mag);
        
        // update i_inv
        orientation_00 = dm_mm_mul_ps(new_rot_j, new_rot_j);
        orientation_00 = dm_mm_fmadd_ps(new_rot_k, new_rot_k, orientation_00);
        orientation_00 = dm_mm_mul_ps(twos, orientation_00);
        orientation_00 = dm_mm_sub_ps(ones, orientation_00);
        
        orientation_01 = dm_mm_mul_ps(new_rot_i, new_rot_j);
        orientation_01 = dm_mm_fmadd_ps(new_rot_k, new_rot_r, orientation_01);
        orientation_01 = dm_mm_mul_ps(twos, orientation_01);
        
        orientation_02 = dm_mm_mul_ps(new_rot_i, new_rot_k);
        orientation_02 = dm_mm_sub_ps(orientation_02, dm_mm_mul_ps(new_rot_j, new_rot_r));
        orientation_02 = dm_mm_mul_ps(twos, orientation_02);
        
        orientation_10 = dm_mm_mul_ps(new_rot_i, new_rot_j);
        orientation_10 = dm_mm_sub_ps(orientation_10, dm_mm_mul_ps(new_rot_k, new_rot_r));
        orientation_10 = dm_mm_mul_ps(twos, orientation_10);
        
        orientation_11 = dm_mm_mul_ps(new_rot_i, new_rot_i);
        orientation_11 = dm_mm_fmadd_ps(new_rot_k, new_rot_k, orientation_11);
        orientation_11 = dm_mm_mul_ps(twos, orientation_11);
        orientation_11 = dm_mm_sub_ps(ones, orientation_11);
        
        orientation_12 = dm_mm_mul_ps(new_rot_j, new_rot_k);
        orientation_12 = dm_mm_fmadd_ps(new_rot_i, new_rot_r, orientation_12);
        orientation_12 = dm_mm_mul_ps(twos, orientation_12);
        
        orientation_20 = dm_mm_mul_ps(new_rot_i, new_rot_k);
        orientation_20 = dm_mm_fmadd_ps(new_rot_j, new_rot_r, orientation_20);
        orientation_20 = dm_mm_mul_ps(twos, orientation_20);
        
        orientation_21 = dm_mm_mul_ps(new_rot_j, new_rot_k);
        orientation_21 = dm_mm_sub_ps(orientation_21, dm_mm_mul_ps(new_rot_i, new_rot_r));
        orientation_21 = dm_mm_mul_ps(twos, orientation_21);
        
        orientation_22 = dm_mm_mul_ps(new_rot_i, new_rot_i);
        dm_mm_fmadd_ps(new_rot_j, new_rot_j, orientation_22);
        dm_mm_mul_ps(twos, orientation_22);
        dm_mm_sub_ps(ones, orientation_22);
        
        // orientation is transposed here
        body_inv_00 = dm_mm_mul_ps(orientation_00, i_inv_00);
        body_inv_01 = dm_mm_mul_ps(orientation_10, i_inv_11);
        body_inv_02 = dm_mm_mul_ps(orientation_20, i_inv_22);
        
        body_inv_10 = dm_mm_mul_ps(orientation_01, i_inv_00);
        body_inv_11 = dm_mm_mul_ps(orientation_11, i_inv_11);
        body_inv_12 = dm_mm_mul_ps(orientation_21, i_inv_22);
        
        body_inv_20 = dm_mm_mul_ps(orientation_02, i_inv_00);
        body_inv_21 = dm_mm_mul_ps(orientation_12, i_inv_11);
        body_inv_22 = dm_mm_mul_ps(orientation_22, i_inv_22);
        
        // final i_inv matrix
        i_inv_00 = dm_mm_mul_ps(orientation_00, body_inv_00);
        i_inv_00 = dm_mm_fmadd_ps(orientation_01, body_inv_10, i_inv_00);
        i_inv_00 = dm_mm_fmadd_ps(orientation_02, body_inv_20, i_inv_00);
        
        i_inv_01 = dm_mm_mul_ps(orientation_00, body_inv_01);
        i_inv_01 = dm_mm_fmadd_ps(orientation_01, body_inv_11, i_inv_01);
        i_inv_01 = dm_mm_fmadd_ps(orientation_02, body_inv_21, i_inv_01);;
        
        i_inv_02 = dm_mm_mul_ps(orientation_00, body_inv_02);
        i_inv_02 = dm_mm_fmadd_ps(orientation_01, body_inv_12, i_inv_02);
        i_inv_02 = dm_mm_fmadd_ps(orientation_02, body_inv_22, i_inv_02);
        
        i_inv_10 = dm_mm_mul_ps(orientation_10, body_inv_00);
        i_inv_10 = dm_mm_fmadd_ps(orientation_11, body_inv_10, i_inv_10);
        i_inv_10 = dm_mm_fmadd_ps(orientation_12, body_inv_20, i_inv_10);
        
        i_inv_11 = dm_mm_mul_ps(orientation_10, body_inv_01);
        i_inv_11 = dm_mm_fmadd_ps(orientation_11, body_inv_11, i_inv_11);
        i_inv_11 = dm_mm_fmadd_ps(orientation_12, body_inv_21, i_inv_11);
        
        i_inv_12 = dm_mm_mul_ps(orientation_10, body_inv_02);
        i_inv_12 = dm_mm_fmadd_ps(orientation_11, body_inv_12, i_inv_12);
        i_inv_12 = dm_mm_fmadd_ps(orientation_12, body_inv_22, i_inv_12);
        
        i_inv_20 = dm_mm_mul_ps(orientation_20, body_inv_00);
        i_inv_20 = dm_mm_fmadd_ps(orientation_21, body_inv_10, i_inv_20);
        i_inv_20 = dm_mm_fmadd_ps(orientation_22, body_inv_20, i_inv_20);
        
        i_inv_21 = dm_mm_mul_ps(orientation_20, body_inv_01);
        i_inv_21 = dm_mm_fmadd_ps(orientation_21, body_inv_11, i_inv_21);
        i_inv_21 = dm_mm_fmadd_ps(orientation_22, body_inv_21, i_inv_21);
        
        i_inv_22 = dm_mm_mul_ps(orientation_20, body_inv_02);
        i_inv_22 = dm_mm_fmadd_ps(orientation_21, body_inv_12, i_inv_22);
        i_inv_22 = dm_mm_fmadd_ps(orientation_22, body_inv_22, i_inv_22);
        
        // store
        dm_mm_store_ps(pos_x_d + i, pos_x);
        dm_mm_store_ps(pos_y_d + i, pos_y);
        dm_mm_store_ps(pos_z_d + i, pos_z);
        
        dm_mm_store_ps(rot_i_d + i, rot_i);
        dm_mm_store_ps(rot_j_d + i, rot_j);
        dm_mm_store_ps(rot_k_d + i, rot_k);
        dm_mm_store_ps(rot_r_d + i, rot_r);
        
        dm_mm_store_ps(vel_x_d + i, vel_x);
        dm_mm_store_ps(vel_y_d + i, vel_y);
        dm_mm_store_ps(vel_z_d + i, vel_z);
        
        dm_mm_store_ps(w_x_d + i, w_x);
        dm_mm_store_ps(w_y_d + i, w_y);
        dm_mm_store_ps(w_z_d + i, w_z);
        
        dm_mm_store_ps(l_x_d + i, l_x);
        dm_mm_store_ps(l_y_d + i, l_y);
        dm_mm_store_ps(l_z_d + i, l_z);
        
        dm_mm_store_ps(i_inv_00_d + i, i_inv_00);
        dm_mm_store_ps(i_inv_01_d + i, i_inv_01);
        dm_mm_store_ps(i_inv_02_d + i, i_inv_02);
        
        dm_mm_store_ps(i_inv_10_d + i, i_inv_10);
        dm_mm_store_ps(i_inv_11_d + i, i_inv_11);
        dm_mm_store_ps(i_inv_12_d + i, i_inv_12);
        
        dm_mm_store_ps(i_inv_20_d + i, i_inv_20);
        dm_mm_store_ps(i_inv_21_d + i, i_inv_21);
        dm_mm_store_ps(i_inv_22_d + i, i_inv_22);
    }
    
    // assign
    i=0;
    for(;i<system->entity_count; i++)
    {
        t_index = system->entity_indices[i][t_id];
        p_index = system->entity_indices[i][p_id];
        r_index = system->entity_indices[i][r_id];
        
        transform->pos_x[t_index] = pos_x_d[i];
        transform->pos_y[t_index] = pos_y_d[i];
        transform->pos_z[t_index] = pos_z_d[i];
        
        transform->rot_i[t_index] = rot_i_d[i];
        transform->rot_j[t_index] = rot_j_d[i];
        transform->rot_k[t_index] = rot_k_d[i];
        transform->rot_r[t_index] = rot_r_d[i];
        
        physics->vel_x[p_index] = vel_x_d[i];
        physics->vel_y[p_index] = vel_y_d[i];
        physics->vel_z[p_index] = vel_z_d[i];
        
        physics->w_x[p_index] = w_x_d[i];
        physics->w_y[p_index] = w_y_d[i];
        physics->w_z[p_index] = w_z_d[i];
        
        physics->l_x[p_index] = l_x_d[i];
        physics->l_y[p_index] = l_y_d[i];
        physics->l_z[p_index] = l_z_d[i];
        
        rigid_body->i_inv_00[r_index] = i_inv_00_d[i];
        rigid_body->i_inv_01[r_index] = i_inv_01_d[i];
        rigid_body->i_inv_02[r_index] = i_inv_02_d[i];
        
        rigid_body->i_inv_10[r_index] = i_inv_10_d[i];
        rigid_body->i_inv_11[r_index] = i_inv_11_d[i];
        rigid_body->i_inv_12[r_index] = i_inv_12_d[i];
        
        rigid_body->i_inv_20[r_index] = i_inv_20_d[i];
        rigid_body->i_inv_21[r_index] = i_inv_21_d[i];
        rigid_body->i_inv_22[r_index] = i_inv_22_d[i];
    }
}