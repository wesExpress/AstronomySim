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
    dm_ecs_system_entity_container entity_a, entity_b;
} physics_system_collision_pair;

typedef enum physics_system_flag_t
{
    DM_PHYSICS_FLAG_PAUSED,
    DM_PHYSICS_FLAG_UNKNOWN
} physics_system_flag;

typedef struct dm_system_physics_broadphase_sort_data_t
{
    uint32_t index;
    size_t   block_size;
    float*   min;
} physics_system_broadphase_sort_data;

#define PHYSICS_SYSTEM_DEFAULT_COLLISION_CAPACITY 16
#define PHYSICS_SYSTEM_DEFAULT_MANIFOLD_CAPACITY  16
#define PHYSICS_SYSTEM_LOAD_FACTOR                0.75f
#define PHYSICS_SYSTEM_RESIZE_FACTOR              2
typedef struct physics_system_manager_t
{
    double          accum_time, simulation_time;
    physics_system_flag flag;
    
    uint32_t broadphase_checks;
    uint32_t num_possible_collisions, collision_capacity;
    uint32_t num_manifolds, manifold_capacity;
    
    dm_ecs_id transform, collision, physics;
    uint32_t  copied_entity_count;
    
    physics_system_collision_pair*   possible_collisions;
    dm_contact_manifold* manifolds;
    
    dm_ecs_system_entity_container* copied_entities;
} physics_system_manager;

bool physics_system_broadphase(dm_ecs_system_manager* system, dm_context* context);
bool physics_system_narrowphase(dm_ecs_system_manager* system, dm_context* context);
void physics_system_solve_constraints(physics_system_manager* manager, dm_context* context);
void physics_system_update_entities(dm_ecs_system_manager* system, dm_context* context);
void physics_system_reset_forces(dm_ecs_system_manager* system, dm_context* context);

/************
SYSTEM FUNCS
**************/
bool physics_system_init(dm_ecs_id t_id, dm_ecs_id c_id, dm_ecs_id p_id, dm_context* context)
{
    dm_ecs_id comps[] = { t_id, c_id, p_id, };
    
    dm_ecs_system_timing timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    dm_ecs_id id = dm_ecs_register_system(comps, DM_ARRAY_LEN(comps), timing, physics_system_run, physics_system_shutdown, context);
    
    dm_ecs_system_manager* system = &context->ecs_manager.systems[timing][id];
    system->system_data = dm_alloc(sizeof(physics_system_manager));
    
    physics_system_manager* manager = system->system_data;
    
    manager->collision_capacity  = PHYSICS_SYSTEM_DEFAULT_COLLISION_CAPACITY;
    manager->manifold_capacity   = PHYSICS_SYSTEM_DEFAULT_MANIFOLD_CAPACITY;
    manager->possible_collisions = dm_alloc(sizeof(physics_system_collision_pair) * manager->collision_capacity);
    manager->manifolds           = dm_alloc(sizeof(dm_contact_manifold) * manager->manifold_capacity);
    
    manager->transform = t_id;
    manager->collision = c_id;
    manager->physics   = p_id;
    
    manager->copied_entity_count = 0;
    
    return true;
}

void physics_system_shutdown(void* s, void* c)
{
    dm_ecs_system_manager* system = s;   
    physics_system_manager* manager = system->system_data;
    
    if(manager->copied_entities) dm_free(manager->copied_entities);
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
    
    dm_timer_start(&full, context);
    while(manager->accum_time >= DM_PHYSICS_FIXED_DT)
    {
        iters++;
        
        // broadphase
        dm_timer_start(&t, context);
        if(!physics_system_broadphase(system, context)) return false;
        broad_time += dm_timer_elapsed_ms(&t, context);
        
        // narrowphase
        dm_timer_start(&t, context);
        if(!physics_system_narrowphase(system, context)) return false;
        narrow_time += dm_timer_elapsed_ms(&t, context);
        
        // collision resolution
        dm_timer_start(&t, context);
        physics_system_solve_constraints(manager, context);
        collision_time += dm_timer_elapsed_ms(&t, context);
        
        // update
        dm_timer_start(&t, context);
        physics_system_update_entities(system, context);
        update_time += dm_timer_elapsed_ms(&t, context);
        
        manager->accum_time -= DM_PHYSICS_FIXED_DT;
    }
    
    physics_system_reset_forces(system, context);
    total_time = dm_timer_elapsed_ms(&full, context);
    
    float iter_f = (float)iters;
    
    imgui_draw_text_fmt(20,20, 1,1,0,1, context, "Physics broadphase average: %0.3lf ms (%u checks)", broad_time / iter_f, manager->broadphase_checks);
    imgui_draw_text_fmt(20,40, 1,1,0,1, context, "Physics narrowphase average: %0.3lf ms (%u checks)", narrow_time / iter_f, manager->num_possible_collisions);
    imgui_draw_text_fmt(20,60, 1,1,0,1, context, "Physics collision resolution average: %0.3lf ms (%u manifolds)", collision_time / iter_f, manager->num_manifolds);
    imgui_draw_text_fmt(20,80, 1,1,0,1, context, "Updating entities average: %0.3lf ms", update_time / iter_f);
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
    physics_system_broadphase_sort_data* sort_data = c;
    dm_ecs_system_entity_container container_a = *(dm_ecs_system_entity_container*)a;
    dm_ecs_system_entity_container container_b = *(dm_ecs_system_entity_container*)b;
    
    uint32_t c_index = sort_data->index;
    
    uint32_t a_c_index = container_a.component_indices[c_index];
    uint32_t b_c_index = container_b.component_indices[c_index];
    
    uint32_t a_b_index = container_a.block_indices[c_index];
    uint32_t b_b_index = container_b.block_indices[c_index];
    
    size_t a_offset = a_b_index * sort_data->block_size;
    size_t b_offset = b_b_index * sort_data->block_size;
    
    float a_min = ((float*)((char*)sort_data->min + a_offset))[a_c_index];
    float b_min = ((float*)((char*)sort_data->min + b_offset))[b_c_index];
    
    return (a_min > b_min) - (a_min < b_min);
}

void physics_system_update_sphere_aabb(uint32_t t_index, uint32_t c_index, component_transform_block* t_block, component_collision_block* c_block)
{
    c_block->aabb_global_min_x[c_index] = c_block->aabb_local_min_x[c_index] + t_block->pos_x[t_index];
    c_block->aabb_global_min_y[c_index] = c_block->aabb_local_min_y[c_index] + t_block->pos_y[t_index];
    c_block->aabb_global_min_z[c_index] = c_block->aabb_local_min_z[c_index] + t_block->pos_z[t_index];
    
    c_block->aabb_global_max_x[c_index] = c_block->aabb_local_max_x[c_index] + t_block->pos_x[t_index];
    c_block->aabb_global_max_y[c_index] = c_block->aabb_local_max_y[c_index] + t_block->pos_y[t_index];
    c_block->aabb_global_max_z[c_index] = c_block->aabb_local_max_z[c_index] + t_block->pos_z[t_index];
}

void physics_system_update_box_aabb(const uint32_t t_index, const uint32_t c_index, component_transform_block* t_block, component_collision_block* c_block)
{
    float a,b      = 0.0f;
    float quat[N4] = { 0 };
    float rot[M3]  = { 0 };
    
    float world_min[3], world_max[3];
    
    quat[0] = t_block->rot_i[t_index];
    quat[1] = t_block->rot_j[t_index];
    quat[2] = t_block->rot_k[t_index];
    quat[3] = t_block->rot_r[t_index];
    
    dm_mat3_rotate_from_quat(quat, rot);
    
    const float min[] = {
        c_block->aabb_local_min_x[c_index],
        c_block->aabb_local_min_y[c_index],
        c_block->aabb_local_min_z[c_index],
    };
    
    const float max[] = {
        c_block->aabb_local_max_x[c_index],
        c_block->aabb_local_max_y[c_index],
        c_block->aabb_local_max_z[c_index],
    };
    
    // start
    world_min[0] = world_max[0] = t_block->pos_x[t_index];
    world_min[1] = world_max[1] = t_block->pos_y[t_index];
    world_min[2] = world_max[2] = t_block->pos_z[t_index];
    
    for(uint32_t i=0; i<3; i++)
    {
        for(uint32_t j=0; j<3; j++)
        {
            a = rot[j * 3 + i] * min[j];
            b = rot[j * 3 + i] * max[j];
            
            world_min[i] += DM_MIN(a,b);
            world_max[i] += DM_MAX(a,b);
        }
    }
    
    c_block->aabb_global_min_x[c_index] = world_min[0];
    c_block->aabb_global_min_y[c_index] = world_min[1];
    c_block->aabb_global_min_z[c_index] = world_min[2];
    
    c_block->aabb_global_max_x[c_index] = world_max[0];
    c_block->aabb_global_max_y[c_index] = world_max[1];
    c_block->aabb_global_max_z[c_index] = world_max[2];
}

void physics_system_update_world_aabb(uint32_t t_index, uint32_t c_index, component_transform_block* t_block, component_collision_block* c_block)
{
    switch(c_block->shape[c_index])
    {
        case DM_COLLISION_SHAPE_SPHERE: physics_system_update_sphere_aabb(t_index, c_index, t_block, c_block);
        break;
        
        case DM_COLLISION_SHAPE_BOX: physics_system_update_box_aabb(t_index, c_index, t_block, c_block);
        break;
        
        default:
        DM_LOG_ERROR("Unknown collider! Shouldn't be here...");
        break;
    }
}

int physics_system_broadphase_get_variance_axis(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    float center_sum[3]    = { 0 };
    float center_sq_sum[3] = { 0 };
    
    int axis = 0;
    
    const uint32_t t_id = manager->transform;
    const uint32_t c_id = manager->collision;
    
    component_transform_block* master_t_block = dm_ecs_get_component_block(t_id, context);
    component_collision_block* master_c_block = dm_ecs_get_component_block(c_id, context);
    
    component_transform_block* t_block = NULL;
    component_collision_block* c_block = NULL;
    
    uint32_t  t_block_index, t_index;
    uint32_t  c_block_index, c_index;
    
    float center[N3];
    
    for(uint32_t e=0; e<system->entity_count; e++)
    {
        t_index       = system->entity_containers[e].component_indices[t_id];
        t_block_index = system->entity_containers[e].block_indices[t_id];
        
        c_index       = system->entity_containers[e].component_indices[c_id];
        c_block_index = system->entity_containers[e].block_indices[c_id];
        
        t_block = master_t_block + t_block_index;
        c_block = master_c_block + c_block_index;
        
        // reset collision flags
        c_block->flag[c_index] = COLLISION_FLAG_NO;
        
        // update global aabb
        physics_system_update_world_aabb(t_index, c_index, t_block, c_block);
        
        // add to center and center sq vectors
        center[0] = 0.5f * (c_block->aabb_global_max_x[c_index] - c_block->aabb_global_min_x[c_index]);
        center[1] = 0.5f * (c_block->aabb_global_max_y[c_index] - c_block->aabb_global_min_y[c_index]);
        center[2] = 0.5f * (c_block->aabb_global_max_z[c_index] - c_block->aabb_global_min_z[c_index]);
        
        float dum[3];
        dm_vec3_add_vec3(center_sum, center, center_sum);
        dm_vec3_mul_vec3(center_sum, center_sum, dum);
        dm_vec3_add_vec3(center_sq_sum, dum, center_sq_sum);
    }
    
    const float scale = 1.0f / system->entity_count;
    dm_vec3_scale(center_sum, scale, center_sum);
    dm_vec3_scale(center_sq_sum, scale, center_sq_sum);
    
    float var[3];
    dm_vec3_mul_vec3(center_sum, center_sum, center_sum);
    var[0] = center_sq_sum[0] - center_sum[0];
    var[1] = center_sq_sum[1] - center_sum[1];
    var[2] = center_sq_sum[2] - center_sum[2];
    
    float max_var = -FLT_MAX;
    for(uint32_t i=0; i<N3; i++)
    {
        if(var[0] <= max_var) continue;
        
        max_var = var[i];
        axis = i;
    }
    
    return axis;
}

// uses simple sort and sweep based on objects' aabbs
bool physics_system_broadphase(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    manager->broadphase_checks = 0;
    
    if(manager->copied_entity_count==0) 
    {
        manager->copied_entity_count = system->entity_count;
        manager->copied_entities = dm_alloc(sizeof(dm_ecs_system_entity_container) * system->entity_count);
        dm_memcpy(manager->copied_entities, system->entity_containers, sizeof(dm_ecs_system_entity_container) * manager->copied_entity_count);
    }
    else if(manager->copied_entity_count!=system->entity_count)
    {
        manager->copied_entity_count = system->entity_count;
        manager->copied_entities = dm_realloc(manager->copied_entities, sizeof(dm_ecs_system_entity_container) * system->entity_count);
        dm_memcpy(manager->copied_entities, system->entity_containers, sizeof(dm_ecs_system_entity_container) * manager->copied_entity_count);
    }
    
    //manager->collision_capacity = DM_PHYSICS_DEFAULT_COLLISION_CAPACITY;
    manager->num_possible_collisions = 0;
    
    const int axis = physics_system_broadphase_get_variance_axis(system, context);
    
    physics_system_broadphase_sort_data data = { 0 };
    data.index = manager->collision;
    data.block_size = sizeof(component_collision_block);
    
    component_collision_block* master_block = dm_ecs_get_component_block(manager->collision, context);
    
    // sort
    switch(axis)
    {
        case 0:
        data.min = master_block->aabb_global_min_x;
        break;
        
        case 1:
        data.min = master_block->aabb_global_min_y;
        break;
        
        case 2:
        data.min = master_block->aabb_global_min_z;
        break;
    }
    
#ifdef DM_PLATFORM_WIN32
    qsort_s(manager->copied_entities, manager->copied_entity_count, sizeof(dm_ecs_system_entity_container), physics_system_broadphase_sort, &data);
#elif defined(DM_PLATFORM_LINUX)
    qsort_r(manager->copied_entities, manager->copied_entity_count, sizeof(dm_ecs_system_entity_container), physics_system_broadphase_sort, &data);
#elif defined(DM_PLATFORM_APPLE)
    qsort_r(manager->copied_entities, manager->copied_entity_count, sizeof(dm_ecs_system_entity_container), &data, physics_system_broadphase_sort);
#endif
    
    // sweep
    float max_i, min_j;
    dm_ecs_system_entity_container entity_a, entity_b;
    uint32_t a_c_index, b_c_index;
    
    bool x_check, y_check, z_check;
    
    float a_pos[3], a_dim[3];
    float b_pos[3], b_dim[3];
    
    component_collision_block* a_block = NULL;
    component_collision_block* b_block = NULL;
    
    for(uint32_t i=0; i<manager->copied_entity_count; i++)
    {
        //entity_a = system->entity_containers[i];
        entity_a = manager->copied_entities[i];
        
        a_c_index = entity_a.component_indices[data.index];
        
        a_block = master_block + entity_a.block_indices[data.index];
        
        a_dim[0] = 0.5f * dm_fabs(a_block->aabb_global_max_x[a_c_index] - a_block->aabb_global_min_x[a_c_index]);
        a_dim[1] = 0.5f * dm_fabs(a_block->aabb_global_max_y[a_c_index] - a_block->aabb_global_min_y[a_c_index]);
        a_dim[2] = 0.5f * dm_fabs(a_block->aabb_global_max_z[a_c_index] - a_block->aabb_global_min_z[a_c_index]);
        
        a_pos[0] = a_block->aabb_global_max_x[a_c_index] - a_dim[0];
        a_pos[1] = a_block->aabb_global_max_y[a_c_index] - a_dim[1];
        a_pos[2] = a_block->aabb_global_max_z[a_c_index] - a_dim[2];
        
        switch(axis)
        {
            case 0: max_i = a_block->aabb_global_max_x[a_c_index];
            break;
            
            case 1: max_i = a_block->aabb_global_max_y[a_c_index];
            break;
            
            case 2: max_i = a_block->aabb_global_max_z[a_c_index];
            break;
            
            default:
            DM_LOG_FATAL("Shouldn't be here in broadphase");
            return false;
        }
        
        for(uint32_t j=i+1; j<system->entity_count; j++)
        {
            //entity_b = system->entity_containers[j];
            entity_b = manager->copied_entities[j];
            
            b_c_index = entity_b.component_indices[data.index];
            
            b_block = master_block + entity_b.block_indices[data.index];
            
            b_dim[0] = 0.5f * dm_fabs(b_block->aabb_global_max_x[b_c_index] - b_block->aabb_global_min_x[b_c_index]);
            b_dim[1] = 0.5f * dm_fabs(b_block->aabb_global_max_y[b_c_index] - b_block->aabb_global_min_y[b_c_index]);
            b_dim[2] = 0.5f * dm_fabs(b_block->aabb_global_max_z[b_c_index] - b_block->aabb_global_min_z[b_c_index]);
            
            b_pos[0] = b_block->aabb_global_max_x[b_c_index] - b_dim[0];
            b_pos[1] = b_block->aabb_global_max_y[b_c_index] - b_dim[1];
            b_pos[2] = b_block->aabb_global_max_z[b_c_index] - b_dim[2];
            
            switch(axis)
            {
                case 0: min_j = b_block->aabb_global_min_x[b_c_index];
                break;
                
                case 1: min_j = b_block->aabb_global_min_y[b_c_index];
                break;
                
                case 2: min_j = b_block->aabb_global_min_z[b_c_index];
                break;
                
                default:
                DM_LOG_FATAL("Shouldn't be here in broadphase");
                return false;
            }
            
            if(min_j > max_i) break;
            
            manager->broadphase_checks++;
            
            x_check = dm_fabs(a_pos[0] - b_pos[0]) <= (a_dim[0] + b_dim[0]);
            y_check = dm_fabs(a_pos[1] - b_pos[1]) <= (a_dim[1] + b_dim[1]);
            z_check = dm_fabs(a_pos[2] - b_pos[2]) <= (a_dim[2] + b_dim[2]);
            
            if(!x_check || !y_check || !z_check) continue;
            
            a_block->flag[a_c_index] = COLLISION_FLAG_POSSIBLE;
            b_block->flag[b_c_index] = COLLISION_FLAG_POSSIBLE;
            
            manager->possible_collisions[manager->num_possible_collisions].entity_a = entity_a;
            manager->possible_collisions[manager->num_possible_collisions].entity_b = entity_b;
            manager->num_possible_collisions++;
            
            dm_grow_dyn_array((void**)&manager->possible_collisions, manager->num_possible_collisions, &manager->collision_capacity, sizeof(physics_system_collision_pair), PHYSICS_SYSTEM_LOAD_FACTOR, PHYSICS_SYSTEM_RESIZE_FACTOR);
        }
    }
    
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
    
    const dm_ecs_id t_id = manager->transform;
    const dm_ecs_id c_id = manager->collision;
    const dm_ecs_id p_id = manager->physics;
    
    component_transform_block* t_block = dm_ecs_get_component_block(t_id, context);
    component_collision_block* c_block = dm_ecs_get_component_block(c_id, context);
    component_physics_block*   p_block = dm_ecs_get_component_block(p_id, context);
    
    float              pos[2][3], rots[2][4], cens[2][3], internals[2][6], vels[2][3], ws[2][3];
    dm_collision_shape shapes[2];
    
    uint32_t a_t_c_index, a_c_c_index, a_p_c_index;
    uint32_t b_t_c_index, b_c_c_index, b_p_c_index;
    
    component_transform_block* a_t_block, *b_t_block;
    component_collision_block* a_c_block, *b_c_block;
    component_physics_block*   a_p_block, *b_p_block;
    
    physics_system_collision_pair  collision_pair;
    dm_ecs_system_entity_container entity_a, entity_b;
    
    dm_contact_manifold* manifold;
    
    dm_simplex simplex;
    
    for(uint32_t i=0; i<manager->num_possible_collisions; i++)
    {
        collision_pair = manager->possible_collisions[i];
        
        entity_a = collision_pair.entity_a;
        entity_b = collision_pair.entity_b;
        
        // get all data
        
        // entity a
        a_t_c_index = entity_a.component_indices[t_id];
        a_c_c_index = entity_a.component_indices[c_id];
        a_p_c_index = entity_a.component_indices[p_id];
        
        a_t_block = t_block + entity_a.block_indices[t_id];;
        a_c_block = c_block + entity_a.block_indices[c_id];;
        a_p_block = p_block + entity_a.block_indices[p_id];;
        
        pos[0][0]       = a_t_block->pos_x[a_t_c_index];
        pos[0][1]       = a_t_block->pos_y[a_t_c_index];
        pos[0][2]       = a_t_block->pos_z[a_t_c_index];
        rots[0][0]      = a_t_block->rot_i[a_t_c_index];
        rots[0][1]      = a_t_block->rot_j[a_t_c_index];
        rots[0][2]      = a_t_block->rot_k[a_t_c_index];
        rots[0][3]      = a_t_block->rot_r[a_t_c_index];
        cens[0][0]      = a_c_block->center_x[a_c_c_index];
        cens[0][1]      = a_c_block->center_y[a_c_c_index];
        cens[0][2]      = a_c_block->center_z[a_c_c_index];
        internals[0][0] = a_c_block->internal_0[a_c_c_index];
        internals[0][1] = a_c_block->internal_1[a_c_c_index];
        internals[0][2] = a_c_block->internal_2[a_c_c_index];
        internals[0][3] = a_c_block->internal_3[a_c_c_index];
        internals[0][4] = a_c_block->internal_4[a_c_c_index];
        internals[0][5] = a_c_block->internal_5[a_c_c_index];
        shapes[0]       = a_c_block->shape[a_c_c_index];
        vels[0][0]      = a_p_block->vel_x[a_p_c_index];
        vels[0][1]      = a_p_block->vel_y[a_p_c_index];
        vels[0][2]      = a_p_block->vel_z[a_p_c_index];
        ws[0][0]        = a_p_block->w_x[a_p_c_index];
        ws[0][1]        = a_p_block->w_y[a_p_c_index];
        ws[0][2]        = a_p_block->w_z[a_p_c_index];
        
        // entity b
        b_t_c_index = entity_b.component_indices[t_id];
        b_c_c_index = entity_b.component_indices[c_id];
        b_p_c_index = entity_b.component_indices[p_id];
        
        b_t_block = t_block + entity_b.block_indices[t_id];;
        b_c_block = c_block + entity_b.block_indices[c_id];;
        b_p_block = p_block + entity_b.block_indices[p_id];;
        
        pos[1][0]       = b_t_block->pos_x[b_t_c_index];
        pos[1][1]       = b_t_block->pos_y[b_t_c_index];
        pos[1][2]       = b_t_block->pos_z[b_t_c_index];
        rots[1][0]      = b_t_block->rot_i[b_t_c_index];
        rots[1][1]      = b_t_block->rot_j[b_t_c_index];
        rots[1][2]      = b_t_block->rot_k[b_t_c_index];
        rots[1][3]      = b_t_block->rot_r[b_t_c_index];
        cens[1][0]      = b_c_block->center_x[b_c_c_index];
        cens[1][1]      = b_c_block->center_y[b_c_c_index];
        cens[1][2]      = b_c_block->center_z[b_c_c_index];
        internals[1][0] = b_c_block->internal_0[b_c_c_index];
        internals[1][1] = b_c_block->internal_1[b_c_c_index];
        internals[1][2] = b_c_block->internal_2[b_c_c_index];
        internals[1][3] = b_c_block->internal_3[b_c_c_index];
        internals[1][4] = b_c_block->internal_4[b_c_c_index];
        internals[1][5] = b_c_block->internal_5[b_c_c_index];
        shapes[1]       = b_c_block->shape[b_c_c_index];
        vels[1][0]      = b_p_block->vel_x[b_p_c_index];
        vels[1][1]      = b_p_block->vel_y[b_p_c_index];
        vels[1][2]      = b_p_block->vel_z[b_p_c_index];
        ws[1][0]        = b_p_block->w_x[b_p_c_index];
        ws[1][1]        = b_p_block->w_y[b_p_c_index];
        ws[1][2]        = b_p_block->w_z[b_p_c_index];
        
        //////
        simplex = (dm_simplex){ 0 };
        
        float supports[2][3];
        if(!dm_physics_gjk(pos, rots, cens, internals, shapes, supports, &simplex)) continue;
        
        assert(simplex.size==4);
        
        manifold = &manager->manifolds[manager->num_manifolds++];
        *manifold = (dm_contact_manifold){ 0 };
        
        a_c_block->flag[a_c_c_index] = COLLISION_FLAG_YES;
        
        manifold->contact_data[0].vel_x         = &a_p_block->vel_x[a_p_c_index];
        manifold->contact_data[0].vel_y         = &a_p_block->vel_y[a_p_c_index];
        manifold->contact_data[0].vel_z         = &a_p_block->vel_z[a_p_c_index];
        manifold->contact_data[0].w_x           = &a_p_block->w_x[a_p_c_index];
        manifold->contact_data[0].w_y           = &a_p_block->w_y[a_p_c_index];
        manifold->contact_data[0].w_z           = &a_p_block->w_z[a_p_c_index];
        manifold->contact_data[0].mass          = a_p_block->mass[a_p_c_index];
        manifold->contact_data[0].inv_mass      = a_p_block->inv_mass[a_p_c_index];
        manifold->contact_data[0].v_damp        = a_p_block->damping_v[a_p_c_index];
        manifold->contact_data[0].w_damp        = a_p_block->damping_w[a_p_c_index];
        manifold->contact_data[0].i_body_inv_00 = a_p_block->i_body_inv_00[a_p_c_index];
        manifold->contact_data[0].i_body_inv_11 = a_p_block->i_body_inv_11[a_p_c_index];
        manifold->contact_data[0].i_body_inv_22 = a_p_block->i_body_inv_22[a_p_c_index];
        
        b_c_block->flag[b_c_c_index] = COLLISION_FLAG_YES;
        
        manifold->contact_data[1].vel_x         = &b_p_block->vel_x[b_p_c_index];
        manifold->contact_data[1].vel_y         = &b_p_block->vel_y[b_p_c_index];
        manifold->contact_data[1].vel_z         = &b_p_block->vel_z[b_p_c_index];
        manifold->contact_data[1].w_x           = &b_p_block->w_x[b_p_c_index];
        manifold->contact_data[1].w_y           = &b_p_block->w_y[b_p_c_index];
        manifold->contact_data[1].w_z           = &b_p_block->w_z[b_p_c_index];
        manifold->contact_data[1].mass          = b_p_block->mass[b_p_c_index];
        manifold->contact_data[1].inv_mass      = b_p_block->inv_mass[b_p_c_index];
        manifold->contact_data[1].v_damp        = b_p_block->damping_v[b_p_c_index];
        manifold->contact_data[1].w_damp        = b_p_block->damping_w[b_p_c_index];
        manifold->contact_data[1].i_body_inv_00 = b_p_block->i_body_inv_00[b_p_c_index];
        manifold->contact_data[1].i_body_inv_11 = b_p_block->i_body_inv_11[b_p_c_index];
        manifold->contact_data[1].i_body_inv_22 = b_p_block->i_body_inv_22[b_p_c_index];
        
        if(!dm_physics_collide_entities(pos, rots, cens, internals, vels, ws, shapes, &simplex, manifold)) return false;
        
        // resize manifolds
        dm_grow_dyn_array((void**)&manager->manifolds, manager->num_manifolds, &manager->manifold_capacity, sizeof(dm_contact_manifold), PHYSICS_SYSTEM_LOAD_FACTOR, PHYSICS_SYSTEM_RESIZE_FACTOR);
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
    
    const uint32_t t_id = manager->transform;
    const uint32_t p_id = manager->physics;
    
    component_transform_block* master_t_block = dm_ecs_get_component_block(t_id, context);
    component_physics_block*   master_p_block = dm_ecs_get_component_block(p_id, context);
    
    uint32_t t_index, p_index;
    
    component_transform_block* t_block;
    component_physics_block*   p_block;
    
    float dt_mass;
    
    float w_x, w_y, w_z;
    float rot_i, rot_j, rot_k, rot_r;
    
    float new_rot_i, new_rot_j, new_rot_k, new_rot_r;
    float new_rot_mag;
    
    float orientation_00, orientation_01, orientation_02;
    float orientation_10, orientation_11, orientation_12;
    float orientation_20, orientation_21, orientation_22;
    
    float body_inv_00, body_inv_01, body_inv_02;
    float body_inv_10, body_inv_11, body_inv_12;
    float body_inv_20, body_inv_21, body_inv_22;
    
    float i_inv_00, i_inv_01, i_inv_02;
    float i_inv_10, i_inv_11, i_inv_12;
    float i_inv_20, i_inv_21, i_inv_22;
    
    const static float half_dt = 0.5f * DM_PHYSICS_FIXED_DT;
    
    dm_ecs_system_entity_container entity;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        entity = system->entity_containers[i];
        
        t_block = master_t_block + entity.block_indices[t_id];
        p_block = master_p_block + entity.block_indices[p_id];
        
        t_index = entity.component_indices[t_id];
        p_index = entity.component_indices[p_id];
        
        // integrate position
        t_block->pos_x[t_index] += p_block->vel_x[p_index] * DM_PHYSICS_FIXED_DT;
        t_block->pos_y[t_index] += p_block->vel_y[p_index] * DM_PHYSICS_FIXED_DT;
        t_block->pos_z[t_index] += p_block->vel_z[p_index] * DM_PHYSICS_FIXED_DT;
        
        // integrate velocity
        dt_mass = p_block->mass[p_index] * DM_PHYSICS_FIXED_DT;
        
        p_block->vel_x[p_index] += p_block->force_x[p_index] * dt_mass;
        p_block->vel_y[p_index] += p_block->force_y[p_index] * dt_mass;
        p_block->vel_z[p_index] += p_block->force_z[p_index] * dt_mass;
        
        // integrate angular momentum
        p_block->l_x[p_index] += p_block->torque_x[p_index] * DM_PHYSICS_FIXED_DT;
        p_block->l_y[p_index] += p_block->torque_y[p_index] * DM_PHYSICS_FIXED_DT;
        p_block->l_z[p_index] += p_block->torque_z[p_index] * DM_PHYSICS_FIXED_DT;
        
        // integrate angular velocity
        p_block->w_x[p_index] += p_block->i_inv_00[p_index] * p_block->l_x[p_index];
        p_block->w_x[p_index] += p_block->i_inv_01[p_index] * p_block->l_y[p_index];
        p_block->w_x[p_index] += p_block->i_inv_02[p_index] * p_block->l_z[p_index];
        
        p_block->w_y[p_index] += p_block->i_inv_10[p_index] * p_block->l_x[p_index];
        p_block->w_y[p_index] += p_block->i_inv_11[p_index] * p_block->l_y[p_index];
        p_block->w_y[p_index] += p_block->i_inv_12[p_index] * p_block->l_z[p_index];
        
        p_block->w_z[p_index] += p_block->i_inv_20[p_index] * p_block->l_x[p_index];
        p_block->w_z[p_index] += p_block->i_inv_21[p_index] * p_block->l_y[p_index];
        p_block->w_z[p_index] += p_block->i_inv_22[p_index] * p_block->l_z[p_index];
        
        // integrate rotation
        rot_i = t_block->rot_i[t_index];
        rot_j = t_block->rot_j[t_index];
        rot_k = t_block->rot_k[t_index];
        rot_r = t_block->rot_r[t_index];
        
        w_x = p_block->w_x[p_index];
        w_y = p_block->w_y[p_index];
        w_z = p_block->w_z[p_index];
        
        new_rot_i  = w_x * rot_r;
        new_rot_i += w_y * rot_k;
        new_rot_i -= w_z * rot_j;
        new_rot_i *= half_dt;
        new_rot_i += rot_i;
        
        new_rot_j  = -w_x * rot_k;
        new_rot_j +=  w_y * rot_r;
        new_rot_j +=  w_z * rot_i;
        new_rot_j *= half_dt;
        new_rot_j += rot_j;
        
        new_rot_k  = w_x * rot_j;
        new_rot_k -= w_y * rot_i;
        new_rot_k += w_z * rot_r;
        new_rot_k *= half_dt;
        new_rot_k += rot_k;
        
        new_rot_r  = -w_x * rot_i;
        new_rot_r -=  w_y * rot_j;
        new_rot_r -=  w_z * rot_k;
        new_rot_r *= half_dt;
        new_rot_r += rot_r;
        
        new_rot_mag  = new_rot_i * new_rot_i;
        new_rot_mag += new_rot_j * new_rot_j;
        new_rot_mag += new_rot_k * new_rot_k;
        new_rot_mag += new_rot_r * new_rot_r;
        new_rot_mag  = dm_sqrtf(new_rot_mag);
        
        new_rot_i /= new_rot_mag;
        new_rot_j /= new_rot_mag;
        new_rot_k /= new_rot_mag;
        new_rot_r /= new_rot_mag;
        
        t_block->rot_i[t_index] = new_rot_i;
        t_block->rot_j[t_index] = new_rot_j;
        t_block->rot_k[t_index] = new_rot_k;
        t_block->rot_r[t_index] = new_rot_r;
        
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
        body_inv_00 = orientation_00 * p_block->i_inv_00[p_index];
        body_inv_01 = orientation_10 * p_block->i_inv_11[p_index];
        body_inv_02 = orientation_20 * p_block->i_inv_22[p_index];
        
        body_inv_10 = orientation_01 * p_block->i_inv_00[p_index];
        body_inv_11 = orientation_11 * p_block->i_inv_11[p_index];
        body_inv_12 = orientation_21 * p_block->i_inv_22[p_index];
        
        body_inv_20 = orientation_02 * p_block->i_inv_00[p_index];
        body_inv_21 = orientation_12 * p_block->i_inv_11[p_index];
        body_inv_22 = orientation_22 * p_block->i_inv_22[p_index];
        
        // final i_inv matrix
        i_inv_00  = orientation_00 * body_inv_00;
        i_inv_00 += orientation_01 * body_inv_10;
        i_inv_00 += orientation_02 * body_inv_20;
        
        i_inv_01  = orientation_00 * body_inv_01;
        i_inv_01 += orientation_01 * body_inv_11;
        i_inv_01 += orientation_02 * body_inv_21;
        
        i_inv_02  = orientation_00 * body_inv_02;
        i_inv_02 += orientation_01 * body_inv_12;
        i_inv_02 += orientation_02 * body_inv_22;
        
        i_inv_10  = orientation_10 * body_inv_00;
        i_inv_10 += orientation_11 * body_inv_10;
        i_inv_10 += orientation_12 * body_inv_20;
        
        i_inv_11  = orientation_10 * body_inv_01;
        i_inv_11 += orientation_11 * body_inv_11;
        i_inv_11 += orientation_12 * body_inv_21;
        
        i_inv_12  = orientation_10 * body_inv_02;
        i_inv_12 += orientation_11 * body_inv_12;
        i_inv_12 += orientation_12 * body_inv_22;
        
        i_inv_20  = orientation_20 * body_inv_00;
        i_inv_20 += orientation_21 * body_inv_10;
        i_inv_20 += orientation_22 * body_inv_20;
        
        i_inv_21  = orientation_20 * body_inv_01;
        i_inv_21 += orientation_21 * body_inv_11;
        i_inv_21 += orientation_22 * body_inv_21;
        
        i_inv_22  = orientation_20 * body_inv_02;
        i_inv_22 += orientation_21 * body_inv_12;
        i_inv_22 += orientation_22 * body_inv_22;
        
        p_block->i_inv_00[p_index] = i_inv_00;
        p_block->i_inv_01[p_index] = i_inv_01;
        p_block->i_inv_02[p_index] = i_inv_02;
        
        p_block->i_inv_10[p_index] = i_inv_10;
        p_block->i_inv_11[p_index] = i_inv_11;
        p_block->i_inv_12[p_index] = i_inv_12;
        
        p_block->i_inv_20[p_index] = i_inv_20;
        p_block->i_inv_21[p_index] = i_inv_21;
        p_block->i_inv_22[p_index] = i_inv_22;
    }
}

void physics_system_reset_forces(dm_ecs_system_manager* system, dm_context* context)
{
    physics_system_manager* manager = system->system_data;
    
    const uint32_t p_id = manager->physics;
    component_physics_block* master_p_block = dm_ecs_get_component_block(p_id, context);
    
    uint32_t p_index;
    
    component_physics_block* p_block;
    
    for(uint32_t i=0; i<system->entity_count; i++)
    {
        dm_ecs_system_entity_container entity = system->entity_containers[i];
        
        p_block = master_p_block + entity.block_indices[p_id];
        p_index = entity.component_indices[p_id];
        
        p_block->force_x[p_index] = 0;
        p_block->force_y[p_index] = 0;
        p_block->force_z[p_index] = 0;
        
        p_block->torque_x[p_index] = 0;
        p_block->torque_y[p_index] = 0;
        p_block->torque_z[p_index] = 0;
    }
}