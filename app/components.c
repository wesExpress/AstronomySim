#include "components.h"
#include "app.h"
#include "dm.h"

void entity_add_transform(dm_entity entity, dm_ecs_id t_id, float pos_x,float pos_y,float pos_z, float scale_x,float scale_y,float scale_z, float rot_i,float rot_j,float rot_k,float rot_r, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add transform to invalid entity"); return; }
    
    uint32_t index;
    component_transform_block* transform_block = dm_ecs_get_component_block(t_id, context);
    dm_ecs_get_component_count(t_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    transform_block->pos_x[index] = pos_x;
    transform_block->pos_y[index] = pos_y;
    transform_block->pos_z[index] = pos_z;
    
    transform_block->scale_x[index] = scale_x;
    transform_block->scale_y[index] = scale_y;
    transform_block->scale_z[index] = scale_z;
    
    transform_block->rot_i[index] = rot_i;
    transform_block->rot_j[index] = rot_j;
    transform_block->rot_k[index] = rot_k;
    transform_block->rot_r[index] = rot_r;
    
    dm_ecs_iterate_component_block(t_id, context);
}

void entity_add_kinematics(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_physics_block* physics_block = dm_ecs_get_component_block(p_id, context);
    dm_ecs_get_component_count(p_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    physics_block->vel_x[index] = vel_x;
    physics_block->vel_y[index] = vel_y;
    physics_block->vel_z[index] = vel_z;
    
    physics_block->mass[index]     = mass;
    physics_block->inv_mass[index] = 1.0f / mass;
    
    physics_block->damping_v[index] = damping_v;
    physics_block->damping_w[index] = damping_w;
    
    // default for now
    physics_block->movement_type[index] = PHYSICS_MOVEMENT_KINEMATIC;
    
    dm_ecs_iterate_component_block(p_id, context);
}

void entity_add_collider_box(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float scale_x,float scale_y,float scale_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision_block* collision_block = dm_ecs_get_component_block(c_id, context);
    dm_ecs_get_component_count(c_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    scale_x *= 0.5f;
    scale_y *= 0.5f;
    scale_z *= 0.5f;
    
    float min_x = center_x - scale_x;
    float min_y = center_y - scale_y;
    float min_z = center_z - scale_z;
    
    float max_x = center_x + scale_x;
    float max_y = center_y + scale_y;
    float max_z = center_z + scale_z;
    
    collision_block->aabb_local_min_x[index] = min_x;
    collision_block->aabb_local_min_y[index] = min_y;
    collision_block->aabb_local_min_z[index] = min_z;
    
    collision_block->aabb_local_max_x[index] = max_x;
    collision_block->aabb_local_max_y[index] = max_y;
    collision_block->aabb_local_max_z[index] = max_z;
    
    collision_block->aabb_global_min_x[index] = min_x;
    collision_block->aabb_global_min_y[index] = min_y;
    collision_block->aabb_global_min_z[index] = min_z;
    
    collision_block->aabb_global_max_x[index] = max_x;
    collision_block->aabb_global_max_y[index] = max_y;
    collision_block->aabb_global_max_z[index] = max_z;
    
    collision_block->center_x[index] = center_x;
    collision_block->center_y[index] = center_y;
    collision_block->center_z[index] = center_z;
    
    collision_block->internal_0[index] = min_x;
    collision_block->internal_1[index] = min_y;
    collision_block->internal_2[index] = min_z;
    collision_block->internal_3[index] = max_x;
    collision_block->internal_4[index] = max_y;
    collision_block->internal_5[index] = max_z;
    
    collision_block->shape[index] = DM_COLLISION_SHAPE_BOX;
    collision_block->flag[index]  = COLLISION_FLAG_NO;
    
    dm_ecs_iterate_component_block(c_id, context);
}

void entity_add_collider_sphere(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision_block* collision_block = dm_ecs_get_component_block(c_id, context);
    dm_ecs_get_component_count(c_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    float min_x = center_x - radius;
    float min_y = center_y - radius;
    float min_z = center_z - radius;
    
    float max_x = center_x + radius;
    float max_y = center_y + radius;
    float max_z = center_z + radius;
    
    collision_block->aabb_local_min_x[index] = min_x;
    collision_block->aabb_local_min_y[index] = min_y;
    collision_block->aabb_local_min_z[index] = min_z;
    
    collision_block->aabb_local_max_x[index] = max_x;
    collision_block->aabb_local_max_y[index] = max_y;
    collision_block->aabb_local_max_z[index] = max_z;
    
    collision_block->aabb_global_min_x[index] = min_x;
    collision_block->aabb_global_min_y[index] = min_y;
    collision_block->aabb_global_min_z[index] = min_z;
    
    collision_block->aabb_global_max_x[index] = max_x;
    collision_block->aabb_global_max_y[index] = max_y;
    collision_block->aabb_global_max_z[index] = max_z;
    
    collision_block->center_x[index] = center_x;
    collision_block->center_y[index] = center_y;
    collision_block->center_z[index] = center_z;
    
    collision_block->internal_0[index] = radius;
    
    collision_block->shape[index] = DM_COLLISION_SHAPE_SPHERE;
    collision_block->flag[index]  = COLLISION_FLAG_NO;
    
    dm_ecs_iterate_component_block(c_id, context);
}

void entity_add_rigid_body_box(dm_entity entity, dm_ecs_id r_id, float mass, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_rigid_body_block* rigid_body_block = dm_ecs_get_component_block(r_id, context);
    dm_ecs_get_component_count(r_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    float i_body_00, i_body_11, i_body_22;
    
    float dim_x = max_x - min_x;
    float dim_y = max_y - min_y;
    float dim_z = max_z - min_z;
    
    dim_x *= dim_x;
    dim_y *= dim_y;
    dim_z *= dim_z;
    
    i_body_00 = (dim_y + dim_z) * mass * DM_MATH_INV_12;
    i_body_11 = (dim_x + dim_z) * mass * DM_MATH_INV_12;
    i_body_22 = (dim_x + dim_y) * mass * DM_MATH_INV_12;
    
    rigid_body_block->i_body_00[index] = i_body_00;
    rigid_body_block->i_body_11[index] = i_body_11;
    rigid_body_block->i_body_22[index] = i_body_22;
    
    rigid_body_block->i_body_inv_00[index] = 1.0f / i_body_00;
    rigid_body_block->i_body_inv_11[index] = 1.0f / i_body_11;
    rigid_body_block->i_body_inv_22[index] = 1.0f / i_body_22;
    
    rigid_body_block->i_inv_00[index] = rigid_body_block->i_body_inv_00[index];
    rigid_body_block->i_inv_11[index] = rigid_body_block->i_body_inv_11[index];
    rigid_body_block->i_inv_22[index] = rigid_body_block->i_body_inv_22[index];
    
    dm_ecs_iterate_component_block(r_id, context);
}

void entity_add_rigid_body_sphere(dm_entity entity, dm_ecs_id r_id, float mass, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_rigid_body_block* rigid_body_block = dm_ecs_get_component_block(r_id, context);
    dm_ecs_get_component_count(r_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    const float scalar = 2.0f * 0.2f * mass * radius * radius;
    const float inv_s = 1.0f / scalar;
    
    rigid_body_block->i_body_00[index] = scalar;
    rigid_body_block->i_body_11[index] = scalar;
    rigid_body_block->i_body_22[index] = scalar;
    
    rigid_body_block->i_body_inv_00[index] = inv_s;
    rigid_body_block->i_body_inv_11[index] = inv_s;
    rigid_body_block->i_body_inv_22[index] = inv_s;
    
    rigid_body_block->i_inv_00[index] = rigid_body_block->i_body_inv_00[index];
    rigid_body_block->i_inv_11[index] = rigid_body_block->i_body_inv_11[index];
    rigid_body_block->i_inv_22[index] = rigid_body_block->i_body_inv_22[index];
    
    dm_ecs_iterate_component_block(r_id, context);
}

/************
HELPER FUNCS
**************/
void entity_add_velocity(dm_entity entity, dm_ecs_id p_id, float v_x, float v_y, float v_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics_block* physics_block = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t block_index  = context->ecs_manager.entity_block_indices[entity_index][p_id];
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    (physics_block + block_index)->vel_x[comp_index] += v_x;
    (physics_block + block_index)->vel_y[comp_index] += v_y;
    (physics_block + block_index)->vel_z[comp_index] += v_z;
}

void entity_add_angular_velocity(dm_entity entity, dm_ecs_id p_id, float w_x, float w_y, float w_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics_block* physics_block = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t block_index  = context->ecs_manager.entity_block_indices[entity_index][p_id];
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    (physics_block + block_index)->w_x[comp_index] += w_x;
    (physics_block + block_index)->w_y[comp_index] += w_y;
    (physics_block + block_index)->w_z[comp_index] += w_z;
}

void entity_add_force(dm_entity entity, dm_ecs_id p_id, float f_x, float f_y, float f_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics_block* physics_block = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t block_index  = context->ecs_manager.entity_block_indices[entity_index][p_id];
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    (physics_block + block_index)->force_x[comp_index] += f_x;
    (physics_block + block_index)->force_y[comp_index] += f_y;
    (physics_block + block_index)->force_z[comp_index] += f_z;
}

/**********
REGISTRING
************/
bool register_transform(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_transform_block), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register transform component"); 
    return false;
}

bool register_physics(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_physics_block), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register physics component"); 
    return false;
}

bool register_collision(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_collision_block), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register collision component"); 
    return false;
}

bool register_rigid_body(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_rigid_body_block), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register rigidbody component"); 
    return false;
}