#include "components.h"
#include "dm.h"
#include "app.h"

void entity_add_transform(dm_entity entity, dm_ecs_id t_id, float pos_x,float pos_y,float pos_z, float scale_x,float scale_y,float scale_z, float rot_i,float rot_j,float rot_k,float rot_r, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add transform to invalid entity"); return; }
    
    uint32_t index;
    component_transform_block* transform_block = dm_ecs_get_current_component_block(t_id, &index, context);
    
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
    
    dm_ecs_iterate_component_block(entity, t_id, context);
}

void entity_add_kinematics_box_rigid_body(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, float min_x,float min_y, float min_z, float max_x, float max_y, float max_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    uint32_t index;
    component_physics_block* physics_block = dm_ecs_get_current_component_block(p_id, &index, context);
    
    physics_block->vel_x[index] = vel_x;
    physics_block->vel_y[index] = vel_y;
    physics_block->vel_z[index] = vel_z;
    
    physics_block->mass[index]     = mass;
    physics_block->inv_mass[index] = 1.0f / mass;
    
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
    
    physics_block->i_body_00[index] = i_body_00;
    physics_block->i_body_11[index] = i_body_11;
    physics_block->i_body_22[index] = i_body_22;
    
    physics_block->i_body_inv_00[index] = 1.0f / i_body_00;
    physics_block->i_body_inv_11[index] = 1.0f / i_body_11;
    physics_block->i_body_inv_22[index] = 1.0f / i_body_22;
    
    physics_block->i_inv_00[index] = physics_block->i_body_inv_00[index];
    physics_block->i_inv_11[index] = physics_block->i_body_inv_11[index];
    physics_block->i_inv_22[index] = physics_block->i_body_inv_22[index];
    
    physics_block->damping_v[index] = damping_v;
    physics_block->damping_w[index] = damping_w;
    
    // default for now
    physics_block->body_type[index] = PHYSICS_BODY_TYPE_RIGID;
    physics_block->movement_type[index] = PHYSICS_MOVEMENT_KINEMATIC;
    
    dm_ecs_iterate_component_block(entity, p_id, context);
}

void entity_add_kinematics_sphere_rigid_body(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    uint32_t index;
    component_physics_block* physics_block = dm_ecs_get_current_component_block(p_id, &index, context);
    
    physics_block->vel_x[index] = vel_x;
    physics_block->vel_y[index] = vel_y;
    physics_block->vel_z[index] = vel_z;
    
    physics_block->mass[index]     = mass;
    physics_block->inv_mass[index] = 1.0f / mass;
    
    float i_body_00, i_body_11, i_body_22;
    
    float scalar = 2.0f * 0.2f * mass * radius * radius;
    
    i_body_00 = scalar;
    i_body_11 = scalar;
    i_body_22 = scalar;
    
    physics_block->i_body_00[index] = i_body_00;
    physics_block->i_body_11[index] = i_body_11;
    physics_block->i_body_22[index] = i_body_22;
    
    physics_block->i_body_inv_00[index] = 1.0f / i_body_00;
    physics_block->i_body_inv_11[index] = 1.0f / i_body_11;
    physics_block->i_body_inv_22[index] = 1.0f / i_body_22;
    
    physics_block->i_inv_00[index] = physics_block->i_body_inv_00[index];
    physics_block->i_inv_11[index] = physics_block->i_body_inv_11[index];
    physics_block->i_inv_22[index] = physics_block->i_body_inv_22[index];
    
    physics_block->damping_v[index] = damping_v;
    physics_block->damping_w[index] = damping_w;
    
    // default for now
    physics_block->body_type[index] = PHYSICS_BODY_TYPE_RIGID;
    physics_block->movement_type[index] = PHYSICS_MOVEMENT_KINEMATIC;
    
    dm_ecs_iterate_component_block(entity, p_id, context);
}

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

void entity_add_box_collider(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float scale_x,float scale_y,float scale_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision_block* collision_block = dm_ecs_get_current_component_block(c_id, &index, context);
    
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
    
    dm_ecs_iterate_component_block(entity, c_id, context);
}

void entity_add_sphere_collider(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision_block* collision_block = dm_ecs_get_current_component_block(c_id, &index, context);
    
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
    
    dm_ecs_iterate_component_block(entity, c_id, context);
}

const component_transform entity_get_transform(dm_entity entity, dm_ecs_id t_id, dm_context* context)
{
    uint32_t index;
    component_transform_block* transform_block = dm_ecs_entity_get_component_block(entity, t_id, &index, context);
    component_transform transform = { 0 };
    if(!transform_block) return transform;
    
    transform.pos[0] = transform_block->pos_x[index];
    transform.pos[1] = transform_block->pos_y[index];
    transform.pos[2] = transform_block->pos_z[index];
    
    transform.scale[0] = transform_block->scale_x[index];
    transform.scale[1] = transform_block->scale_y[index];
    transform.scale[2] = transform_block->scale_z[index];
    
    transform.rot[0] = transform_block->rot_i[index];
    transform.rot[1] = transform_block->rot_j[index];
    transform.rot[2] = transform_block->rot_k[index];
    transform.rot[3] = transform_block->rot_r[index];
    
    return transform;
}

const component_collision entity_get_collision(dm_entity entity, dm_ecs_id c_id, dm_context* context)
{
    uint32_t index;
    component_collision_block* collision_block = dm_ecs_entity_get_component_block(entity, c_id, &index, context);
    component_collision collision = { 0 };
    if(!collision_block) return collision;
    
    collision.aabb_local_min[0] = collision_block->aabb_local_min_x[index];
    collision.aabb_local_min[1] = collision_block->aabb_local_min_y[index];
    collision.aabb_local_min[2] = collision_block->aabb_local_min_z[index];
    
    collision.aabb_local_max[0] = collision_block->aabb_local_max_x[index];
    collision.aabb_local_max[1] = collision_block->aabb_local_max_y[index];
    collision.aabb_local_max[2] = collision_block->aabb_local_max_z[index];
    
    collision.aabb_global_min[0] = collision_block->aabb_global_min_x[index];
    collision.aabb_global_min[1] = collision_block->aabb_global_min_y[index];
    collision.aabb_global_min[2] = collision_block->aabb_global_min_z[index];
    
    collision.aabb_global_max[0] = collision_block->aabb_global_max_x[index];
    collision.aabb_global_max[1] = collision_block->aabb_global_max_y[index];
    collision.aabb_global_max[2] = collision_block->aabb_global_max_z[index];
    
    collision.center[0] = collision_block->center_x[index];
    collision.center[1] = collision_block->center_y[index];
    collision.center[2] = collision_block->center_z[index];
    
    collision.internal[0] = collision_block->internal_0[index];
    collision.internal[1] = collision_block->internal_1[index];
    collision.internal[2] = collision_block->internal_2[index];
    collision.internal[3] = collision_block->internal_3[index];
    collision.internal[4] = collision_block->internal_4[index];
    collision.internal[5] = collision_block->internal_5[index];
    
    collision.shape = collision_block->shape[index];
    collision.flag = collision_block->flag[index];
    
    return collision;
}

const component_physics entity_get_physics(dm_entity entity, dm_ecs_id p_id, dm_context* context)
{
    uint32_t index;
    component_physics_block* physics_block = dm_ecs_entity_get_component_block(entity, p_id, &index, context);
    component_physics physics = { 0 };
    if(!physics_block) return physics;
    
    physics.vel[0] = physics_block->vel_x[index];
    physics.vel[1] = physics_block->vel_y[index];
    physics.vel[2] = physics_block->vel_z[index];
    
    physics.w[0] = physics_block->w_x[index];
    physics.w[1] = physics_block->w_y[index];
    physics.w[2] = physics_block->w_z[index];
    
    physics.l[0] = physics_block->l_x[index];
    physics.l[1] = physics_block->l_y[index];
    physics.l[2] = physics_block->l_z[index];
    
    physics.force[0] = physics_block->force_x[index];
    physics.force[1] = physics_block->force_y[index];
    physics.force[2] = physics_block->force_z[index];
    
    physics.torque[0] = physics_block->torque_x[index];
    physics.torque[1] = physics_block->torque_y[index];
    physics.torque[2] = physics_block->torque_z[index];
    
    physics.mass = physics_block->mass[index];
    physics.inv_mass = physics_block->inv_mass[index];
    
    physics.i_body[0] = physics_block->i_body_00[index];
    physics.i_body[1] = physics_block->i_body_11[index];
    physics.i_body[2] = physics_block->i_body_22[index];
    
    physics.i_body_inv[0] = physics_block->i_body_inv_00[index];
    physics.i_body_inv[1] = physics_block->i_body_inv_11[index];
    physics.i_body_inv[2] = physics_block->i_body_inv_22[index];
    
    physics.i_inv_0[0] = physics_block->i_inv_00[index];
    physics.i_inv_0[1] = physics_block->i_inv_01[index];
    physics.i_inv_0[2] = physics_block->i_inv_02[index];
    
    physics.i_inv_1[0] = physics_block->i_inv_10[index];
    physics.i_inv_1[1] = physics_block->i_inv_11[index];
    physics.i_inv_1[2] = physics_block->i_inv_12[index];
    
    physics.i_inv_2[0] = physics_block->i_inv_20[index];
    physics.i_inv_2[1] = physics_block->i_inv_21[index];
    physics.i_inv_2[2] = physics_block->i_inv_22[index];
    
    physics.damping[0] = physics_block->damping_v[index];
    physics.damping[1] = physics_block->damping_w[index];
    
    physics.body_type     = physics_block->body_type[index];
    physics.movement_type = physics_block->movement_type[index];
    
    return physics;
}

/**********
REGISTRING
************/
bool register_components(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    app_data->components.transform = dm_ecs_register_component(sizeof(component_transform_block), context);
    if(app_data->components.transform==DM_ECS_INVALID_ID) { DM_LOG_FATAL("Could not register transform component"); return false; }
    
    app_data->components.collision = dm_ecs_register_component(sizeof(component_collision_block), context);
    if(app_data->components.collision==DM_ECS_INVALID_ID) { DM_LOG_FATAL("Could not register collision component"); return false; }
    
    app_data->components.physics = dm_ecs_register_component(sizeof(component_physics_block), context);
    if(app_data->components.physics==DM_ECS_INVALID_ID) { DM_LOG_FATAL("Could not register physics component"); return false; }
    
    return true;
}