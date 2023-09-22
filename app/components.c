#include "components.h"
#include "app.h"

void entity_add_transform(dm_entity entity, dm_ecs_id t_id, float pos_x,float pos_y,float pos_z, float scale_x,float scale_y,float scale_z, float rot_i,float rot_j,float rot_k,float rot_r, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add transform to invalid entity"); return; }
    
    uint32_t index;
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    dm_ecs_get_component_insert_index(t_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    float s = rot_i * rot_i + rot_j * rot_j + rot_k * rot_k + rot_r * rot_r;
    if(s==0)
    {
        rot_r = 1;
        s = 1;
    }
    else s = 1 / dm_sqrtf(s);
    
    transform->pos_x[index] = pos_x;
    transform->pos_y[index] = pos_y;
    transform->pos_z[index] = pos_z;
    
    transform->scale_x[index] = scale_x;
    transform->scale_y[index] = scale_y;
    transform->scale_z[index] = scale_z;
    
    transform->rot_i[index] = rot_i * s;
    transform->rot_j[index] = rot_j * s;
    transform->rot_k[index] = rot_k * s;
    transform->rot_r[index] = rot_r * s;
    
    dm_ecs_entity_add_component(entity, t_id, context);
}

void entity_add_kinematics(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_physics* physics = dm_ecs_get_component_block(p_id, context);
    dm_ecs_get_component_insert_index(p_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    physics->vel_x[index] = vel_x;
    physics->vel_y[index] = vel_y;
    physics->vel_z[index] = vel_z;
    
    physics->mass[index]     = mass;
    physics->inv_mass[index] = 1.0f / mass;
    
    physics->damping_v[index] = damping_v;
    physics->damping_w[index] = damping_w;
    
    physics->movement_type[index] = DM_PHYSICS_MOVEMENT_TYPE_KINEMATIC;
    
    dm_ecs_entity_add_component(entity, p_id, context);
}

void entity_add_statics(dm_entity entity, dm_ecs_id p_id, float mass, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_physics* physics = dm_ecs_get_component_block(p_id, context);
    dm_ecs_get_component_insert_index(p_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    physics->mass[index]     = mass;
    physics->inv_mass[index] = 1.0f / mass;
    
    // default for now
    physics->movement_type[index] = DM_PHYSICS_MOVEMENT_TYPE_STATIC;
    
    dm_ecs_entity_add_component(entity, p_id, context);
}

void entity_add_collider_box(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float scale_x,float scale_y,float scale_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    dm_ecs_get_component_insert_index(c_id, &index, context);
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
    
    collision->aabb_local_min_x[index] = min_x;
    collision->aabb_local_min_y[index] = min_y;
    collision->aabb_local_min_z[index] = min_z;
    
    collision->aabb_local_max_x[index] = max_x;
    collision->aabb_local_max_y[index] = max_y;
    collision->aabb_local_max_z[index] = max_z;
    
    collision->aabb_global_min_x[index] = min_x;
    collision->aabb_global_min_y[index] = min_y;
    collision->aabb_global_min_z[index] = min_z;
    
    collision->aabb_global_max_x[index] = max_x;
    collision->aabb_global_max_y[index] = max_y;
    collision->aabb_global_max_z[index] = max_z;
    
    collision->center_x[index] = center_x;
    collision->center_y[index] = center_y;
    collision->center_z[index] = center_z;
    
    collision->internal_0[index] = min_x;
    collision->internal_1[index] = min_y;
    collision->internal_2[index] = min_z;
    collision->internal_3[index] = max_x;
    collision->internal_4[index] = max_y;
    collision->internal_5[index] = max_z;
    
    collision->shape[index] = DM_COLLISION_SHAPE_BOX;
    collision->flag[index]  = COLLISION_FLAG_NO;
    
    dm_ecs_entity_add_component(entity, c_id, context);
}

void entity_add_collider_sphere(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add collision to invalid entity"); return; }
    
    uint32_t index;
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    dm_ecs_get_component_insert_index(c_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    float min_x = center_x - radius;
    float min_y = center_y - radius;
    float min_z = center_z - radius;
    
    float max_x = center_x + radius;
    float max_y = center_y + radius;
    float max_z = center_z + radius;
    
    collision->aabb_local_min_x[index] = min_x;
    collision->aabb_local_min_y[index] = min_y;
    collision->aabb_local_min_z[index] = min_z;
    
    collision->aabb_local_max_x[index] = max_x;
    collision->aabb_local_max_y[index] = max_y;
    collision->aabb_local_max_z[index] = max_z;
    
    collision->aabb_global_min_x[index] = min_x;
    collision->aabb_global_min_y[index] = min_y;
    collision->aabb_global_min_z[index] = min_z;
    
    collision->aabb_global_max_x[index] = max_x;
    collision->aabb_global_max_y[index] = max_y;
    collision->aabb_global_max_z[index] = max_z;
    
    collision->center_x[index] = center_x;
    collision->center_y[index] = center_y;
    collision->center_z[index] = center_z;
    
    collision->internal_0[index] = radius;
    
    collision->shape[index] = DM_COLLISION_SHAPE_SPHERE;
    collision->flag[index]  = COLLISION_FLAG_NO;
    
    dm_ecs_entity_add_component(entity, c_id, context);
}

void entity_add_rigid_body_box(dm_entity entity, dm_ecs_id r_id, float mass, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    dm_ecs_get_component_insert_index(r_id, &index, context);
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
    
    rigid_body->i_body_00[index] = i_body_00;
    rigid_body->i_body_11[index] = i_body_11;
    rigid_body->i_body_22[index] = i_body_22;
    
    rigid_body->i_body_inv_00[index] = 1.0f / i_body_00;
    rigid_body->i_body_inv_11[index] = 1.0f / i_body_11;
    rigid_body->i_body_inv_22[index] = 1.0f / i_body_22;
    
    rigid_body->i_inv_00[index] = rigid_body->i_body_inv_00[index];
    rigid_body->i_inv_11[index] = rigid_body->i_body_inv_11[index];
    rigid_body->i_inv_22[index] = rigid_body->i_body_inv_22[index];
    
    dm_ecs_entity_add_component(entity, r_id, context);
}

void entity_add_rigid_body_sphere(dm_entity entity, dm_ecs_id r_id, float mass, float radius, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_rigid_body* rigid_body = dm_ecs_get_component_block(r_id, context);
    dm_ecs_get_component_insert_index(r_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    const float scalar = 2.0f * 0.2f * mass * radius * radius;
    const float inv_s = 1.0f / scalar;
    
    rigid_body->i_body_00[index] = scalar;
    rigid_body->i_body_11[index] = scalar;
    rigid_body->i_body_22[index] = scalar;
    
    rigid_body->i_body_inv_00[index] = inv_s;
    rigid_body->i_body_inv_11[index] = inv_s;
    rigid_body->i_body_inv_22[index] = inv_s;
    
    rigid_body->i_inv_00[index] = rigid_body->i_body_inv_00[index];
    rigid_body->i_inv_11[index] = rigid_body->i_body_inv_11[index];
    rigid_body->i_inv_22[index] = rigid_body->i_body_inv_22[index];
    
    dm_ecs_entity_add_component(entity, r_id, context);
}

void entity_add_mesh(dm_entity entity, uint32_t m_id, uint32_t mesh, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    
    uint32_t index;
    component_mesh* mesh_component = dm_ecs_get_component_block(m_id, context);
    dm_ecs_get_component_insert_index(m_id, &index, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    mesh_component->mesh_id[index] = mesh;
    
    dm_ecs_entity_add_component(entity, m_id, context);
}

/************
HELPER FUNCS
**************/
void entity_apply_velocity(dm_entity entity, dm_ecs_id p_id, float v_x, float v_y, float v_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics* physics = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    physics->vel_x[comp_index] += v_x;
    physics->vel_y[comp_index] += v_y;
    physics->vel_z[comp_index] += v_z;
}

void entity_apply_angular_velocity(dm_entity entity, dm_ecs_id p_id, float w_x, float w_y, float w_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics* physics = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    physics->w_x[comp_index] += w_x;
    physics->w_y[comp_index] += w_y;
    physics->w_z[comp_index] += w_z;
}

void entity_apply_force(dm_entity entity, dm_ecs_id p_id, float f_x, float f_y, float f_z, dm_context* context)
{
    if(entity==DM_ECS_INVALID_ENTITY) { DM_LOG_ERROR("Trying to add physics to invalid entity"); return; }
    component_physics* physics = context->ecs_manager.components[p_id].data;
    
    uint32_t entity_index = dm_ecs_entity_get_index(entity, context);
    uint32_t comp_index   = context->ecs_manager.entity_component_indices[entity_index][p_id];
    
    if(physics->movement_type[comp_index]==DM_PHYSICS_MOVEMENT_TYPE_STATIC) return;
    
    physics->force_x[comp_index] += f_x;
    physics->force_y[comp_index] += f_y;
    physics->force_z[comp_index] += f_z;
}

/**********
REGISTRING
************/
bool register_transform(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_transform), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register transform component"); 
    return false;
}

bool register_physics(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_physics), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register physics component"); 
    return false;
}

bool register_collision(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_collision), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register collision component"); 
    return false;
}

bool register_rigid_body(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_rigid_body), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register rigidbody component"); 
    return false;
}

bool register_mesh(dm_ecs_id* id, dm_context* context)
{
    *id = dm_ecs_register_component(sizeof(component_mesh), context);
    if(*id!=DM_ECS_INVALID_ID) return true; 
    
    DM_LOG_FATAL("Could not register mesh component"); 
    return false;
}