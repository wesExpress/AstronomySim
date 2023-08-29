#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "app_defines.h"
#include "dm.h"

// transform
typedef struct component_transform_t
{
    float pos_x[DM_ECS_MAX_ENTITIES];
    float pos_y[DM_ECS_MAX_ENTITIES];
    float pos_z[DM_ECS_MAX_ENTITIES];
    
    float scale_x[DM_ECS_MAX_ENTITIES];
    float scale_y[DM_ECS_MAX_ENTITIES];
    float scale_z[DM_ECS_MAX_ENTITIES];
    
    float rot_i[DM_ECS_MAX_ENTITIES];
    float rot_j[DM_ECS_MAX_ENTITIES];
    float rot_k[DM_ECS_MAX_ENTITIES];
    float rot_r[DM_ECS_MAX_ENTITIES];
} component_transform;

// collision
typedef enum collision_flag_t
{
    COLLISION_FLAG_NO,
    COLLISION_FLAG_YES,
    COLLISION_FLAG_POSSIBLE,
    COLLISION_FLAG_UNKNOWN
} collision_flag;

typedef struct component_collision_t
{
    float aabb_local_min_x[DM_ECS_MAX_ENTITIES];
    float aabb_local_min_y[DM_ECS_MAX_ENTITIES];
    float aabb_local_min_z[DM_ECS_MAX_ENTITIES];
    
    float aabb_local_max_x[DM_ECS_MAX_ENTITIES];
    float aabb_local_max_y[DM_ECS_MAX_ENTITIES];
    float aabb_local_max_z[DM_ECS_MAX_ENTITIES];
    
    float aabb_global_min_x[DM_ECS_MAX_ENTITIES];
    float aabb_global_min_y[DM_ECS_MAX_ENTITIES];
    float aabb_global_min_z[DM_ECS_MAX_ENTITIES];
    
    float aabb_global_max_x[DM_ECS_MAX_ENTITIES];
    float aabb_global_max_y[DM_ECS_MAX_ENTITIES];
    float aabb_global_max_z[DM_ECS_MAX_ENTITIES];
    
    float center_x[DM_ECS_MAX_ENTITIES];
    float center_y[DM_ECS_MAX_ENTITIES];
    float center_z[DM_ECS_MAX_ENTITIES];
    
    float internal_0[DM_ECS_MAX_ENTITIES];
    float internal_1[DM_ECS_MAX_ENTITIES];
    float internal_2[DM_ECS_MAX_ENTITIES];
    float internal_3[DM_ECS_MAX_ENTITIES];
    float internal_4[DM_ECS_MAX_ENTITIES];
    float internal_5[DM_ECS_MAX_ENTITIES];
    
    dm_collision_shape shape[DM_ECS_MAX_ENTITIES];
    collision_flag     flag[DM_ECS_MAX_ENTITIES];
} component_collision;

// physics
typedef enum physics_movement_type_t
{
    PHYSICS_MOVEMENT_KINEMATIC,
    PHYSICS_MOVEMENT_STATIC,
    MOVEMENT_UNKNOWN
} physics_movement_type;

typedef struct dm_component_physics_t
{
    float vel_x[DM_ECS_MAX_ENTITIES];
    float vel_y[DM_ECS_MAX_ENTITIES];
    float vel_z[DM_ECS_MAX_ENTITIES];
    
    float w_x[DM_ECS_MAX_ENTITIES];
    float w_y[DM_ECS_MAX_ENTITIES];
    float w_z[DM_ECS_MAX_ENTITIES];
    
    float l_x[DM_ECS_MAX_ENTITIES];
    float l_y[DM_ECS_MAX_ENTITIES];
    float l_z[DM_ECS_MAX_ENTITIES];
    
    float force_x[DM_ECS_MAX_ENTITIES];
    float force_y[DM_ECS_MAX_ENTITIES];
    float force_z[DM_ECS_MAX_ENTITIES];
    
    float torque_x[DM_ECS_MAX_ENTITIES];
    float torque_y[DM_ECS_MAX_ENTITIES];
    float torque_z[DM_ECS_MAX_ENTITIES];
    
    float mass[DM_ECS_MAX_ENTITIES];
    float inv_mass[DM_ECS_MAX_ENTITIES];
    
    // damping coefs
    float damping_v[DM_ECS_MAX_ENTITIES];
    float damping_w[DM_ECS_MAX_ENTITIES];
    
    // enums
    physics_movement_type movement_type[DM_ECS_MAX_ENTITIES];
} component_physics;

// rigid body
typedef struct component_rigid_body_t
{
    // moment of inertia at rest are diagonals
    // but global inertia is a full 3x3 matrix
    float i_body_00[DM_ECS_MAX_ENTITIES];
    float i_body_11[DM_ECS_MAX_ENTITIES];
    float i_body_22[DM_ECS_MAX_ENTITIES];
    
    float i_body_inv_00[DM_ECS_MAX_ENTITIES];
    float i_body_inv_11[DM_ECS_MAX_ENTITIES];
    float i_body_inv_22[DM_ECS_MAX_ENTITIES];
    
    float i_inv_00[DM_ECS_MAX_ENTITIES];
    float i_inv_01[DM_ECS_MAX_ENTITIES];
    float i_inv_02[DM_ECS_MAX_ENTITIES];
    
    float i_inv_10[DM_ECS_MAX_ENTITIES];
    float i_inv_11[DM_ECS_MAX_ENTITIES];
    float i_inv_12[DM_ECS_MAX_ENTITIES];
    
    float i_inv_20[DM_ECS_MAX_ENTITIES];
    float i_inv_21[DM_ECS_MAX_ENTITIES];
    float i_inv_22[DM_ECS_MAX_ENTITIES];
} component_rigid_body;

// funcs
bool register_transform(dm_ecs_id* id, dm_context* context);
bool register_physics(dm_ecs_id* id, dm_context* context);
bool register_collision(dm_ecs_id* id, dm_context* context);
bool register_rigid_body(dm_ecs_id* id, dm_context* context);

void entity_add_transform(dm_entity entity, dm_ecs_id t_id, float pos_x,float pos_y,float pos_z, float scale_x,float scale_y,float scale_z, float rot_i,float rot_j,float rot_k,float rot_r, dm_context* context);
void entity_add_kinematics(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, dm_context* context);
void entity_add_collider_box(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float dim_x,float dim_y,float dim_z, dm_context* context);
void entity_add_collider_sphere(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float radius, dm_context* context);
void entity_add_rigid_body_box(dm_entity entity, dm_ecs_id r_id, float mass, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, dm_context* context);
void entity_add_rigid_body_sphere(dm_entity entity, dm_ecs_id r_id, float mass, float radius, dm_context* context);

void entity_apply_velocity(dm_entity, dm_ecs_id p_id, float v_x, float v_y, float v_z, dm_context* context);
void entity_apply_angular_velocity(dm_entity, dm_ecs_id p_id, float w_x, float w_y, float w_z, dm_context* context);
void entity_apply_force(dm_entity, dm_ecs_id p_id, float f_x, float f_y, float f_z, dm_context* context);

#endif //COMPONENTS_H
