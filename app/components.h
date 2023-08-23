#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "dm.h"

// transform
typedef struct component_transform_block_t
{
    float pos_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float pos_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float pos_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float scale_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float scale_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float scale_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float rot_i[DM_ECS_COMPONENT_BLOCK_SIZE];
    float rot_j[DM_ECS_COMPONENT_BLOCK_SIZE];
    float rot_k[DM_ECS_COMPONENT_BLOCK_SIZE];
    float rot_r[DM_ECS_COMPONENT_BLOCK_SIZE];
} component_transform_block;

typedef struct component_transform_t
{
    float pos[3];
    float scale[3];
    float rot[4];
} component_transform;

// collision
typedef enum collision_flag_t
{
    COLLISION_FLAG_NO,
    COLLISION_FLAG_YES,
    COLLISION_FLAG_POSSIBLE,
    COLLISION_FLAG_UNKNOWN
} collision_flag;

typedef struct aabb_t
{
    float min[3];
    float max[3];
} aabb;

typedef struct component_collision_block_t
{
    float aabb_local_min_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_local_min_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_local_min_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float aabb_local_max_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_local_max_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_local_max_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float aabb_global_min_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_global_min_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_global_min_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float aabb_global_max_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_global_max_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float aabb_global_max_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float center_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float center_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float center_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float internal_0[DM_ECS_COMPONENT_BLOCK_SIZE];
    float internal_1[DM_ECS_COMPONENT_BLOCK_SIZE];
    float internal_2[DM_ECS_COMPONENT_BLOCK_SIZE];
    float internal_3[DM_ECS_COMPONENT_BLOCK_SIZE];
    float internal_4[DM_ECS_COMPONENT_BLOCK_SIZE];
    float internal_5[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    dm_collision_shape shape[DM_ECS_COMPONENT_BLOCK_SIZE];
    collision_flag     flag[DM_ECS_COMPONENT_BLOCK_SIZE];
} component_collision_block;

typedef struct component_collision_t
{
    float aabb_local_min[3];
    float aabb_local_max[3];
    float aabb_global_min[3];
    float aabb_global_max[3];
    
    float center[3];
    
    float internal[6];
    
    dm_collision_shape shape;
    collision_flag     flag;
} component_collision;

// physics
typedef enum physics_body_type_t
{
    PHYSICS_BODY_TYPE_RIGID,
    PHYSICS_BODY_TYPE_UNKNOWN
} physics_body_type;

typedef enum physics_movement_type_t
{
    PHYSICS_MOVEMENT_KINEMATIC,
    PHYSICS_MOVEMENT_STATIC,
    MOVEMENT_UNKNOWN
} physics_movement_type;

typedef struct dm_component_physics_block_t
{
    float vel_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float vel_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float vel_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float w_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float w_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float w_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float l_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float l_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float l_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float force_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float force_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float force_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float torque_x[DM_ECS_COMPONENT_BLOCK_SIZE];
    float torque_y[DM_ECS_COMPONENT_BLOCK_SIZE];
    float torque_z[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float mass[DM_ECS_COMPONENT_BLOCK_SIZE];
    float inv_mass[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    // moment of inertia at rest are diagonals
    // but global inertia is a full 3x3 matrix
    float i_body_00[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_body_11[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_body_22[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float i_body_inv_00[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_body_inv_11[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_body_inv_22[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float i_inv_00[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_01[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_02[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float i_inv_10[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_11[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_12[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    float i_inv_20[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_21[DM_ECS_COMPONENT_BLOCK_SIZE];
    float i_inv_22[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    // damping coefs
    float damping_v[DM_ECS_COMPONENT_BLOCK_SIZE];
    float damping_w[DM_ECS_COMPONENT_BLOCK_SIZE];
    
    // enums
    physics_body_type     body_type[DM_ECS_COMPONENT_BLOCK_SIZE];
    physics_movement_type movement_type[DM_ECS_COMPONENT_BLOCK_SIZE];
} component_physics_block;

typedef struct dm_component_physics_t
{
    float vel[3];
    float w[3];
    float l[3];
    float force[3];
    float torque[3];
    float mass, inv_mass;
    
    // moment of inertia at rest are diagonals
    // but global inertia is a full 3x3 matrix
    float i_body[3];
    float i_body_inv[3];
    float i_inv_0[3];
    float i_inv_1[3];
    float i_inv_2[3];
    
    // damping coefs
    float damping[2];
    
    // enums
    physics_body_type     body_type;
    physics_movement_type movement_type;
} component_physics;

// funcs
bool  register_components(dm_context* context);

const component_transform entity_get_transform(dm_entity entity, dm_ecs_id t_id, dm_context* context);
const component_physics   entity_get_physics(dm_entity entity, dm_ecs_id p_id, dm_context* context);
const component_collision entity_get_collision(dm_entity entity, dm_ecs_id c_id, dm_context* context);

void entity_add_transform(dm_entity entity, dm_ecs_id t_id, float pos_x,float pos_y,float pos_z, float scale_x,float scale_y,float scale_z, float rot_i,float rot_j,float rot_k,float rot_r, dm_context* context);
void entity_add_kinematics_box_rigid_body(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, float min_x,float min_y, float min_z, float max_x, float max_y, float max_z, dm_context* context);
void entity_add_kinematics_sphere_rigid_body(dm_entity entity, dm_ecs_id p_id, float mass, float vel_x, float vel_y, float vel_z, float damping_v, float damping_w, float radius, dm_context* context);
void entity_add_box_collider(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float dim_x,float dim_y,float dim_z, dm_context* context);
void entity_add_sphere_collider(dm_entity entity, dm_ecs_id c_id, float center_x,float center_y,float center_z, float radius, dm_context* context);

void entity_add_velocity(dm_entity, dm_ecs_id p_id, float v_x, float v_y, float v_z, dm_context* context);
void entity_add_angular_velocity(dm_entity, dm_ecs_id p_id, float w_x, float w_y, float w_z, dm_context* context);
void entity_add_force(dm_entity, dm_ecs_id p_id, float f_x, float f_y, float f_z, dm_context* context);

#endif //COMPONENTS_H
