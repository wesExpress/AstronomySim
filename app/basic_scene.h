#ifndef __BASIC_SCENE_H__
#define __BASIC_SCENE_H__

#include "components.h"

#define NUM_ENTITIES 10
static dm_entity entities[NUM_ENTITIES] = { 0 };
static uint32_t  entity_count = 0;

#define NBODY
#define OBJ_SPHERE

return_code app_init()
{
#ifdef NBODY
    gravity_system_init();
#endif
    
    //dm_physics_toggle_pause();
    
    entities[entity_count] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(entities[entity_count], 0,4,0, 0,0,0, 0,0,0,1);
    add_point_light_component(entities[entity_count], dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec3_set(0,0,0), 1,0.0009f,0.00007f);
    
    entities[++entity_count] = dm_ecs_create_entity();
#define PLANE_SCALE 5.0f
#if 0
#define PLANE_DEPTH  0.1f
    dm_ecs_entity_add_transform(entities[entity_count], 0,0,0, PLANE_SCALE,PLANE_DEPTH,PLANE_SCALE, 0,0,0,1);
    dm_ecs_entity_add_collision_box(entities[entity_count], dm_vec3_set(-PLANE_SCALE * 0.5f,-PLANE_DEPTH * 0.5f,-PLANE_SCALE * 0.5f), dm_vec3_set(PLANE_SCALE * 0.5f,PLANE_DEPTH * 0.5f,PLANE_SCALE * 0.5f));
    dm_ecs_entity_add_mesh(entities[entity_count], BOX_MESH);
#else
    dm_ecs_entity_add_transform(entities[entity_count], 0,0,0, PLANE_SCALE * 0.5f,PLANE_SCALE * 0.5f,PLANE_SCALE * 0.5f, 0,0,0,1);
    dm_ecs_entity_add_collision_sphere(entities[entity_count], PLANE_SCALE * 0.5f);
    dm_ecs_entity_add_mesh(entities[entity_count], ICOSPHERE_MESH);
#endif
    dm_ecs_entity_add_physics_at_rest(entities[entity_count], 1e12f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(entities[entity_count], dm_vec4_set(0.5f,0.15f,0,1), dm_vec4_set(0.5f,0,0,1));
    
    //dm_physics_add_angular_momentum(entities[entity_count], dm_vec3_set(0,5e10f,0));
    dm_physics_add_angular_velocity(entities[entity_count], dm_vec3_set(0,0.1f,0));
    
    float size = 0.25f;
    float half_s = size * 0.5f;
    
    entities[++entity_count] = dm_ecs_create_entity();
#if 1
    dm_ecs_entity_add_transform(entities[entity_count], 7,0,0, size,size,size, 1,0.25f,1,1);
    dm_ecs_entity_add_collision_box(entities[entity_count], dm_vec3_set(-half_s, -half_s, -half_s), dm_vec3_set(half_s, half_s, half_s));
    dm_ecs_entity_add_mesh(entities[entity_count], BOX_MESH);
#else
    dm_ecs_entity_add_transform(entities[entity_count], 7.0f,0,0, half_s,half_s,half_s, 1,0.25f,1,1);
    dm_ecs_entity_add_collision_sphere(entities[entity_count], half_s);
    dm_ecs_entity_add_mesh(entities[entity_count], ICOSPHERE_MESH);
#endif
    dm_ecs_entity_add_physics_at_rest(entities[entity_count], 1.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(entities[entity_count], dm_vec4_set(0,0.25f,0.25f,1), dm_vec4_set(0.5f,0,0,1));
    
    //dm_physics_add_impulse(entities[entity_count], dm_vec3_set(0,0,-0.5f));
    
    floating_origin_system_init(entities[entity_count]);
    
    return SUCCESS;
}

return_code app_update(view_camera* camera)
{
    update_camera(camera);
    
#ifndef NBODY
    dm_physics_apply_earth_gravity(entities[entity_count]);
#endif
    
    //if(dm_ecs_entity_is_colliding(entities[entity_count])) floating_origin_enable_rot(entities[entity_count-1]);
    //else floating_origin_disable_rot();
    
    return SUCCESS;
}

return_code app_render()
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* radius = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_UNION_0);
    
    dm_entity entity = entities[entity_count];
    
    dm_debug_render_line(pos_x[entity],pos_y[entity],pos_z[entity], pos_x[entity],pos_y[entity]-radius[entity],pos_z[entity], 1.0f, 1,1,1,1);
    
    return SUCCESS;
}

#endif //BASIC_SCENE_H
