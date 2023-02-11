#include "components.h"

#define G            6.673e-11f

#define NUM_OBJECTS 100
typedef struct physics_test_data_t
{
    dm_entity entities[NUM_OBJECTS];
    uint32_t  num_entities;
    
    dm_entity ref_entity;
} physics_test_data;

static physics_test_data physics_data = { 0 };
static dm_vec4 gray = { 0.25f,0.25f,0.25f,1.0f };

dm_entity make_entity(view_camera* camera)
{
    dm_entity entity = dm_ecs_create_entity();
    
    static float radius = 0.25f;
    static float v      = 2.0f;
    float f = dm_random_float();
    
    dm_ecs_entity_add_transform_v(entity, camera->pos, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    if(f > 0.5f)
    {
        dm_ecs_entity_add_collision_sphere(entity, radius);
        dm_ecs_entity_add_mesh(entity, ICOSPHERE_MESH);
    }
    else
    {
        dm_ecs_entity_add_collision_box(entity, dm_vec3_set(-radius,-radius,-radius), dm_vec3_set(radius,radius,radius));
        dm_ecs_entity_add_mesh(entity, BOX_MESH);
    }
    dm_ecs_entity_add_physics_at_rest(entity, 1e3f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(entity, gray, gray);
    
    dm_physics_add_impulse(entity, dm_vec3_scale(dm_vec3_norm(camera->forward), v));
    
    return entity;
}

float get_circular_velocity(dm_entity host, float x, float y, float z)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* mass  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_MASS);
    
    float r_x = pos_x[host] - x;
    float r_y = pos_y[host] - y;
    float r_z = pos_z[host] - z;
    
    float gm = G * mass[host];
    float r  = dm_sqrtf(r_x * r_x + r_y * r_y + r_z * r_z);
    return dm_sqrtf(gm / r);
}

return_code app_init()
{
    // gravity system
    gravity_system_init();
    
    dm_physics_toggle_pause();
    
    // light
    dm_entity entity = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(entity, dm_vec3_set(0,4,0), dm_vec3_set(0,0,0), dm_quat_set(0,0,0,1));
    add_point_light_component(entity, dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec3_set(0,0,0), 1.0f, 0.009f, 0.009f, COMPONENT_LIGHT);
    
    physics_data.entities[physics_data.num_entities++] = entity;
    
    // massive object
    float pos_x0 = 0.0f;
    float pos_y0 = 0.0f;
    float pos_z0 = 0.0f;
    float radius = 1.0f;
    float mass0 = 1e12f;
    entity = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(entity, pos_x0,pos_y0,pos_z0, radius,radius,radius, 0,0,0,1);
    dm_ecs_entity_add_collision_sphere(entity, radius);
    dm_ecs_entity_add_physics_at_rest(entity, mass0, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(entity, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(entity, gray, gray);
    
    dm_physics_add_angular_velocity(entity, dm_vec3_set(0,1,0));
    dm_physics_add_impulse(entity, dm_vec3_set(-2,0,0));
    
    physics_data.entities[physics_data.num_entities++] = entity;
    
    float pos_x1 = 0.0f;
    float pos_y1 = 0.0f;
    float pos_z1 = 10.0f;
    float mass1 = 1e12f;
    dm_entity entity3 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(entity3, pos_x1,pos_y1,pos_z1, radius,radius,radius, 0,0,0,1);
    dm_ecs_entity_add_collision_sphere(entity3, radius);
    dm_ecs_entity_add_physics_at_rest(entity3, mass1, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(entity3, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(entity3, gray,gray);
    
    dm_physics_add_angular_velocity(entity3, dm_vec3_set(0,1,0));
    dm_physics_add_impulse(entity3, dm_vec3_set(2,0,0));
    
    physics_data.entities[physics_data.num_entities++] = entity3;
    
    // orbiting object(s)
    float pos_x2 = 0;
    float pos_y2 = 0;
    float pos_z2 = 2;
    radius = 0.25f;
    float mass2  = 10.0f;
    dm_entity entity2 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(entity2, pos_x2,pos_y2,pos_z2, radius,radius,radius, 0,0,0,1);
    dm_ecs_entity_add_collision_sphere(entity2, radius);
    dm_ecs_entity_add_physics_at_rest(entity2, mass2, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(entity2, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(entity2, gray, gray);
    
    physics_data.entities[physics_data.num_entities++] = entity2;
    physics_data.ref_entity = entity2;
    
    float vc = get_circular_velocity(entity, pos_x2,pos_y2,pos_z2) - 2.0f;
    //dm_physics_add_impulse(entity2, dm_vec3_set(vc,0,0));
    
    pos_z2 = 13.0f;
    entity2 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(entity2, pos_x2,pos_y2,pos_z2, radius,radius,radius, 0,0,0,1);
    dm_ecs_entity_add_collision_sphere(entity2, radius);
    dm_ecs_entity_add_physics_at_rest(entity2, mass2, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(entity2, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(entity2, gray, gray);
    
    physics_data.entities[physics_data.num_entities++] = entity2;
    
    vc = get_circular_velocity(entity3, pos_x2,pos_y2,pos_z2) + 2.0f;
    dm_physics_add_impulse(entity2, dm_vec3_set(vc,0,0));
    
    return SUCCESS;
}

return_code app_update(view_camera* camera)
{
    update_camera(dm_get_delta_time(), camera);
    
#if 0 
    if(dm_input_mousebutton_just_pressed(DM_MOUSEBUTTON_L))
    {
        if(physics_data.num_entities < NUM_OBJECTS) physics_data.entities[physics_data.num_entities++] = make_entity(camera);
        else DM_LOG_ERROR("Trying to create entity beyond max: %u", NUM_OBJECTS);
    }
#endif
    
    dm_entity host = physics_data.entities[1];
    
    // continuous floating origin around one of orbiting bodies
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
    float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
    float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
    float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
    float* w_x = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_X);
    float* w_y = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_Y);
    float* w_z = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_Z);
    
    // rotate everyone around orbiting bodies with angular rate of host omega?
    float ref_x = pos_x[physics_data.ref_entity];
    float ref_y = pos_y[physics_data.ref_entity];
    float ref_z = pos_z[physics_data.ref_entity];
    float host_x = pos_x[host];
    float host_y = pos_y[host];
    float host_z = pos_z[host];
    
    dm_vec3 w = dm_vec3_set(w_x[host], w_y[host], w_z[host]);
    w = dm_vec3_negate(w);
    dm_quat rot = dm_quat_set(rot_i[host], rot_j[host], rot_k[host], rot_r[host]);
    dm_quat delta_rot = dm_vec3_mul_quat(dm_vec3_scale(w, dm_get_delta_time()), rot);
    delta_rot = dm_quat_scale(delta_rot, 0.5f);
    dm_mat3 r = dm_mat3_rotate_from_quat(delta_rot);
    
    for(uint32_t i=0; i<physics_data.num_entities; i++)
    {
        dm_entity entity = physics_data.entities[i];
        
        dm_vec3 p = dm_vec3_set(pos_x[entity] - host_x, pos_y[entity] - host_y, pos_z[entity] - host_z);
        p = dm_mat3_mul_vec3(r, p);
        
        pos_x[entity] = p.x - ref_x + host_x;
        pos_y[entity] = p.y - ref_y + host_y;
        pos_z[entity] = p.z - ref_z + host_z;
    }
    
    return SUCCESS;
}

return_code app_render()
{
#if 0
    for(uint32_t i=1; i<physics_data.num_entities; i++)
    {
        dm_debug_render_transform(physics_data.entities[i]);
        dm_debug_render_aabb(physics_data.entities[i]);
        dm_debug_render_velocity_vector(physics_data.entities[i]);
    }
#endif
    
    return SUCCESS;
}

