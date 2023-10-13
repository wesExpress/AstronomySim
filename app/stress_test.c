#include "stress_test.h"

#include "dm.h"

#include "components.h"

#define WORLD_SIZE 150

dm_entity create_entity(application_data* app_data, dm_context* context)
{
    dm_entity entity = dm_ecs_entity_create(context);
    
    static const float h = (float)WORLD_SIZE * 0.5f;
    
    float pos_x = (dm_random_float(context) * 2 - 1) * h;
    float pos_y = (dm_random_float(context) * 2 - 1) * h;
    float pos_z = (dm_random_float(context) * 2 - 1) * h;
    
    float scale_x = dm_random_float_range(0.5,3,context);
    float scale_y = dm_random_float_range(0.5,3,context);
    float scale_z = dm_random_float_range(0.5,3,context);
    
    float rot_i = dm_random_float(context);
    float rot_j = dm_random_float(context);
    float rot_k = dm_random_float(context);
    float rot_r = dm_random_float(context);
    float mag   = dm_sqrtf(rot_i*rot_i + rot_j*rot_j + rot_k*rot_k + rot_r*rot_r);
    rot_i /= mag;
    rot_j /= mag;
    rot_k /= mag;
    rot_r /= mag;
    
    float mass = DM_MIN(scale_x, DM_MIN(scale_y, scale_z)) * 1e2f;
    entity_add_kinematics(entity, app_data->components.physics, mass, 0,0,0, 0,0.1f, context);
    
    if(dm_random_float(context) > 1)
    {
        scale_y = scale_z = scale_x;
        
        float radius = scale_x * 0.5f;
        
        entity_add_collider_sphere(entity, app_data->components.collision, 0,0,0, radius, context);
        entity_add_rigid_body_sphere(entity, app_data->components.rigid_body, mass, radius, context);
    }
    else
    {
        float x_dim = scale_x * 0.5f;
        float y_dim = scale_y * 0.5f;
        float z_dim = scale_z * 0.5f;
        
        entity_add_collider_box(entity, app_data->components.collision, 0,0,0, scale_x,scale_y,scale_z, context);
        entity_add_rigid_body_box(entity, app_data->components.rigid_body, mass, -x_dim,-y_dim,-z_dim,x_dim,y_dim,z_dim, context);
    }
    
    entity_add_transform(entity, app_data->components.transform, pos_x,pos_y,pos_z, scale_x,scale_y,scale_z, rot_i,rot_j,rot_k,rot_r, context);
    
    return entity;
}

void stress_test_init_entities(application_data* app_data, dm_context* context)
{
    for(uint32_t i=0; i<DM_ECS_MAX_ENTITIES; i++)
    {
        app_data->entities[app_data->entity_count++] = create_entity(app_data, context);
    }
}