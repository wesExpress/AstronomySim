#include "physics_test.h"
#include "components.h"
#include "dm.h"
#include <float.h>

#define STACK_WIDTH  6
#define STACK_HEIGHT 6
#define STACK_DEPTH  6

void physics_test_init_entities(application_data* app_data, dm_context* context)
{
    dm_entity entity;
    
    // ground
    entity = dm_ecs_entity_create(context);
    
    const float ground_mass = FLT_MAX;
    
    const float ground_width  = 50;
    const float ground_height = 0.25f;
    const float ground_depth  = 50;
    
    const float ground_half_width = ground_width * 0.5f;
    const float ground_half_height = ground_height * 0.5f;
    const float ground_half_depth = ground_depth * 0.5f;
    
    entity_add_transform(entity, app_data->components.transform, 0,0,0, ground_width,ground_height,ground_depth, 0,0,0,1, context);
    entity_add_statics(entity, app_data->components.physics, ground_mass, context);
    entity_add_collider_box(entity, app_data->components.collision, 0,0,0, ground_width,ground_height,ground_depth, context);
    entity_add_rigid_body_box(entity, app_data->components.rigid_body, ground_mass, -ground_half_width,-ground_half_height,-ground_half_depth, ground_half_width,ground_half_height,ground_half_depth, context);
    
    app_data->entities[app_data->entity_count++] = entity;
    
    // box stack
    const float box_height_start = 10.0f;
    
    const float box_width  = 1.0f;
    const float box_height = 1.0f;
    const float box_depth  = 1.0f;
    
    const float half_width  = box_width * 0.5f;
    const float half_height = box_height * 0.5f;
    const float half_depth  = box_depth * 0.5f;
    
    const float mass = 1.0f;
    
    float pos_x;
    float pos_y;
    float pos_z;
    
    for(uint32_t i=0; i<STACK_WIDTH; i++)
    {
        pos_x = -half_width + i * box_width - STACK_WIDTH * half_width;
        for(uint32_t j=0; j<STACK_HEIGHT; j++)
        {
            pos_y = -half_height + j * box_height + box_height_start;
            for(uint32_t k=0; k<STACK_DEPTH; k++)
            {
                pos_z = -half_depth + k * box_depth - STACK_DEPTH * half_depth;
                
                entity = dm_ecs_entity_create(context);
                
                entity_add_transform(entity, app_data->components.transform, pos_x,pos_y,pos_z, box_width,box_height,box_depth, 0,0,0,1, context);
                entity_add_kinematics(entity, app_data->components.physics, mass, 0,0,0, 0,0, context);
                entity_add_collider_box(entity, app_data->components.collision, 0,0,0, box_width, box_height, box_depth, context);
                entity_add_rigid_body_box(entity, app_data->components.rigid_body, mass, -half_width,-half_height,-half_depth, half_width,half_height,half_depth, context);
                
                app_data->entities[app_data->entity_count++] = entity;
            }
        }
    }
}

void physics_test_update_entities(application_data* app_data, dm_context* context)
{
    const float box_mass = 1.0f;
    float force = 0.5f * -9.8f * box_mass;
    
    for(uint32_t i=0; i<app_data->entity_count; i++)
    {
        entity_apply_force(app_data->entities[i], app_data->components.physics, 0,force,0, context);
    }
}