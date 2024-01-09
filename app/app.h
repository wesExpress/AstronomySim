#ifndef APP_H
#define APP_H

#include "camera.h"

typedef struct component_ids_t
{
    dm_ecs_id transform;
    dm_ecs_id collision;
    dm_ecs_id physics;
    dm_ecs_id rigid_body;
    dm_ecs_id mesh;
} component_ids;

#define DRAW_LEN 100
typedef struct timer_draw_data_t
{
    float draw_pos_x[DRAW_LEN];
    float draw_pos_y[DRAW_LEN];
    float draw_pos_z[DRAW_LEN];
    dm_timer draw_timer;
} timer_draw_data;

typedef struct application_data_t
{
    basic_camera camera;
    
    component_ids components;
    
    dm_ecs_id physics_system, gravity_system;
    dm_ecs_system_timing physics_system_timing, gravity_system_timing;
    
    dm_entity    entities[DM_ECS_MAX_ENTITIES];
    uint32_t     entity_count;
    
    timer_draw_data draw_data;
    
    void* render_pass_data;
    void* debug_render_pass_data;
    void* imgui_pass_data;
    
    void* internal_data;
} application_data;

#endif //APP_H