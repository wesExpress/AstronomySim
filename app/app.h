#ifndef APP_H
#define APP_H

#include "dm.h"
#include "camera.h"

typedef struct component_ids_t
{
    dm_ecs_id transform;
    dm_ecs_id collision;
    dm_ecs_id physics;
} component_ids;

#define DRAW_LEN 100
#define MAX_ENTITIES 4096
typedef struct application_data_t
{
    uint32_t     entity_count;
    dm_entity    entities[MAX_ENTITIES];
    basic_camera camera;
    
    component_ids components;
    
    float draw_pos_x[DRAW_LEN];
    float draw_pos_y[DRAW_LEN];
    float draw_pos_z[DRAW_LEN];
    dm_timer draw_timer;
    
    void*        render_pass_data;
    void*        debug_render_pass_data;
    void*        imgui_pass_data;
} application_data;

#endif //APP_H