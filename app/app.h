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

#define MAX_ENTITIES 1024
typedef struct application_data_t
{
    uint32_t     entity_count;
    dm_entity    entities[MAX_ENTITIES];
    basic_camera camera;
    
    component_ids components;
    
    void*        render_pass_data;
    void*        debug_render_pass_data;
    void*        imgui_pass_data;
} application_data;

/*
bool app_init(dm_context* context);
void app_shutdown(dm_context* context);
bool app_update(dm_context* context);
bool app_render(dm_context* context);
*/

#endif //APP_H