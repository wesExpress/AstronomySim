#include "ray_trace.h"

typedef struct ray_trace_data_t
{
    dm_render_handle compute_shader;
} ray_trace_data;

bool ray_trace_init(application_data* app_data, dm_context* context)
{
    app_data->internal_data = dm_alloc(sizeof(ray_trace_data));
    ray_trace_data* data = app_data->internal_data;
    
    /*
        compute testing
        */
    typedef struct t_t
    {
        float x;
        float y;
        float z;
    } t;
    
    dm_compute_shader_desc desc = {
        .path="assets/shaders/test_compute.fxc",
        .input_stride=sizeof(t),
        .max_input_count=DM_ECS_MAX_ENTITIES
    };
    
    if(!dm_renderer_create_compute_shader(desc, &data->compute_shader, context)) return false;
    
    return true;
}

void ray_trace_init_entities(application_data* app_data, dm_context* context)
{
    
}