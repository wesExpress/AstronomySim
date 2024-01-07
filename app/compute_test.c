#include "dm.h"

#include <assert.h>
#include <string.h>

#define ARRAY_LENGTH 1 << 24

typedef struct application_data_t
{
    dm_compute_handle in_a, in_b, result;
    dm_compute_handle shader;
    
    float *a_buffer, *b_buffer, *result_buffer;
} application_data;

void dm_application_setup(dm_context_init_packet* init_packet) {}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    dm_compute_shader_desc desc;
    
#ifdef DM_METAL
    strcpy(desc.path, "assets/shaders/test_compute.metallib");
    strcpy(desc.function, "add_arrays");
#else
    DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
    assert(false);
#endif
    
    dm_compute_create_shader(desc, &app_data->shader, context);
    
    size_t data_size = sizeof(float) * ARRAY_LENGTH;
    
    dm_compute_create_buffer(data_size, sizeof(float), &app_data->in_a, context);
    dm_compute_create_buffer(data_size, sizeof(float), &app_data->in_b, context);
    dm_compute_create_buffer(data_size, sizeof(float), &app_data->result, context);
    
    app_data->a_buffer      = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->b_buffer      = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->result_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_free(app_data->a_buffer);
    dm_free(app_data->b_buffer);
    dm_free(app_data->result_buffer);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_timer t = { 0 };
    
    dm_timer_start(&t, context);
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        float a = dm_random_float(context);
        float b = dm_random_float(context);
        app_data->a_buffer[i] = a;
        app_data->b_buffer[i] = b;
        
        app_data->result_buffer[i] = a + b;
    }
    double ms = dm_timer_elapsed_ms(&t, context);
    
    if(!dm_compute_command_bind_shader(app_data->shader, context)) return false;
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    if(!dm_compute_command_update_buffer(app_data->in_a, app_data->a_buffer, data_size, 0, context)) return false;
    if(!dm_compute_command_update_buffer(app_data->in_b, app_data->b_buffer, data_size, 0, context)) return false;
    
    if(!dm_compute_command_bind_buffer(app_data->in_a, 0, 0, context)) return false;
    if(!dm_compute_command_bind_buffer(app_data->in_b, 0, 1, context)) return false;
    if(!dm_compute_command_bind_buffer(app_data->result, 0, 2, context)) return false;
    
    if(!dm_compute_command_dispatch(ARRAY_LENGTH,1,1, 1024,1,1, context)) return false;
    
    float* result_2 = dm_compute_command_get_buffer_data(app_data->result, context);
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        if(app_data->result_buffer[i] == result_2[i]) continue;
        
        DM_LOG_FATAL("Compute shader output does not match CPU output");
        return false;
    }
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    return true;
}
