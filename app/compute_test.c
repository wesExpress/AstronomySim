#include "dm.h"

#include <assert.h>
#include <string.h>

#define ARRAY_LENGTH 5000

typedef struct application_data_t
{
    dm_compute_handle shader;
    
    double compute_timing;
    
    DM_ALIGN(16) float* x;
    DM_ALIGN(16) float* y;
    DM_ALIGN(16) float* z;
    
    DM_ALIGN(16) float* m;
    
    DM_ALIGN(16) float* f_x;
    DM_ALIGN(16) float* f_y;
    DM_ALIGN(16) float* f_z;
    
    dm_compute_handle xb, yb, zb, mb;
    dm_compute_handle fx_b, fy_b, fz_b;
} application_data;

void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->vsync = false;
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    dm_compute_shader_desc desc = { 0 };
    
#ifdef DM_METAL
    strcpy(desc.path, "assets/shaders/gravity_compute.metallib");
    strcpy(desc.function, "gravity_calc");
#elif defined(DM_DIRECTX)
    strcpy(desc.path, "assets/shaders/test_compute.fxc");
#else
    DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
    assert(false);
#endif
    
    if(!dm_compute_create_shader(desc, &app_data->shader, context)) return false;
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->xb, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->yb, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->zb, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->mb, context))   return false;

    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fx_b, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fy_b, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fz_b, context))   return false;

    app_data->x   = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->y   = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->z   = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->m   = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    
    app_data->f_x = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->f_y = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    app_data->f_z = dm_alloc(sizeof(float) * ARRAY_LENGTH);
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        app_data->x[i] = dm_random_float(context) * 100.f - 50.f;
        app_data->y[i] = dm_random_float(context) * 100.f - 50.f;
        app_data->z[i] = dm_random_float(context) * 100.f - 50.f;
        
        app_data->m[i] = dm_random_float(context) * 1e4f;
    }
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_free(app_data->x);
    dm_free(app_data->y);
    dm_free(app_data->z);
    dm_free(app_data->m);
    
    dm_free(app_data->f_x);
    dm_free(app_data->f_y);
    dm_free(app_data->f_z);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    
    if(!dm_compute_command_bind_shader(app_data->shader, context)) return false;
    
    if(!dm_compute_command_update_buffer(app_data->xb, app_data->x, data_size, 0, context)) return false;
    if(!dm_compute_command_update_buffer(app_data->yb, app_data->y, data_size, 0, context)) return false;
    if(!dm_compute_command_update_buffer(app_data->zb, app_data->z, data_size, 0, context)) return false;
    if(!dm_compute_command_update_buffer(app_data->mb, app_data->m, data_size, 0, context)) return false;
    
    if(!dm_compute_command_bind_buffer(app_data->xb, 0, 0, context))  return false;
    if(!dm_compute_command_bind_buffer(app_data->yb, 0, 1, context))  return false;
    if(!dm_compute_command_bind_buffer(app_data->zb, 0, 2, context))  return false;
    if(!dm_compute_command_bind_buffer(app_data->mb, 0, 3, context))  return false;
    if(!dm_compute_command_bind_buffer(app_data->fx_b, 0, 4, context)) return false;
    if(!dm_compute_command_bind_buffer(app_data->fy_b, 0, 5, context)) return false;
    if(!dm_compute_command_bind_buffer(app_data->fz_b, 0, 6, context)) return false;

    dm_timer t = { 0 };
    dm_timer_start(&t, context);
#ifdef DM_METAL
    if(!dm_compute_command_dispatch(ARRAY_LENGTH,1,1, 1024,1,1, context)) return false;
#elif defined(DM_DIRECTX)
    if(!dm_compute_command_dispatch(65535,1,1, 1024,1,1, context)) return false;
#endif
    app_data->compute_timing = dm_timer_elapsed_ms(&t, context);
    
    float* f_x = dm_compute_command_get_buffer_data(app_data->fx_b, context);
    float* f_y = dm_compute_command_get_buffer_data(app_data->fy_b, context);
    float* f_z = dm_compute_command_get_buffer_data(app_data->fz_b, context);

    if(!f_x || !f_y || !f_z) return false;
    
    dm_memcpy(app_data->f_x, f_x, data_size);
    dm_memcpy(app_data->f_y, f_y, data_size);
    dm_memcpy(app_data->f_z, f_z, data_size);

    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    if(nk_begin(ctx, "Test", nk_rect(100, 100, 200, 100), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Compute took", app_data->compute_timing);
        
        nk_end(ctx);
    }
    
    return true;
}
