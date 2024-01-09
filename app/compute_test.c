#include "dm.h"

#include <assert.h>
#include <string.h>

#define ARRAY_LENGTH (1 << 24)

typedef struct stuff_data_t
{
    float offset_a, offset_b;
    float scale_a, scale_b;
} stuff_data;

typedef struct application_data_t
{
    dm_compute_handle in_a, in_b, stuff, result;
    dm_compute_handle shader;
    
    DM_ALIGN(16) float *a_buffer;
    DM_ALIGN(16) float *b_buffer;
    DM_ALIGN(16) float *result_buffer;
} application_data;

void dm_application_setup(dm_context_init_packet* init_packet) {}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    dm_compute_shader_desc desc = { 0 };
    
#ifdef DM_METAL
    strcpy(desc.path, "assets/shaders/test_compute.metallib");
    strcpy(desc.function, "add_arrays");
#elif defined(DM_DIRECTX)
    strcpy(desc.path, "assets/shaders/test_compute.fxc");
#else
    DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
    assert(false);
#endif
    
    dm_compute_create_shader(desc, &app_data->shader, context);
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->in_a, context))   return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->in_b, context))   return false;
    //if(!dm_compute_create_uniform(sizeof(stuff_data), &app_data->stuff, context))       return false;
    if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->result, context)) return false;
    
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
    
    stuff_data s = { 0 };
    s.offset_a = 10.0f;
    s.offset_b = -2.5f;
    s.scale_a = 123.4f;
    s.scale_b = 2.0f;
    
    dm_timer t = { 0 };
    
    static const float mil_elems = (float)ARRAY_LENGTH / 1e6f;
    
    DM_LOG_WARN("%0.2f million elements", mil_elems);
    
    dm_timer_start(&t, context);
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        app_data->a_buffer[i] = dm_random_float(context);
        app_data->b_buffer[i] = dm_random_float(context);
    }
    DM_LOG_WARN("Generating: %0.2lf ms", dm_timer_elapsed_ms(&t, context));
    
    dm_timer_start(&t, context);
#if 0
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        //app_data->result_buffer[i] = (app_data->a_buffer[i] + s.offset_a) * s.scale_a + (app_data->b_buffer[i] + s.offset_b) * s.scale_b;
        app_data->result_buffer[i] = app_data->a_buffer[i] + app_data->b_buffer[i];
    }
#else
#ifdef DM_SIMD_X86
    dm_simd256_float a, b, result, temp_a, temp_b;
    const dm_simd256_float offset_a = dm_simd256_set1_float(s.offset_a);
    const dm_simd256_float offset_b = dm_simd256_set1_float(s.offset_b);
    const dm_simd256_float scale_a  = dm_simd256_set1_float(s.scale_a);
    const dm_simd256_float scale_b  = dm_simd256_set1_float(s.scale_b);
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i+=DM_SIMD256_FLOAT_N)
    {
        a = dm_simd256_load_float(app_data->a_buffer + i);
        b = dm_simd256_load_float(app_data->b_buffer + i);
        
#if 0
        temp_a = dm_simd256_add_float(a, offset_a);
        temp_a = dm_simd256_mul_float(temp_a, scale_a);
        temp_b = dm_simd256_add_float(b, offset_b);
        temp_b = dm_simd256_mul_float(temp_b, scale_b);
        
        result = dm_simd256_add_float(temp_a, temp_b);
#else
        result = dm_simd256_add_float(a, b);
#endif
        
        dm_simd256_store_float(app_data->result_buffer + i, result);
    }
#else
    dm_simd_float a, b, result, temp_a, temp_b;
    const dm_simd_float offset_a = dm_simd_set1_float(s.offset_a);
    const dm_simd_float offset_b = dm_simd_set1_float(s.offset_b);
    const dm_simd_float scale_a  = dm_simd_set1_float(s.scale_a);
    const dm_simd_float scale_b  = dm_simd_set1_float(s.scale_b);
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i+=DM_SIMD_FLOAT_N)
    {
        a = dm_simd_load_float(app_data->a_buffer + i);
        b = dm_simd_load_float(app_data->b_buffer + i);
        
#if 0
        temp_a = dm_simd_add_float(a, offset_a);
        temp_a = dm_simd_mul_float(temp_a, scale_a);
        temp_b = dm_simd_add_float(b, offset_b);
        temp_b = dm_simd_mul_float(temp_b, scale_b);
        
        result = dm_simd_add_float(temp_a, temp_b);
#else
        result = dm_simd_add_float(a, b);
#endif
        
        dm_simd_store_float(app_data->result_buffer + i, result);
    }
#endif
#endif
    DM_LOG_WARN("CPU: %0.2lf ms", dm_timer_elapsed_ms(&t, context));
    
    if(!dm_compute_command_bind_shader(app_data->shader, context)) return false;
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    if(!dm_compute_command_update_buffer(app_data->in_a, app_data->a_buffer, data_size, 0, context)) return false;
    if(!dm_compute_command_update_buffer(app_data->in_b, app_data->b_buffer, data_size, 0, context)) return false;
    //if(!dm_compute_command_update_buffer(app_data->stuff, &s, sizeof(s), 0, context))                return false;
    
    if(!dm_compute_command_bind_buffer(app_data->in_a, 0, 0, context))   return false;
    if(!dm_compute_command_bind_buffer(app_data->in_b, 0, 1, context))   return false;
    //if(!dm_compute_command_bind_buffer(app_data->stuff, 0, 2, context))  return false;
    if(!dm_compute_command_bind_buffer(app_data->result, 0, 2, context)) return false;
    
    dm_timer_start(&t, context);
#ifdef DM_METAL
    if(!dm_compute_command_dispatch(ARRAY_LENGTH,1,1, 1,1,1, context)) return false;
#elif defined(DM_DIRECTX)
    if(!dm_compute_command_dispatch(65535,1,1, 1024,1,1, context)) return false;
#endif
    DM_LOG_INFO("Compute shader: %0.2lf ms", dm_timer_elapsed_ms(&t, context));
    
    float* result_2 = dm_compute_command_get_buffer_data(app_data->result, context);
    
    dm_timer_start(&t, context);
    float r1, r2;
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        r1 = app_data->result_buffer[i];
        r2 = result_2[i];
        if(r1 == r2) continue;
        if(dm_fabs(r1 - r2) < 0.001f) continue;
        
        DM_LOG_FATAL("Compute shader output does not match CPU output");
        return false;
    }
    DM_LOG_INFO("Checking: %lf ms\n", dm_timer_elapsed(&t, context));
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    if(nk_begin(ctx, "Test", nk_rect(100, 100, 100, 100), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_end(ctx);
    }
    
    return true;
}
