#include "physics.h"
#include <assert.h>

#define FIXED_DT 0.008333

bool physics_init(void** data, dm_context* context)
{
    *data = dm_alloc(sizeof(physics_data));
    physics_data* p_data = *data;
    
    dm_compute_shader_desc physics_desc = { 0 };
#ifdef DM_METAL
    strcpy(physics_desc.path, "assets/shaders/physics_compute.metallib");
    strcpy(physics_desc.function, "physics_update");
#elif defined(DM_DIRECTX11) || defined(DM_DIRECTX12)
    strcpy(physics_desc.path, "assets/shaders/physics_compute.fxc");
#else
    DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
    assert(false);
#endif
    
    if(!dm_compute_create_shader(physics_desc, &p_data->shader, context)) return false;
    
    return true;
}

void physics_shutdown(void** data, dm_context* context)
{
    dm_free(*data);
}

bool physics_run(dm_compute_handle t_buffer, dm_compute_handle p_buffer, uint32_t group_count, uint32_t block_size, void* data, dm_context* context)
{
    physics_data* p_data = data;
    
    p_data->accumulated_time += context->delta;
    
    while(p_data->accumulated_time >= FIXED_DT)
    {
        p_data->iterations++;
        
        if(!dm_compute_command_bind_shader(p_data->shader, context)) return false;
        
        if (!dm_compute_command_bind_buffer(t_buffer, 0, 0, context)) return false;
        if (!dm_compute_command_bind_buffer(p_buffer, 0, 1, context)) return false;
        
        if(!dm_compute_command_dispatch(group_count,1,1, block_size,1,1, context)) return false;
        
        p_data->accumulated_time -= FIXED_DT;
    }
    
    return true;
}
