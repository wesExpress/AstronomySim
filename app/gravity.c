#include "gravity.h"

bool gravity_init(void** data, dm_context* context)
{
    *data = dm_alloc(sizeof(gravity_data));
    gravity_data* grav_data = *data;
    
    dm_compute_shader_desc gravity_desc = { 0 };
#ifdef DM_METAL
    strcpy(gravity_desc.path, "assets/shaders/gravity_compute.metallib");
    strcpy(gravity_desc.function, "gravity_calc");
#elif defined(DM_DIRECTX)
    strcpy(gravity_desc.path, "assets/shaders/gravity_compute.fxc");
#else
    DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
    assert(false);
#endif
    
    if(!dm_compute_create_shader(gravity_desc, &grav_data->shader, context)) return false;
    
    return true;
}

void gravity_shutdown(void** data, dm_context* context)
{
    dm_free(*data);
}

bool gravity_run(dm_compute_handle t_buffer, dm_compute_handle p_buffer, uint32_t group_count, uint32_t block_size, void* data, dm_context* context)
{
    gravity_data* grav_data = data;
    
    if(!dm_compute_command_bind_shader(grav_data->shader, context)) return false;
    
    if (!dm_compute_command_bind_buffer(t_buffer, 0, 0, context)) return false;
    if (!dm_compute_command_bind_buffer(p_buffer, 0, 1, context)) return false;
    
    if(!dm_compute_command_dispatch(group_count,1,1, block_size,1,1, context)) return false;
    
    return true;
}
