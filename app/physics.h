#ifndef PHYSICS_H
#define PHYSICS_H

#include "dm.h"

typedef struct physics_data_t
{
    dm_compute_handle shader;
    
    double   accumulated_time;
    uint32_t iterations;
} physics_data;

bool physics_init(void** data, dm_context* context);
void physics_shutdown(void** data, dm_context* context);
bool physics_run(dm_compute_handle t_buffer, dm_compute_handle p_buffer, uint32_t group_count, uint32_t block_size, void* data, dm_context* context);

#endif //PHYSICS_H
