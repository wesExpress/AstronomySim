#ifndef GRAVITY_H
#define GRAVITY_H

#include "dm.h"

typedef struct gravity_data_t
{
    dm_compute_handle shader;
} gravity_data;

bool gravity_init(void** data, dm_context* context);
void gravity_shutdown(void** data, dm_context* context);
bool gravity_run(dm_compute_handle t_buffer, dm_compute_handle p_buffer, uint32_t group_count, uint32_t block_size, void* data, dm_context* context);

#endif //GRAVITY_H
