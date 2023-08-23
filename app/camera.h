#ifndef CAMERA_H
#define CAMERA_H

#include "dm.h"

typedef struct basic_camera_t
{
    float fov, near_plane, far_plane;
    float move_speed, look_sens;
    
    uint32_t width, height;
    
    float pos[N3];
    float up[N3], forward[N3], right[N3];
    float proj[M4], view[M4], inv_view[M4], view_proj[M4];
} basic_camera;

void camera_init(const float pos[3], const float forward[3], float near_plane, float far_plane, float fov, uint32_t width, uint32_t height, float move_speed, float look_sens, basic_camera* camera);
void camera_update(basic_camera* camera, dm_context* context);

#endif //CAMERA_H
