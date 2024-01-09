#ifndef CAMERA_H
#define CAMERA_H

#include "dm.h"

typedef struct basic_camera_t
{
    float fov, near_plane, far_plane;
    float move_speed, look_sens;
    
    uint32_t width, height;
    
    dm_vec3 pos;
    dm_vec3 up, forward, right;
    dm_mat4 proj, view, inv_view, view_proj, inv_proj;
} basic_camera;

void camera_init(const dm_vec3 pos, const dm_vec3 forward, float near_plane, float far_plane, float fov, uint32_t width, uint32_t height, float move_speed, float look_sens, basic_camera* camera);
bool camera_update(basic_camera* camera, dm_context* context);
void camera_resize(const uint32_t width, const uint32_t height, basic_camera* camera, dm_context* context);

#endif //CAMERA_H
