#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "../DarkMatter/dm.h"

typedef struct view_camera_t
{
    dm_mat4 view, proj, view_proj;
    dm_mat4 inv_view;
    dm_vec3 pos, forward, right, up;
    float near_plane, far_plane, fov;
    float look_sens, move_speed;
    uint32_t width, height;
} view_camera;

void init_camera(dm_vec3 pos, dm_vec3 forward, float near_plane, float far_plane, float fov, float look_sens, float move_speed, uint32_t width, uint32_t height, view_camera* camera);
void update_camera(view_camera* camera);
void resize_camera(uint32_t width, uint32_t height, view_camera* camera);

void track_camera(dm_vec3 pos, dm_vec3 up, float distance, view_camera* camera);
void fps_camera(dm_vec3 pos, dm_vec3 up, view_camera* camera);

#endif //CAMERA_H
