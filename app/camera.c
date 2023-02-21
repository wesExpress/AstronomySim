#include "camera.h"

void update_camera_view(view_camera* camera)
{
    camera->view = dm_mat_view_v(camera->pos, dm_vec3_add_vec3(camera->pos, camera->forward), camera->up);
    camera->inv_view = dm_mat4_inverse(camera->view);
    
    camera->view_proj = dm_mat4_mul_mat4(camera->view, camera->proj);
}

void update_camera_proj(view_camera* camera)
{
    camera->proj = dm_mat_perspective_deg(camera->fov, (float)camera->width / (float)camera->height, camera->near_plane, camera->far_plane);
    
    camera->view_proj = dm_mat4_mul_mat4(camera->view, camera->proj);
}

void init_camera(dm_vec3 pos, dm_vec3 forward, float near_plane, float far_plane, float fov, float look_sens, float move_speed, uint32_t width, uint32_t height, view_camera* camera)
{
    camera->pos = pos;
    camera->forward = forward;
    camera->near_plane = near_plane;
    camera->far_plane = far_plane;
    camera->fov = fov;
    camera->look_sens = look_sens;
    camera->move_speed = move_speed;
    camera->width = width;
    camera->height = height;
    
    camera->view = dm_mat_view_v(pos, forward, dm_vec3_unit_y);
    camera->inv_view = dm_mat4_inverse(camera->view);
    camera->proj = dm_mat_perspective(fov, (float)width / (float)height, near_plane, far_plane);
    
    camera->view_proj = dm_mat4_mul_mat4(camera->view, camera->proj);
}

void update_camera(float delta_time, view_camera* camera)
{
    if(!dm_input_is_key_pressed(DM_KEY_LSHIFT)) return;
    
    camera->up = dm_vec3_unit_y;
    
    float speed = delta_time * camera->move_speed;
    int delta_x, delta_y;
    dm_input_get_mouse_delta(&delta_x, &delta_y);
    dm_vec2 look_delta = dm_vec2_set((float)delta_x * camera->look_sens, (float)delta_y * camera->look_sens);
    camera->right = dm_vec3_cross(camera->forward, camera->up);
    
    dm_vec3 delta_pos = { 0 };
    
    // movement
    bool moved = (dm_input_is_key_pressed(DM_KEY_A) || dm_input_is_key_pressed(DM_KEY_D) || dm_input_is_key_pressed(DM_KEY_W) || dm_input_is_key_pressed(DM_KEY_S) || dm_input_is_key_pressed(DM_KEY_Q) || dm_input_is_key_pressed(DM_KEY_E));
    
    if(moved)
    {
        if(dm_input_is_key_pressed(DM_KEY_A)) delta_pos.x = -1;
        else if(dm_input_is_key_pressed(DM_KEY_D)) delta_pos.x = 1;;
        
        if(dm_input_is_key_pressed(DM_KEY_W)) delta_pos.z = 1;
        else if(dm_input_is_key_pressed(DM_KEY_S)) delta_pos.z = -1;
        
        if(dm_input_is_key_pressed(DM_KEY_Q)) delta_pos.y = -1;
        else if(dm_input_is_key_pressed(DM_KEY_E)) delta_pos.y = 1;
        
        dm_vec3 move_vec = dm_vec3_scale(camera->right, delta_pos.x);
        move_vec = dm_vec3_add_vec3(move_vec, dm_vec3_scale(camera->forward, delta_pos.z));
        move_vec = dm_vec3_add_vec3(move_vec, dm_vec3_scale(camera->up, delta_pos.y));
        move_vec = dm_vec3_norm(move_vec);
        move_vec = dm_vec3_scale(move_vec, speed);
        camera->pos = dm_vec3_add_vec3(camera->pos, move_vec);
    }
    
    // rotation
    bool rotated = false;
    if(look_delta.x || look_delta.y)
    {
        float delta_pitch = look_delta.y;
        float delta_yaw = look_delta.x;
        
        dm_quat q1 = dm_quat_from_axis_angle_deg(camera->right, -delta_pitch);
        dm_quat q2 = dm_quat_from_axis_angle_deg(camera->up, -delta_yaw);
        dm_quat rot = dm_quat_cross(q1, q2);
        dm_quat_norm_inpl(&rot);
        
        dm_vec3_rotate_inpl(rot, &camera->forward);
        
        rotated = true;
    }
    
    if(moved || rotated) update_camera_view(camera);
}

void resize_camera(uint32_t width, uint32_t height, view_camera* camera)
{
    if((width == camera->width) && (height == camera->height)) return;
    
    camera->width = width;
    camera->height = height;
    
    update_camera_proj(camera);
}

void track_camera(dm_vec3 pos, dm_vec3 up, float distance, view_camera* camera)
{
    int delta_x, delta_y;
    dm_input_get_mouse_delta(&delta_x, &delta_y);
    
    static float dx = 0.0f;
    static float dy = 0.0f;
    
    dx += (float)delta_x * camera->look_sens;
    dy += (float)delta_y * camera->look_sens;
    
    camera->pos.x = pos.x + distance * dm_sind(dy) * dm_cosd(dx);
    camera->pos.y = pos.y + distance * dm_cosd(dy);
    camera->pos.z = pos.z + distance * dm_sind(dy) * dm_sind(dx);
    
    camera->forward = dm_vec3_sub_vec3(pos, camera->pos);
    camera->up = up;
    
    update_camera_view(camera);
}

void set_camera_pos(dm_vec3 pos, view_camera* camera)
{
    camera->pos = pos;
    update_camera_view(camera);
}

void fps_camera(dm_vec3 pos, dm_vec3 up, view_camera* camera)
{
    int delta_x, delta_y;
    dm_input_get_mouse_delta(&delta_x, &delta_y);
    dm_vec2 look_delta = dm_vec2_set((float)delta_x * camera->look_sens, (float)delta_y * camera->look_sens);
    camera->up = up;
    camera->right = dm_vec3_cross(camera->forward, camera->up);
    
    camera->pos = pos;
    
    // rotation
    if(look_delta.x || look_delta.y)
    {
        float delta_pitch = look_delta.y;
        float delta_yaw = look_delta.x;
        
        dm_quat q1 = dm_quat_from_axis_angle_deg(camera->right, -delta_pitch);
        dm_quat q2 = dm_quat_from_axis_angle_deg(camera->up, -delta_yaw);
        dm_quat rot = dm_quat_cross(q1, q2);
        dm_quat_norm_inpl(&rot);
        
        dm_vec3_rotate_inpl(rot, &camera->forward);
    }
    
    update_camera_view(camera);
}