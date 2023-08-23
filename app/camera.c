#include "camera.h"

void camera_init(const float pos[N3], const float forward[N3], float near_plane, float far_plane, float fov, uint32_t width, uint32_t height, float move_speed, float look_sens, basic_camera* camera)
{
    static const size_t vec3_size = sizeof(float) * 3;
    
    camera->fov        = fov;
    camera->near_plane = near_plane;
    camera->far_plane  = far_plane;
    camera->move_speed = move_speed;
    camera->look_sens  = look_sens;
    camera->width      = width;
    camera->height     = height;
    
    dm_memcpy(camera->pos,     pos,     vec3_size);
    dm_memcpy(camera->forward, forward, vec3_size);
    camera->up[0] = 0; camera->up[1] = 1; camera->up[2] = 0;
    
    float look[N3];
    dm_vec3_add_vec3(camera->pos, camera->forward, look);
    
    dm_mat_view(camera->pos, look, camera->up, camera->view);
    dm_mat4_inverse(camera->view, camera->inv_view);
    dm_mat_perspective(dm_deg_to_rad(camera->fov), (float)width / (float)height, camera->near_plane, camera->far_plane, camera->proj);
    dm_mat4_mul_mat4(camera->view, camera->proj, camera->view_proj);
    
    dm_vec3_cross(camera->forward, camera->up, camera->right);
}

void update_camera_view(basic_camera* camera)
{
    float look[3];
    dm_vec3_add_vec3(camera->pos, camera->forward, look);
    
    dm_vec3_cross(camera->right, camera->forward, camera->up);
    dm_vec3_norm(camera->up, camera->up);
    
    dm_mat_view(camera->pos, look, camera->up, camera->view);
    dm_mat4_inverse(camera->view, camera->inv_view);
    
    dm_mat4_mul_mat4(camera->view, camera->proj, camera->view_proj);
}

void update_camera_proj(basic_camera* camera)
{
    dm_mat_perspective(dm_deg_to_rad(camera->fov), (float)camera->width / (float)camera->height, camera->near_plane, camera->far_plane, camera->proj);
    
    dm_mat4_mul_mat4(camera->view, camera->proj, camera->view_proj);
}

void resize_camera(basic_camera* camera, dm_context* context)
{
    uint32_t width  = DM_SCREEN_WIDTH(context);
    uint32_t height = DM_SCREEN_HEIGHT(context);
    if((width == camera->width) && (height == camera->height)) return;
    
    camera->width = width;
    camera->height = height;
    
    update_camera_proj(camera);
}

void camera_update(basic_camera* camera, dm_context* context)
{
    static float up[] = { 0,1,0 };
    
    resize_camera(camera, context);
    
    bool moved = (dm_input_is_key_pressed(DM_KEY_A, context) || dm_input_is_key_pressed(DM_KEY_D, context) || dm_input_is_key_pressed(DM_KEY_W, context) || dm_input_is_key_pressed(DM_KEY_S, context) || dm_input_is_key_pressed(DM_KEY_Q, context) || dm_input_is_key_pressed(DM_KEY_E, context));
    
    int delta_x, delta_y;
    dm_input_get_mouse_delta(&delta_x, &delta_y, context);
    bool rotated = (delta_x != 0) || (delta_y != 0);
    
    // movement
    if(moved)
    {
        float speed         = camera->move_speed * context->delta;
        float delta_pos[N3] = { 0 };
        
        if(dm_input_is_key_pressed(DM_KEY_A, context))      delta_pos[0] = -1;
        else if(dm_input_is_key_pressed(DM_KEY_D, context)) delta_pos[0] =  1;
        
        if(dm_input_is_key_pressed(DM_KEY_W, context))      delta_pos[2] =  1;
        else if(dm_input_is_key_pressed(DM_KEY_S, context)) delta_pos[2] = -1;
        
        if(dm_input_is_key_pressed(DM_KEY_Q, context))      delta_pos[1] = -1;
        else if(dm_input_is_key_pressed(DM_KEY_E, context)) delta_pos[1] =  1;
        
        float move_vec[3], dum[3];
        
        dm_vec3_scale(camera->right, delta_pos[0], move_vec);
        dm_vec3_scale(camera->forward, delta_pos[2], dum);
        dm_vec3_add_vec3(move_vec, dum, move_vec);
        dm_vec3_scale(up, delta_pos[1], dum);
        dm_vec3_add_vec3(move_vec, dum, move_vec);
        
        dm_vec3_norm(move_vec, move_vec);
        dm_vec3_scale(move_vec, speed, move_vec);
        
        dm_vec3_add_vec3(camera->pos, move_vec, camera->pos);
    }
    
    // rotation
    if(rotated)
    {
        float delta_yaw   = (float)delta_x * camera->look_sens;
        float delta_pitch = (float)delta_y * camera->look_sens;
        
        float q1[4], q2[4], rot[4];
        
        dm_quat_from_axis_angle_deg(camera->right, -delta_pitch, q1);
        dm_quat_from_axis_angle_deg(up,            -delta_yaw,   q2);
        dm_quat_cross(q1, q2, rot);
        dm_quat_norm(rot, rot);
        
        dm_vec3_rotate(camera->forward, rot, camera->forward);
        dm_vec3_norm(camera->forward, camera->forward);
        
        dm_vec3_cross(camera->forward, up, camera->right);
    }
    
    if(moved || rotated) update_camera_view(camera);
}