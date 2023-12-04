#include "dm.h"
#include "camera.h"

typedef struct app_handles_t
{
    dm_render_handle vb, shader, pipeline;
    dm_render_handle texture;
    dm_render_handle compute_shader;
} app_handles;

typedef struct app_image_t
{
    uint32_t  w,h;
    size_t    data_size;
    uint32_t* data;
} app_image;

typedef struct application_data_t
{
    app_handles handles;
    app_image   image;
    
    basic_camera camera;
    
    // rays
    float (*ray_dirs)[3];
} application_data;

/*******
HELPERS
*********/
uint32_t vec4_to_uint32(const float vec[4])
{
    const uint8_t r = (uint8_t)(255.0f * vec[0]);
    const uint8_t g = (uint8_t)(255.0f * vec[1]);
    const uint8_t b = (uint8_t)(255.0f * vec[2]);
    const uint8_t a = (uint8_t)(255.0f * vec[3]);
    
    uint32_t result = (a << 24) | (b << 16) | (g << 8) | r;
    return result;
}

void fill_pixel(const float ray_dir[3], const float ray_origin[3], float color[4])
{
    float radius = 0.5f;
    
    const float a = dm_vec3_dot(ray_dir, ray_dir);
    const float b = 2.0f * dm_vec3_dot(ray_origin, ray_dir);
    const float c = dm_vec3_dot(ray_origin, ray_origin) - radius * radius;
    
    const float dis = b * b - 4.0f * a * c;
    
    // haven't hit anything, so early out
    if(dis < 0) return;
    
    const float t = 0.5f * (-b - dm_sqrtf(dis)) / a;
    float hit_point[3];
    dm_vec3_scale(ray_dir, t, hit_point);
    dm_vec3_add_vec3(ray_origin, hit_point, hit_point);
    
    float normal[3] = { 0 };
    dm_vec3_norm(hit_point, normal);
    
    float light_dir[] = { -1,-1,-1 };
    float light_dir_neg[3];
    dm_vec3_negate(light_dir, light_dir_neg);
    
    float d = dm_vec3_dot(normal, light_dir_neg);
    d = DM_MAX(d, 0);
    
    // determine color of intersection
    float sphere_color[3] = { 1,0,1 };
    dm_vec3_scale(sphere_color, d, sphere_color);
    
    color[0] = sphere_color[0];
    color[1] = sphere_color[1];
    color[2] = sphere_color[2];
}

void recreate_rays(application_data* app_data)
{
    if(app_data->ray_dirs) 
    {
        app_data->ray_dirs = dm_realloc(app_data->ray_dirs, sizeof(float) * 3 * app_data->image.w * app_data->image.h);
    }
    else
    {
        app_data->ray_dirs = dm_alloc(sizeof(float) * 3 * app_data->image.w * app_data->image.h);
    }
    
    const float height_f_inv = 1.0f / (float)app_data->image.h;
    const float width_f_inv  = 1.0f / (float)app_data->image.w;
    
    float target[4];
    uint32_t index;
    float w;
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            target[0] = (float)x * width_f_inv;
            target[0] = target[0] * 2.0f - 1.0f;
            
            target[1] = (float)y * height_f_inv;
            target[1] = target[1] * 2.0f - 1.0f;
            
            target[2] = target[3] = 1;
            
            dm_mat4_mul_vec4(app_data->camera.inv_proj, target, target);
            
            w = 1.0f / target[3];
            target[3] = 0.0f;
            dm_vec4_scale(target, w, target);
            dm_vec4_norm(target, target);
            dm_mat4_mul_vec4(app_data->camera.inv_view, target, target);
            
            index = x + y * app_data->image.w;
            dm_memcpy(app_data->ray_dirs + index, target, sizeof(float) * 3);
        }
    }
}

/*******************
FRAMEWORK INTERFACE
*********************/
void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->window_width  = 1000;
    init_packet->window_height = 1000;
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    // vb is just a quad
    typedef struct vertex_t
    {
        float pos[2];
        float uv[2];
    } vertex;
    
    vertex vertices[] = {
        // triangle 1
        { { -1,-1 }, { 0,0 } },
        { {  1,-1 }, { 1,0 } },
        { {  1, 1 }, { 1,1 } },
        
        // triangle 2
        { {  1, 1 }, { 1,1 } },
        { { -1, 1 }, { 0,1 } },
        { { -1,-1 }, { 0,0 } },
    };
    
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(vertex), &app_data->handles.vb, context)) return false;
    
    // pipeline and shader
    dm_pipeline_desc p_desc = dm_renderer_default_pipeline();
    
    dm_shader_desc s_desc = {
#ifdef DM_DIRECTX
        .vertex="assets/shaders/quad_vertex.fxc",
        .pixel="assets/shaders/quad_pixel.fxc",
#elif defined(DM_OPENGL)
        .vertex="assets/shaders/quad_vertex.glsl",
        .pixel="assets/shaders/quad_pixel.glsl",
#endif
        .vb={ app_data->handles.vb },
        .vb_count=1
    };
    
    dm_vertex_attrib_desc attrib_descs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, pos), .count=2, .index=0, .normalized=false },
        { .name="TEX_COORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, uv), .count=2, .index=0, .normalized=false },
    };
    
    if(!dm_renderer_create_shader_and_pipeline(s_desc, p_desc, attrib_descs, 2, &app_data->handles.shader, &app_data->handles.pipeline, context)) return false;
    
    // texture we write to
    app_data->image.w = context->platform_data.window_data.width;
    app_data->image.h = context->platform_data.window_data.height;
    
    app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
    
    app_data->image.data = dm_alloc(app_data->image.data_size);
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            app_data->image.data[x + y * app_data->image.w]  = dm_random_uint32(context);
            app_data->image.data[x + y * app_data->image.w] |= 0xff000000;
        }
    }
    
    if(!dm_renderer_create_dynamic_texture(app_data->image.w, app_data->image.h, 4, app_data->image.data, "image_texture", &app_data->handles.texture, context)) return false;
    
    // camera
    float camera_p[] = { 0,0,5 };
    float camera_f[] = { 0,0,-1 };
    camera_init(camera_p, camera_f, 0.1f, 100.0f, 75.0f, app_data->image.w, app_data->image.h, 5.0f, 1.0f, &app_data->camera);
    
    recreate_rays(app_data);
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_free(app_data->ray_dirs);
    dm_free(app_data->image.data);
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // check image width and height
    const bool width_changed  = context->platform_data.window_data.width != app_data->image.w;
    const bool height_changed = context->platform_data.window_data.height != app_data->image.h;
    const bool camera_updated = camera_update(&app_data->camera, context);
    
    if(width_changed || height_changed) 
    {
        camera_resize(app_data->image.w, app_data->image.h, &app_data->camera, context);
        
        app_data->image.w = context->platform_data.window_data.width;
        app_data->image.h = context->platform_data.window_data.height;
        
        app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
        app_data->image.data = dm_realloc(app_data->image.data, app_data->image.data_size);
    }
    
    if(width_changed || height_changed || camera_updated) recreate_rays(app_data);
    
    // update image
    uint32_t index;
    
    float color[N4] = { 0,1,0,1 };
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            index = x + y * app_data->image.w;
            
            color[0] = 0; color[1] = 0; color[2] = 0; color[3] = 1;
            fill_pixel(app_data->ray_dirs[index], app_data->camera.pos, color);
            color[0] = DM_CLAMP(color[0], 0, 1);
            color[1] = DM_CLAMP(color[1], 0, 1);
            color[2] = DM_CLAMP(color[2], 0, 1);
            
            app_data->image.data[index] = vec4_to_uint32(color);
        }
    }
    
    dm_render_command_update_texture(app_data->handles.texture, app_data->image.w, app_data->image.h, app_data->image.data, app_data->image.data_size, context);
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    
    dm_render_command_bind_shader(app_data->handles.shader, context);
    dm_render_command_bind_pipeline(app_data->handles.pipeline, context);
    dm_render_command_bind_buffer(app_data->handles.vb, 0, context);
    dm_render_command_bind_texture(app_data->handles.texture, 0, context);
    dm_render_command_draw_arrays(0, 6, context);
    
    return true;
}
