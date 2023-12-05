#include "dm.h"
#include "camera.h"
#include <float.h>

typedef struct ray_t
{
    float origin[N3];
    float direction[N3];
} ray;

typedef struct sphere_t
{
    float pos[3];
    float radius;
    float albedo[4];
} sphere;

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
    
    float clear_color[4];
    
    // timings
    dm_timer timer;
    double t;
    
    // ray directions
    float (*ray_dirs)[3];
    
    // objects
    sphere   spheres[10];
    uint32_t sphere_count;
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

void trace_ray(const ray r, float color[4], application_data* app_data)
{
    if(app_data->sphere_count==0) return;
    
    sphere* s = NULL;
    sphere* nearest_sphere = NULL;
    float origin[3];
    float closest_t;
    float hit_distance = FLT_MAX;
    
    for(uint32_t i=0; i<app_data->sphere_count; i++)
    {
        s = &app_data->spheres[i];
        
        dm_vec3_sub_vec3(r.origin, s->pos, origin);
        
        const float a = dm_vec3_dot(r.direction, r.direction);
        const float b = 2.0f * dm_vec3_dot(origin, r.direction);
        const float c = dm_vec3_dot(origin, origin) - (s->radius * s->radius);
        
        const float dis = b * b - 4.0f * a * c;
        
        // don't hit this sphere
        if(dis < 0) continue;
        
        // are we the closest sphere?
        closest_t = (-b - dm_sqrtf(dis)) / (2.0f * a);
        if(closest_t > hit_distance) continue;
        
        hit_distance = closest_t;
        nearest_sphere = s;
    }
    
    if(!nearest_sphere) return;
    
    dm_vec3_sub_vec3(r.origin, nearest_sphere->pos, origin);
    
    float hit_point[3];
    dm_vec3_scale(r.direction, hit_distance, hit_point);
    dm_vec3_add_vec3(origin, hit_point, hit_point);
    
    float normal[3] = { 0 };
    dm_vec3_norm(hit_point, normal);
    
    float light_dir[] = { -1,-1,-1 };
    dm_vec3_norm(light_dir, light_dir);
    
    float light_dir_neg[3];
    dm_vec3_negate(light_dir, light_dir_neg);
    
    float d = dm_vec3_dot(normal, light_dir_neg);
    d = DM_MAX(d, 0);
    
    // determine color of intersection
    float sphere_color[3];
    dm_vec3_scale(nearest_sphere->albedo, d, sphere_color);
    
    color[0] = sphere_color[0];
    color[1] = sphere_color[1];
    color[2] = sphere_color[2];
}

void recreate_rays(application_data* app_data)
{
    if(app_data->ray_dirs) 
    {
        app_data->ray_dirs = dm_realloc(app_data->ray_dirs, DM_VEC3_SIZE * app_data->image.w * app_data->image.h);
    }
    else
    {
        app_data->ray_dirs = dm_alloc(DM_VEC3_SIZE * app_data->image.w * app_data->image.h);
    }
    
    const float height_f_inv = 1.0f / (float)app_data->image.h;
    const float width_f_inv  = 1.0f / (float)app_data->image.w;
    
    float target[4];
    uint32_t index;
    float w;
    
    float coords[4] = { 0,0,1,1 };
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        coords[1] = (float)y * height_f_inv;
        coords[1] = coords[1] * 2.0f - 1.0f;
        
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            coords[0] = (float)x * width_f_inv;
            coords[0] = coords[0] * 2.0f - 1.0f;
            
            dm_mat4_mul_vec4(app_data->camera.inv_proj, coords, target);
            
            w = 1.0f / target[3];
            target[3] = 0.0f;
            dm_vec4_scale(target, w, target);
            dm_vec4_norm(target, target);
            dm_mat4_mul_vec4(app_data->camera.inv_view, target, target);
            
            index = x + y * app_data->image.w;
            DM_VEC3_COPY(app_data->ray_dirs + index, target);
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
    float camera_p[] = { 0,0,100 };
    float camera_f[] = { 0,0,-1 };
    camera_init(camera_p, camera_f, 0.01f, 100.0f, 75.0f, app_data->image.w, app_data->image.h, 5.0f, 0.1f, &app_data->camera);
    
    recreate_rays(app_data);
    
    // spheres
    sphere s1 = { 
        .pos={0.1f,0,0},
        .radius=0.5f,
        .albedo={1,0,1,1}
    };
    
    dm_memcpy(app_data->spheres + app_data->sphere_count++, &s1, sizeof(s1));
    
    sphere s2 = { 
        .pos={0,0.5f,0},
        .radius=0.2f,
        .albedo={0,0,1,1}
    };
    
    dm_memcpy(app_data->spheres + app_data->sphere_count++, &s2, sizeof(s2));
    
    // misc
    app_data->clear_color[3] = 1;
    
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
    
    dm_timer_start(&app_data->timer, context);
    
    // check image width and height
    const bool width_changed  = context->platform_data.window_data.width != app_data->image.w;
    const bool height_changed = context->platform_data.window_data.height != app_data->image.h;
    
    if(width_changed || height_changed) 
    {
        app_data->image.w = context->platform_data.window_data.width;
        app_data->image.h = context->platform_data.window_data.height;
        
        app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
        app_data->image.data = dm_realloc(app_data->image.data, app_data->image.data_size);
        
        camera_resize(app_data->image.w, app_data->image.h, &app_data->camera, context);
    }
    
    const bool camera_updated = camera_update(&app_data->camera, context);
    if(width_changed || height_changed || camera_updated) recreate_rays(app_data);
    
    // update image
    uint32_t index;
    
    float color[N4] = { 1,1,1,1 };
    ray r;
    DM_VEC3_COPY(r.origin, app_data->camera.pos);
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            index = x + y * app_data->image.w;
            DM_VEC3_COPY(r.direction, app_data->ray_dirs[index]);
            
            dm_memcpy(color, app_data->clear_color, sizeof(color));
            trace_ray(r, color, app_data);
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            
            app_data->image.data[index] = vec4_to_uint32(color);
        }
    }
    
    app_data->t = dm_timer_elapsed_ms(&app_data->timer, context);
    
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
    
    dm_imgui_nuklear_context* imgui_nk_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_nk_ctx->ctx;
    
    sphere* s = &app_data->spheres[0];
    
    if(nk_begin(ctx, "Ray Trace App", nk_rect(650,50, 300,250), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE | 
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 3);
        nk_value_float(ctx, "Camera X", app_data->camera.pos[0]);
        nk_value_float(ctx, "Camera Y", app_data->camera.pos[1]);
        nk_value_float(ctx, "Camera Z", app_data->camera.pos[2]);
        
        nk_layout_row_dynamic(ctx, 30, 3);
        nk_value_float(ctx, "Forward X", app_data->camera.forward[0]);
        nk_value_float(ctx, "Forward Y", app_data->camera.forward[1]);
        nk_value_float(ctx, "Forward Z", app_data->camera.forward[2]);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Last update time (ms)", app_data->t);
        
        nk_layout_row_dynamic(ctx, 30, 3);
        nk_property_float(ctx, "X", -10, &s->pos[0], 10, 0.1f,0.1f);
        nk_property_float(ctx, "Y", -10, &s->pos[1], 10, 0.1f,0.1f);
        nk_property_float(ctx, "Z", -10, &s->pos[2], 10, 0.1f,0.1f);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_property_float(ctx, "Radius", 0.5f, &s->radius, 2.0f, 0.1f,0.1f);
    }
    nk_end(ctx);
    
    return true;
}
