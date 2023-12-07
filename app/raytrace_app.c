#include "dm.h"
#include "camera.h"
#include <float.h>

typedef struct ray_t
{
    float origin[N3];
    float direction[N3];
} ray;

typedef struct hit_payload_t
{
    float hit_distance;
    
    dm_vec3 world_position;
    dm_vec3 world_normal;
    
    uint32_t obj_index;
} hit_payload;

typedef struct sphere_t
{
    dm_vec3 pos;
    float radius;
    
    dm_vec4 albedo;
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

#define MAX_SPHERE_COUNT 100
typedef struct application_data_t
{
    app_handles handles;
    app_image   image;
    
    basic_camera camera;
    
    dm_vec4 clear_color;
    
    // timings
    dm_timer timer;
    double image_creation_t, ray_creation_t;
    
    // ray directions
    dm_vec4* ray_dirs;
    
    // objects
    sphere   spheres[MAX_SPHERE_COUNT];
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

hit_payload closest_hit(const ray r, float hit_distance, int obj_index, dm_vec4 color, application_data* app_data)
{
    hit_payload payload = {
        .hit_distance=hit_distance,
        .obj_index=obj_index
    };
    
    const sphere* s = &app_data->spheres[obj_index];
    
    dm_vec3 origin;
    dm_vec3_sub_vec3(r.origin, s->pos, origin);
    
    dm_vec3 hit_point;
    dm_vec3_scale(r.direction, hit_distance, hit_point);
    dm_vec3_add_vec3(origin, hit_point, payload.world_position);
    
    dm_vec3 normal = { 0 };
    dm_vec3_norm(payload.world_position, payload.world_normal);
    
    dm_vec3_add_vec3(payload.world_position, s->pos, payload.world_position);
    
    return payload;
}

hit_payload miss(const ray r, dm_vec4 color)
{
    hit_payload payload = {
        .hit_distance=-1
    };
    
    return payload;
}

hit_payload trace_ray(const ray r, float color[4], application_data* app_data)
{
    sphere* s = NULL;
    dm_vec3 origin;
    float closest_t;
    float hit_distance = FLT_MAX;
    int nearest_sphere_index = -1;
    
    for(uint32_t i=0; i<app_data->sphere_count; i++)
    {
        s = &app_data->spheres[i];
        
        dm_vec3_sub_vec3(r.origin, s->pos, origin);
        
        float a = dm_vec3_dot(r.direction, r.direction);
        float b = 2.0f * dm_vec3_dot(origin, r.direction);
        float c = dm_vec3_dot(origin, origin) - s->radius * s->radius;
        
        float dis = b * b - 4.0f * a * c;
        
        // don't hit this sphere
        if(dis < 0) continue;
        
        // are we the closest sphere?
        closest_t = (-b - dm_sqrtf(dis)) / (2.0f * a);
        if(closest_t > hit_distance || closest_t < 0) continue;
        
        hit_distance = closest_t;
        nearest_sphere_index = i;
    }
    
    if(nearest_sphere_index<0) return miss(r, color);
    
    return closest_hit(r, hit_distance, nearest_sphere_index, color, app_data);
}

void per_pixel(uint32_t x, uint32_t y, float color[4], application_data* app_data)
{
    ray r;
    DM_VEC3_COPY(r.origin, app_data->camera.pos);
    
    uint32_t index = x + y * app_data->image.w;
    
    r.direction[0] = app_data->ray_dirs[index][0];
    r.direction[1] = app_data->ray_dirs[index][1];
    r.direction[2] = app_data->ray_dirs[index][2];
    r.direction[3] = app_data->ray_dirs[index][3];
    
    uint32_t bounces=2;
    float multiplier = 1.0f;
    
    for(uint32_t i=0; i<bounces; i++)
    {
        hit_payload payload = trace_ray(r, color, app_data);
        
        if(payload.hit_distance<0)
        {
            color[0] += multiplier * app_data->clear_color[0];
            color[1] += multiplier * app_data->clear_color[1];
            color[2] += multiplier * app_data->clear_color[2];
            
            break;
        }
        
        // lighting
        dm_vec3 light_dir = { -1,-1,-1 };
        dm_vec3_norm(light_dir, light_dir);
        
        dm_vec3 light_dir_neg;
        dm_vec3_negate(light_dir, light_dir_neg);
        
        float d = dm_vec3_dot(payload.world_normal, light_dir_neg);
        d = DM_MAX(d, 0);
        
        // determine color 
        const sphere* s = &app_data->spheres[payload.obj_index];
        dm_vec3 sphere_color;
        dm_vec3_scale(s->albedo, d, sphere_color);
        
        color[0] += multiplier * sphere_color[0];
        color[1] += multiplier * sphere_color[1];
        color[2] += multiplier * sphere_color[2];
        
        multiplier *= 0.75;
        
        dm_vec3 offset_pos;
        dm_vec3 scaled_normal;
        dm_vec3_scale(payload.world_normal, 0.0001f, scaled_normal);
        dm_vec3_add_vec3(payload.world_position, scaled_normal, offset_pos);
        
        r.origin[0] = offset_pos[0];
        r.origin[1] = offset_pos[1];
        r.origin[2] = offset_pos[2];
        
        dm_vec3_reflect(r.direction, payload.world_normal, r.direction);
    }
}

void recreate_rays(application_data* app_data)
{
    if(app_data->ray_dirs) 
    {
        app_data->ray_dirs = dm_realloc(app_data->ray_dirs, DM_VEC4_SIZE * app_data->image.w * app_data->image.h);
    }
    else
    {
        app_data->ray_dirs = dm_alloc(DM_VEC4_SIZE * app_data->image.w * app_data->image.h);
    }
    
    const float height_f_inv = 1.0f / (float)app_data->image.h;
    const float width_f_inv  = 1.0f / (float)app_data->image.w;
    
    dm_vec4 target, dir;
    uint32_t index;
    float w;
    
    dm_mm_float w_mm, target_mm, row1,row2,row3,row4;
    dm_mm_float target_x_mm, target_y_mm, target_z_mm, target_w_mm;
    dm_mm_float coords_mm, coords_x_mm, coords_y_mm, coords_z_mm, coords_w_mm;
    
    dm_vec4 coords = { 0,0,1,1 };
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        coords[1] = (float)y * height_f_inv;
        coords[1] = coords[1] * 2.0f - 1.0f;
        
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            index = x + y * app_data->image.w;
            
            coords[0] = (float)x * width_f_inv;
            coords[0] = coords[0] * 2.0f - 1.0f;
            
            ///// Full SIMD ray direction creation
            coords_mm = dm_mm_load_ps(coords);
            
            coords_x_mm = dm_mm_broadcast_x_ps(coords_mm);
            coords_y_mm = dm_mm_broadcast_y_ps(coords_mm);
            coords_z_mm = dm_mm_broadcast_z_ps(coords_mm);
            coords_w_mm = dm_mm_broadcast_w_ps(coords_mm);
            
            row1 = dm_mm_load_ps(app_data->camera.inv_proj[0]);
            row2 = dm_mm_load_ps(app_data->camera.inv_proj[1]);
            row3 = dm_mm_load_ps(app_data->camera.inv_proj[2]);
            row4 = dm_mm_load_ps(app_data->camera.inv_proj[3]);
            
            dm_mm_transpose_mat4(&row1,&row2,&row3,&row4);
            
            target_mm = dm_mm_mul_ps(coords_x_mm, row1);
            target_mm = dm_mm_fmadd_ps(coords_y_mm,row2, target_mm);
            target_mm = dm_mm_fmadd_ps(coords_z_mm,row3, target_mm);
            target_mm = dm_mm_fmadd_ps(coords_w_mm,row4, target_mm);
            
            w_mm = dm_mm_broadcast_w_ps(target_mm);
            target_mm = dm_mm_mul_ps(target_mm, w_mm);
            target_mm = dm_mm_normalize_ps(target_mm);
            
            target_x_mm = dm_mm_broadcast_x_ps(target_mm);
            target_y_mm = dm_mm_broadcast_y_ps(target_mm);
            target_z_mm = dm_mm_broadcast_z_ps(target_mm);
            
            row1 = dm_mm_load_ps(app_data->camera.inv_view[0]);
            row2 = dm_mm_load_ps(app_data->camera.inv_view[1]);
            row3 = dm_mm_load_ps(app_data->camera.inv_view[2]);
            row4 = dm_mm_load_ps(app_data->camera.inv_view[3]);
            
            dm_mm_transpose_mat4(&row1,&row2,&row3,&row4);
            
            target_mm = dm_mm_mul_ps(target_x_mm, row1);
            target_mm = dm_mm_fmadd_ps(target_y_mm,row2, target_mm);
            target_mm = dm_mm_fmadd_ps(target_z_mm,row3, target_mm);
            
            dm_mm_store_ps(app_data->ray_dirs[index], target_mm);
            /////
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
    camera_init(camera_p, camera_f, 0.1f, 100.0f, 75.0f, app_data->image.w, app_data->image.h, 5.0f, 0.1f, &app_data->camera);
    
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
    
    // test
    mat4 t;
    
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
    
    if(width_changed || height_changed) 
    {
        app_data->image.w = context->platform_data.window_data.width;
        app_data->image.h = context->platform_data.window_data.height;
        
        app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
        app_data->image.data = dm_realloc(app_data->image.data, app_data->image.data_size);
        
        camera_resize(app_data->image.w, app_data->image.h, &app_data->camera, context);
    }
    
    dm_timer_start(&app_data->timer, context);
    const bool camera_updated = camera_update(&app_data->camera, context);
    if(width_changed || height_changed || camera_updated) recreate_rays(app_data);
    app_data->ray_creation_t = dm_timer_elapsed_ms(&app_data->timer, context);
    
    // update image
    dm_timer_start(&app_data->timer, context);
    
    float color[N4] = { 1,1,1,1 };
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            color[0] = app_data->clear_color[0];
            color[1] = app_data->clear_color[1];
            color[2] = app_data->clear_color[2];
            color[3] = app_data->clear_color[3];
            
            per_pixel(x,y, color, app_data);
            //trace_ray(r, color, app_data);
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            
            app_data->image.data[x + y * app_data->image.w] = vec4_to_uint32(color);
        }
    }
    
    app_data->image_creation_t = dm_timer_elapsed_ms(&app_data->timer, context);
    
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
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Ray creation (ms)", app_data->ray_creation_t);
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Image creation (ms)", app_data->image_creation_t);
        
        if(nk_tree_push(ctx, NK_TREE_TAB, "Sphere 1", NK_MAXIMIZED))
        {
            nk_layout_row_dynamic(ctx, 30, 3);
            nk_property_float(ctx, "X", -10, &s->pos[0], 10, 0.1f,0.1f);
            nk_property_float(ctx, "Y", -10, &s->pos[1], 10, 0.1f,0.1f);
            nk_property_float(ctx, "Z", -10, &s->pos[2], 10, 0.1f,0.1f);
            
            nk_layout_row_dynamic(ctx, 30, 1);
            nk_property_float(ctx, "Radius", 0.5f, &s->radius, 2.0f, 0.1f,0.1f);
            
            sphere* s = &app_data->spheres[0];
            struct nk_colorf bg;
            bg.r = s->albedo[0]; bg.g = s->albedo[1]; bg.b = s->albedo[2]; bg.a = s->albedo[3];
            
            if(nk_combo_begin_color(ctx, nk_rgb_cf(bg), nk_vec2(nk_widget_width(ctx),400)))
            {
                nk_layout_row_dynamic(ctx, 120, 1);
                bg = nk_color_picker(ctx, bg, NK_RGBA);
                
                nk_layout_row_dynamic(ctx, 25, 1);
                bg.r = nk_propertyf(ctx, "#R:", 0, bg.r, 1.0f, 0.01f, 0.0005f);
                bg.g = nk_propertyf(ctx, "#B:", 0, bg.b, 1.0f, 0.01f, 0.0005f);
                bg.b = nk_propertyf(ctx, "#G:", 0, bg.g, 1.0f, 0.01f, 0.0005f);
                bg.a = nk_propertyf(ctx, "#A:", 0, bg.a, 1.0f, 0.01f, 0.0005f);
                
                nk_combo_end(ctx);
            }
            nk_tree_pop(ctx);
            
            s->albedo[0] = bg.r; s->albedo[1] = bg.g; s->albedo[2] = bg.b; s->albedo[3] = bg.a;
        }
        
    }
    nk_end(ctx);
    
    return true;
}
