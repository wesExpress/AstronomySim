#include "dm.h"
#include "camera.h"
#include <float.h>

typedef struct ray_t
{
    dm_vec3 origin;
    dm_vec3 direction;
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
    float   radius;
    
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
typedef struct sphere_data_t
{
    DM_ALIGN(16) float x[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float y[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float z[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float radius[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float radius_2[MAX_SPHERE_COUNT];
    
    dm_vec4 albedo[MAX_SPHERE_COUNT];
    
    uint32_t count;
} sphere_data;

typedef struct mt_data_t
{
    uint32_t     y_incr, width;
    dm_vec4      color;
    dm_vec3      ray_pos;
    
    sphere_data* spheres;
    uint32_t*    image_data;
    dm_vec4*     ray_dirs;
} mt_data;

#define NUM_TASKS 8
typedef struct application_data_t
{
    app_handles handles;
    app_image   image;
    
    basic_camera camera;
    
    dm_vec4 clear_color;
    
    // timings
    dm_timer timer;
    double image_creation_t, ray_creation_t;
    double seconds_timer;
    uint32_t rays_processed;
    
    // mt stuff
    dm_threadpool  threadpool;
    dm_thread_task tasks[NUM_TASKS];
    mt_data        thread_data[NUM_TASKS];
    
    // ray directions
    dm_vec4* ray_dirs;
    
    // objects
    sphere_data spheres;
} application_data;

/*******
HELPERS
*********/
void make_sphere(const float x, const float y, const float z, const float radius, const float r, const float g, const float b, const float a, application_data* app_data)
{
    sphere_data* spheres = &app_data->spheres;
    const uint32_t index = spheres->count;
    
    spheres->x[index] = x;
    spheres->y[index] = y;
    spheres->z[index] = z;
    
    spheres->radius[index]   = radius;
    spheres->radius_2[index] = radius * radius;
    
    dm_vec4_set_from_floats(r,g,b,a, spheres->albedo[index]);
    
    spheres->count++;
}

void make_random_sphere(application_data* app_data, dm_context* context)
{
    sphere_data* spheres = &app_data->spheres;
    const uint32_t index = spheres->count;
    
    const float x = dm_random_float_range(-10,10,context);
    const float y = dm_random_float_range(-10,10,context);
    const float z = dm_random_float_range(-10,10,context);
    
    spheres->x[index] = x;
    spheres->y[index] = y;
    spheres->z[index] = z;
    
    const float radius = dm_random_float_range(0.1f,2.0f, context);
    
    spheres->radius[index]   = radius;
    spheres->radius_2[index] = radius * radius;
    
    const float r = dm_random_float(context);
    const float g = dm_random_float(context);
    const float b = dm_random_float(context);
    
    spheres->albedo[index][0] = r;
    spheres->albedo[index][1] = g;
    spheres->albedo[index][2] = b;
    spheres->albedo[index][3] = 1;
    
    spheres->count++;
}

uint32_t vec4_to_uint32(const float vec[4])
{
    const uint8_t r = (uint8_t)(255.0f * vec[0]);
    const uint8_t g = (uint8_t)(255.0f * vec[1]);
    const uint8_t b = (uint8_t)(255.0f * vec[2]);
    const uint8_t a = (uint8_t)(255.0f * vec[3]);
    
    uint32_t result = (a << 24) | (b << 16) | (g << 8) | r;
    return result;
}

hit_payload closest_hit(const ray r, float hit_distance, int obj_index, dm_vec4 color, sphere_data* spheres)
{
    hit_payload payload = {
        .hit_distance=hit_distance,
        .obj_index=obj_index
    };
    
    dm_vec3 origin;
    origin[0] = r.origin[0] - spheres->x[obj_index];
    origin[1] = r.origin[1] - spheres->y[obj_index];
    origin[2] = r.origin[2] - spheres->z[obj_index];
    
    dm_vec3 hit_point;
    dm_vec3_scale(r.direction, hit_distance, hit_point);
    dm_vec3_add_vec3(origin, hit_point, payload.world_position);
    dm_vec3_norm(payload.world_position, payload.world_normal);
    
    payload.world_position[0] += spheres->x[obj_index];
    payload.world_position[1] += spheres->y[obj_index];
    payload.world_position[2] += spheres->z[obj_index];
    
    return payload;
}

hit_payload miss(const ray r)
{
    hit_payload payload = {
        .hit_distance=-1
    };
    
    return payload;
}

hit_payload trace_ray2(const ray r, float color[4], sphere_data* spheres)
{
    dm_mm_float origin_x, origin_y, origin_z;
    //dm_mm_float sphere_x, sphere_y, sphere_z;
    dm_mm_float sphere_r2;
    dm_mm_float b, c, d, d_sqrt, b2;
    dm_mm_float d_mask, hit_mask, final_mask, close_mask;
    dm_mm_float closest_t;
    
    const dm_mm_float ray_origin_x = dm_mm_set1_ps(r.origin[0]);
    const dm_mm_float ray_origin_y = dm_mm_set1_ps(r.origin[1]);
    const dm_mm_float ray_origin_z = dm_mm_set1_ps(r.origin[2]);
    
    const dm_mm_float ray_dir_x = dm_mm_set1_ps(r.direction[0]);
    const dm_mm_float ray_dir_y = dm_mm_set1_ps(r.direction[1]);
    const dm_mm_float ray_dir_z = dm_mm_set1_ps(r.direction[2]);
    
    static int starting_indices[] = { 0,1,2,3 };
    
    dm_mm_int indices = dm_mm_load_i(starting_indices);
    
    const dm_mm_float zeros     = dm_mm_set1_ps(0);
    const dm_mm_float neg_halfs = dm_mm_set1_ps(-0.5f);
    const dm_mm_float twos      = dm_mm_set1_ps(2.0f);
    const dm_mm_float fours     = dm_mm_set1_ps(4.0f);
    
    const dm_mm_int fours_i      = dm_mm_set1_i(4);
    
    dm_mm_int   hit_index = dm_mm_set1_i(-1);
    dm_mm_float hit_t     = dm_mm_set1_ps(FLT_MAX);
    
    for(uint32_t i=0; i<spheres->count; i+=4)
    {
        origin_x  = dm_mm_load_ps(spheres->x + i);
        origin_y  = dm_mm_load_ps(spheres->y + i);
        origin_z  = dm_mm_load_ps(spheres->z + i);
        
        // o = ray.o - sphere.o
        origin_x = dm_mm_sub_ps(ray_origin_x, origin_x);
        origin_y = dm_mm_sub_ps(ray_origin_y, origin_y);
        origin_z = dm_mm_sub_ps(ray_origin_z, origin_z);
        
        // c = dot(o,o) - sphere.r * sphere.r
        c = dm_mm_mul_ps(origin_x,origin_x);
        c = dm_mm_fmadd_ps(origin_y,origin_y, c);
        c = dm_mm_fmadd_ps(origin_z,origin_z, c);
        //sphere_r2 = dm_mm_load_ps(spheres->radius_2 + i);
        c = dm_mm_sub_ps(c, dm_mm_load_ps(spheres->radius_2 + i));
        
        // b = 2 * dot(o, r.dir)
        b  = dm_mm_mul_ps(origin_x, ray_dir_x);
        b  = dm_mm_fmadd_ps(origin_y,ray_dir_y, b);
        b  = dm_mm_fmadd_ps(origin_z,ray_dir_z, b);
        b  = dm_mm_mul_ps(b, twos);
        b2 = dm_mm_mul_ps(b,b);
        
        // d = b * b - 4 * c
        d = dm_mm_mul_ps(fours, c);
        d = dm_mm_sub_ps(b2, d);
        
        // closest hit point
        d_sqrt = dm_mm_sqrt_ps(d);
        
        // (-b - sqrt(d)) / 2
        closest_t = dm_mm_add_ps(b, d_sqrt);
        closest_t = dm_mm_mul_ps(closest_t, neg_halfs);
        
        // closest_t > 0
        close_mask = dm_mm_lt_ps(closest_t, hit_t);
        hit_mask   = dm_mm_gt_ps(closest_t, zeros);
        hit_mask   = dm_mm_and_ps(hit_mask, close_mask);
        
        // d > 0
        d_mask = dm_mm_gt_ps(d, zeros);
        
        final_mask = dm_mm_and_ps(d_mask, hit_mask);
        
        hit_t     = _mm_blendv_ps(hit_t, closest_t, final_mask);
        hit_index = _mm_blendv_epi8(hit_index, indices, dm_mm_cast_float_to_int(final_mask));
        
        // iterate
        indices = dm_mm_add_i(indices, fours_i);
    }
    
    int     hit_inds[4];
    dm_vec4 hits;
    
    dm_mm_store_ps(hits, hit_t);
    dm_mm_store_i(hit_inds, hit_index);
    
    float nearest_hit = FLT_MAX;
    int   nearest_index = -1;
    
    for(uint32_t i=0; i<4; i++)
    {
        if(hits[i] >= nearest_hit) continue;
        
        nearest_hit   = hits[i];
        nearest_index = hit_inds[i];
    }
    
    switch(nearest_index)
    {
        case -1: return miss(r);
        
        default: return closest_hit(r, nearest_hit, nearest_index, color, spheres);
    }
    
#if 0
    if(nearest_index < 0) return miss(r);
    
    return closest_hit(r, nearest_hit, nearest_index, color, spheres);
#endif
}

hit_payload trace_ray(const ray r, float color[4], sphere_data* spheres)
{
    dm_vec3 origin;
    
    float closest_t;
    float hit_distance = FLT_MAX;
    int   nearest_sphere_index = -1;
    
    float b, c, dis;
    
    for(uint32_t i=0; i<spheres->count; i++)
    {
        origin[0] = r.origin[0] - spheres->x[i];
        origin[1] = r.origin[1] - spheres->y[i];
        origin[2] = r.origin[2] - spheres->z[i];
        
        b = 2.0f * dm_vec3_dot(origin, r.direction);
        c = dm_vec3_dot(origin, origin) - spheres->radius_2[i];
        
        dis = b * b - 4.0f * c;
        
        // don't hit this sphere
        if(dis < 0) continue;
        
        // are we the closest sphere?
        closest_t = (-b - dm_sqrtf(dis)) * 0.5f;
        if(closest_t > hit_distance || closest_t < 0) continue;
        
        hit_distance = closest_t;
        nearest_sphere_index = i;
    }
    
    if(nearest_sphere_index<0) return miss(r);
    
    return closest_hit(r, hit_distance, nearest_sphere_index, color, spheres);
}

void per_pixel(uint32_t x, uint32_t y, dm_vec4 color, dm_vec4 clear_color, dm_vec3 pos, dm_vec3 dir, sphere_data* spheres)
{
    ray r;
    
    r.origin[0] = pos[0];
    r.origin[1] = pos[1];
    r.origin[2] = pos[2];
    
    r.direction[0] = dir[0];
    r.direction[1] = dir[1];
    r.direction[2] = dir[2];
    
    uint32_t bounces = 4;
    float multiplier = 0.5f;
    
    hit_payload payload;
    
    for(uint32_t i=0; i<bounces; i++)
    {
        switch(spheres->count)
        {
            case 1:
            case 2:
            case 3:
            payload = trace_ray(r, color, spheres);
            break;
            
            default:
            payload = trace_ray2(r, color, spheres);
            break;
        }
        
        if(payload.hit_distance<0)
        {
            color[0] += multiplier * clear_color[0];
            color[1] += multiplier * clear_color[1];
            color[2] += multiplier * clear_color[2];
            
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
        dm_vec3 sphere_color;
        dm_vec3_scale(spheres->albedo[payload.obj_index], d, sphere_color);
        
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
    
    uint32_t index;
    
    dm_mm_float w_mm, target_mm, row1,row2,row3,row4;
    dm_mm_float target_x_mm, target_y_mm, target_z_mm;
    dm_mm_float coords_mm, coords_x_mm, coords_y_mm, coords_z_mm, coords_w_mm;
    
    //////////// camera matrices are constant, so do them outside
    dm_mm_float proj_row1 = dm_mm_load_ps(app_data->camera.inv_proj[0]);
    dm_mm_float proj_row2 = dm_mm_load_ps(app_data->camera.inv_proj[1]);
    dm_mm_float proj_row3 = dm_mm_load_ps(app_data->camera.inv_proj[2]);
    dm_mm_float proj_row4 = dm_mm_load_ps(app_data->camera.inv_proj[3]);
    
    dm_mm_transpose_mat4(&proj_row1, &proj_row2, &proj_row3, &proj_row4);
    
    dm_mm_float view_row1 = dm_mm_load_ps(app_data->camera.inv_view[0]);
    dm_mm_float view_row2 = dm_mm_load_ps(app_data->camera.inv_view[1]);
    dm_mm_float view_row3 = dm_mm_load_ps(app_data->camera.inv_view[2]);
    dm_mm_float view_row4 = dm_mm_load_ps(app_data->camera.inv_view[3]);
    
    dm_mm_transpose_mat4(&view_row1, &view_row2, &view_row3, &view_row4);
    //////////////////////
    
    dm_vec4 coords = { 0,0,1,1 };
    
    coords_z_mm = dm_mm_set1_ps(1);
    coords_w_mm = dm_mm_set1_ps(1);
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        index = y * app_data->image.w;
        
        coords_y_mm = dm_mm_set1_ps((float)y);
        coords_y_mm = dm_mm_mul_ps(coords_y_mm, dm_mm_set1_ps(height_f_inv));
        coords_y_mm = dm_mm_mul_ps(coords_y_mm, dm_mm_set1_ps(2.0f));
        coords_y_mm = dm_mm_sub_ps(coords_y_mm, dm_mm_set1_ps(1.0f));
        
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            /////////////////
            coords_x_mm = dm_mm_set1_ps((float)x);
            coords_x_mm = dm_mm_mul_ps(coords_x_mm, dm_mm_set1_ps(width_f_inv));
            coords_x_mm = dm_mm_mul_ps(coords_x_mm, dm_mm_set1_ps(2.0f));
            coords_x_mm = dm_mm_sub_ps(coords_x_mm, dm_mm_set1_ps(1.0f));
            
            target_mm = dm_mm_mul_ps(coords_x_mm, proj_row1);
            target_mm = dm_mm_fmadd_ps(coords_y_mm,proj_row2, target_mm);
            target_mm = dm_mm_fmadd_ps(coords_z_mm,proj_row3, target_mm);
            target_mm = dm_mm_fmadd_ps(coords_w_mm,proj_row4, target_mm);
            
            w_mm = dm_mm_broadcast_w_ps(target_mm);
            target_mm = dm_mm_mul_ps(target_mm, w_mm);
            target_mm = dm_mm_normalize_ps(target_mm);
            
            target_x_mm = dm_mm_broadcast_x_ps(target_mm);
            target_y_mm = dm_mm_broadcast_y_ps(target_mm);
            target_z_mm = dm_mm_broadcast_z_ps(target_mm);
            
            target_mm = dm_mm_mul_ps(target_x_mm, view_row1);
            target_mm = dm_mm_fmadd_ps(target_y_mm,view_row2, target_mm);
            target_mm = dm_mm_fmadd_ps(target_z_mm,view_row3, target_mm);
            
            dm_mm_store_ps(app_data->ray_dirs[index++], target_mm);
            /////
        }
    }
}

void create_image(application_data* app_data)
{
    float color[N4] = { 1,1,1,1 };
    dm_vec3 dir;
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            color[0] = app_data->clear_color[0];
            color[1] = app_data->clear_color[1];
            color[2] = app_data->clear_color[2];
            color[3] = app_data->clear_color[3];
            
            dir[0] = app_data->ray_dirs[x + y * app_data->image.w][0];
            dir[1] = app_data->ray_dirs[x + y * app_data->image.w][1];
            dir[2] = app_data->ray_dirs[x + y * app_data->image.w][2];
            per_pixel(x,y, color, app_data->clear_color, app_data->camera.pos, dir, &app_data->spheres);
            
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            
            app_data->image.data[x + y * app_data->image.w] = vec4_to_uint32(color);
        }
    }
}

void* image_mt_func(void* func_data)
{
    mt_data* data = func_data;
    
    dm_vec4 color = { 1,1,1,1 };
    dm_vec3 dir;
    
    for(uint32_t y=0; y<data->y_incr; y++)
    {
        for(uint32_t x=0; x<data->width; x++)
        {
            color[0] = data->color[0];
            color[1] = data->color[1];
            color[2] = data->color[2];
            color[3] = data->color[3];
            
            dir[0] = data->ray_dirs[x + y * data->width][0];
            dir[1] = data->ray_dirs[x + y * data->width][1];
            dir[2] = data->ray_dirs[x + y * data->width][2];
            per_pixel(x,y, color, data->color, data->ray_pos, dir, data->spheres);
            
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            
            data->image_data[x + y * data->width] = vec4_to_uint32(color);
        }
    }
    
    return NULL;
}

void create_image_mt(application_data* app_data)
{
    const uint32_t y_incr    = app_data->image.h / NUM_TASKS;
    const size_t   data_size = sizeof(uint32_t) * y_incr * app_data->image.w;
    const size_t   ray_size  = sizeof(dm_vec4) * y_incr * app_data->image.w;
    
    uint32_t offset;
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        app_data->thread_data[i].y_incr  = y_incr;
        app_data->thread_data[i].width   = app_data->image.w;
        app_data->thread_data[i].spheres = &app_data->spheres;
        
        app_data->thread_data[i].color[0] = app_data->clear_color[0];
        app_data->thread_data[i].color[1] = app_data->clear_color[1];
        app_data->thread_data[i].color[2] = app_data->clear_color[2];
        app_data->thread_data[i].color[3] = app_data->clear_color[3];
        
        app_data->thread_data[i].ray_pos[0] = app_data->camera.pos[0];
        app_data->thread_data[i].ray_pos[1] = app_data->camera.pos[1];
        app_data->thread_data[i].ray_pos[2] = app_data->camera.pos[2];
        
        offset = y_incr * i;
        
        if(!app_data->thread_data[i].image_data)
        {
            app_data->thread_data[i].image_data = dm_alloc(data_size);
        }
        else
        {
            app_data->thread_data[i].image_data = dm_realloc(app_data->thread_data[i].image_data, data_size);
        }
        dm_memcpy(app_data->thread_data[i].image_data, app_data->image.data + offset * app_data->image.w, data_size);
        
        if(!app_data->thread_data[i].ray_dirs)
        {
            app_data->thread_data[i].ray_dirs = dm_alloc(ray_size);
        }
        else
        {
            app_data->thread_data[i].ray_dirs = dm_realloc(app_data->thread_data[i].ray_dirs, ray_size);
        }
        dm_memcpy(app_data->thread_data[i].ray_dirs, app_data->ray_dirs + offset * app_data->image.w, ray_size);
        
        ///////////////////////////
        app_data->tasks[i].func = image_mt_func;
        app_data->tasks[i].args = &app_data->thread_data[i];
    }
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        dm_threadpool_submit_task(&app_data->tasks[i], &app_data->threadpool);
    }
    
    dm_threadpool_wait_for_completion(&app_data->threadpool);
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        offset = y_incr * i;
        
        dm_memcpy(app_data->image.data + offset * app_data->image.w, app_data->thread_data[i].image_data, data_size);
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
    
    for(uint32_t i=0; i<MAX_SPHERE_COUNT; i++)
    {
        app_data->spheres.x[i] = FLT_MAX;
        app_data->spheres.y[i] = FLT_MAX;
        app_data->spheres.z[i] = FLT_MAX;
        
        app_data->spheres.radius[i] = FLT_MAX;
        app_data->spheres.radius_2[i] = FLT_MAX;
    }
    
    // threadool
    if(!dm_threadpool_create("ray_tracer", 4, &app_data->threadpool)) return false;
    
    // sphere 1
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    make_random_sphere(app_data, context);
    
    // misc
    app_data->clear_color[3] = 1;
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        dm_free(app_data->thread_data[i].ray_dirs);
        dm_free(app_data->thread_data[i].image_data);
    }
    
    dm_threadpool_destroy(&app_data->threadpool);
    
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
    
    //create_image(app_data);
    create_image_mt(app_data);
    
    app_data->image_creation_t = dm_timer_elapsed_ms(&app_data->timer, context);
    app_data->rays_processed = app_data->image.w * app_data->image.h;
    
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
    
    if(nk_begin(ctx, "Ray Trace App", nk_rect(650,50, 300,300), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE | 
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_uint(ctx, "Sphere count", app_data->spheres.count);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Ray creation (ms)", app_data->ray_creation_t);
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Image creation (ms)", app_data->image_creation_t);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        const float num_rays = (float)app_data->rays_processed * 1000.0f / 1e6f / app_data->image_creation_t;
        nk_value_float(ctx, "Rays processed (millions per second)", num_rays);
    }
    nk_end(ctx);
    
    return true;
}
