#include "dm.h"
#include "camera.h"

#include "default_render.h"
#include "debug_render.h"

#include "octree.h"

#include <assert.h>
#include <string.h>

#define ARRAY_LENGTH 16000
#define BLOCK_SIZE   128

#define FIXED_DT 0.008333

//#define USE_ASTRO_UNITS

#ifndef USE_ASTRO_UNITS
#define G 6.67e-11f                        // N m^2 / kg^2

#define WORLD_CUBE_SIZE 50                 // m

#define MASS_SCALE      1e10f              // kg
#define SMBH_MASS       MASS_SCALE * 1.f   // kg
#else
#define G          4.3e-3f                 // pc Msun^-1 km^2 / s^2

#define WORLD_CUBE_SIZE 10                 // pc

#define MASS_SCALE 1.f                     // Msun
#define SMBH_MASS  MASS_SCALE * 1e3f       // MSUN
#endif

#define DRAW_BILBOARDS

/*
 object data
 */
typedef struct star_data_soa_t
{
    DM_ALIGN(16) float pos_x[ARRAY_LENGTH];
    DM_ALIGN(16) float pos_y[ARRAY_LENGTH];
    DM_ALIGN(16) float pos_z[ARRAY_LENGTH];
    
    DM_ALIGN(16) float mass[ARRAY_LENGTH];
    
    DM_ALIGN(16) float vel_x[ARRAY_LENGTH];
    DM_ALIGN(16) float vel_y[ARRAY_LENGTH];
    DM_ALIGN(16) float vel_z[ARRAY_LENGTH];
    
    DM_ALIGN(16) float force_x[ARRAY_LENGTH];
    DM_ALIGN(16) float force_y[ARRAY_LENGTH];
    DM_ALIGN(16) float force_z[ARRAY_LENGTH];
} star_data_soa;

typedef struct transform_gpu_element_t
{
    dm_vec3 pos;
    float padding;
} transform_gpu_element;

typedef struct physics_gpu_element_t
{
    dm_vec3 vel;
    float   mass;
    dm_vec3 force;
    float   padding;
} physics_gpu_element;

/*
 app data
 */
#define MAX_INSTS_PER_FRAME 2048
typedef struct application_data_t
{
    dm_compute_handle gravity, physics;
    dm_compute_handle transform_b, physics_b;
    
    
    
    basic_camera camera;
    
    double gravity_timing, physics_timing, instance_creation_timing;
    double accumulated_time;
    
    uint32_t physics_iters, mesh_index_count, mesh_vertex_count;
    
    bool pause;
    
    uint32_t     octree_node_count;
    dm_vec3      octree_center;
    float        octree_half_extents;
    uint16_t     octree_depth;
    octree_node* octree;
    
    star_data_soa* star_data;
    transform_gpu_element* transform_gpu_data;
    physics_gpu_element* physics_gpu_data;
    
    // render passes
    void* default_render_data;
    void* debug_render_data;
} application_data;

/*
 DarkMatter interface
 */
void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->vsync = true;
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    app_data->star_data = dm_alloc(sizeof(star_data_soa));
    app_data->transform_gpu_data = dm_alloc(sizeof(transform_gpu_element) * ARRAY_LENGTH);
    app_data->physics_gpu_data = dm_alloc(sizeof(physics_gpu_element) * ARRAY_LENGTH);
    
    dm_timer oct_t = { 0 };
    
    // compute data
    {
        dm_compute_shader_desc gravity_desc = { 0 };
#ifdef DM_METAL
        strcpy(gravity_desc.path, "assets/shaders/gravity_compute.metallib");
        strcpy(gravity_desc.function, "gravity_calc");
#elif defined(DM_DIRECTX)
        strcpy(gravity_desc.path, "assets/shaders/gravity_compute.fxc");
#else
        DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
        assert(false);
#endif
        
        dm_compute_shader_desc physics_desc = { 0 };
#ifdef DM_METAL
        strcpy(physics_desc.path, "assets/shaders/physics_compute.metallib");
        strcpy(physics_desc.function, "physics_update");
#elif defined(DM_DIRECTX)
        strcpy(physics_desc.path, "assets/shaders/physics_compute.fxc");
#else
        DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
        assert(false);
#endif
        
        if(!dm_compute_create_shader(gravity_desc, &app_data->gravity, context)) return false;
        if(!dm_compute_create_shader(physics_desc, &app_data->physics, context)) return false;
        
        static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
        
        if(!dm_compute_create_buffer(sizeof(transform_gpu_element) * ARRAY_LENGTH, sizeof(transform_gpu_element), DM_COMPUTE_BUFFER_TYPE_READ_WRITE, &app_data->transform_b, context)) return false;
        if(!dm_compute_create_buffer(sizeof(physics_gpu_element) * ARRAY_LENGTH, sizeof(physics_gpu_element), DM_COMPUTE_BUFFER_TYPE_READ_WRITE, &app_data->physics_b, context)) return false;
        
        dm_vec3 p, dir;
        float radius;
        const dm_vec3 up = { 0,1,0 };
        
        app_data->star_data->mass[0] = SMBH_MASS;
        
        app_data->star_data->pos_x[0] = 0;
        app_data->star_data->pos_y[0] = 0;
        app_data->star_data->pos_z[0] = 0;
        
        app_data->star_data->vel_x[0] = 0;
        app_data->star_data->vel_y[0] = 0;
        app_data->star_data->vel_z[0] = 0;
        
        float r_i, vc;
        
        dm_vec3 min_p = { 0 };
        dm_vec3 max_p = { 0 };
        
        for(uint32_t i=1; i<ARRAY_LENGTH; i++)
        {
            p[0] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[1] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[2] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            
            min_p[0] = p[0] < min_p[0] ? p[0] : min_p[0];
            min_p[1] = p[1] < min_p[1] ? p[1] : min_p[1];
            min_p[2] = p[2] < min_p[2] ? p[2] : min_p[2];
            
            max_p[0] = p[0] > max_p[0] ? p[0] : max_p[0];
            max_p[1] = p[1] > max_p[1] ? p[1] : max_p[1];
            max_p[2] = p[2] > max_p[2] ? p[2] : max_p[2];
        
            app_data->star_data->pos_x[i] = p[0];
            app_data->star_data->pos_y[i] = p[1];
            app_data->star_data->pos_z[i] = p[2];
            
            app_data->star_data->mass[i] = dm_random_float(context) * MASS_SCALE;
            
            //p[1] = 0;
            dm_vec3_negate(p, p);
            
            dm_vec3_cross(p, up, dir);
            dm_vec3_norm(dir, dir);
            
            r_i  = app_data->star_data->pos_x[i] * app_data->star_data->pos_x[i];
            r_i += app_data->star_data->pos_y[i] * app_data->star_data->pos_y[i];
            r_i += app_data->star_data->pos_z[i] * app_data->star_data->pos_z[i];
            r_i  = dm_sqrtf(r_i);
            
            vc = G * app_data->star_data->mass[0];
            vc /= r_i;
            vc = dm_sqrtf(vc) * 10.f;
            
            app_data->star_data->vel_x[i] = dir[0] * vc;
            app_data->star_data->vel_y[i] = dir[1] * vc;
            app_data->star_data->vel_z[i] = dir[2] * vc;
        }
        
        // set up octree
        app_data->octree = dm_alloc(sizeof(octree_node));
        
        dm_vec3 extents, center;
        dm_vec3_sub_vec3(max_p, min_p, extents);
        dm_vec3_fabs(extents, extents);
        
        float half_extents = extents[0];
        half_extents = DM_MAX(half_extents, extents[1]);
        half_extents = DM_MAX(half_extents, extents[2]);
        half_extents *= 0.5f;
        
        dm_vec3_scale(extents, 0.5f, extents);
        dm_vec3_sub_vec3(max_p, extents, app_data->octree[0].center);
        app_data->octree[0].half_extents = half_extents;
        app_data->octree[0].first_child = -1;
        app_data->octree[0].obj_index = -1;
        
        app_data->octree_node_count = 1;
        app_data->octree_depth = 0;
        
        dm_timer_start(&oct_t, context);
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
            octree_insert(i, 0, 0, &app_data->octree_depth, &app_data->octree, &app_data->octree_node_count, app_data->star_data->pos_x, app_data->star_data->pos_y, app_data->star_data->pos_z);
        }
    }
    double time = dm_timer_elapsed_ms(&oct_t, context);
    DM_LOG_INFO("Inserting %u objects into octree took: %0.2lf ms", ARRAY_LENGTH, time);

    // render passes
    {
        if(!default_render_init(ARRAY_LENGTH, &app_data->default_render_data, context)) return false;
        if(!debug_render_pass_init(&app_data->debug_render_data, context))              return false;
    }
    
    // camera
    dm_vec3 cam_pos = { 0,0,WORLD_CUBE_SIZE };
    float move_sens = 10.f;
    dm_vec3 cam_for = { 0,0,-1 };
    
    camera_init(cam_pos, cam_for, 0.001f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), move_sens, 1.0f, &app_data->camera);
    
    app_data->pause = true;
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_free(app_data->star_data);
    dm_free(app_data->transform_gpu_data);
    dm_free(app_data->physics_gpu_data);
    
    if(app_data->octree) dm_free(app_data->octree);
    
    default_render_shutdown(&app_data->default_render_data, context);
    debug_render_pass_shutdown(&app_data->debug_render_data, context);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // camera
    camera_update(&app_data->camera, context);
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE, context)) app_data->pause = !app_data->pause;
    
    dm_vec3 p = { 0,0,0 };
    dm_vec4 d = { 10,10,10 };
    dm_vec4 c = { 1,0,0,1 };
    dm_quat o = { 0,0.5f,0,1 };
    debug_render_aabb(p, d, c, &app_data->debug_render_data, context);
    debug_render_cube(p, d, o, c, &app_data->debug_render_data, context);
    
    if(app_data->pause) return true;
    
    app_data->accumulated_time += context->delta;
    app_data->physics_timing = 0;
    app_data->physics_iters  = 0;
    
    const int group_count = (int)dm_ceil((float)ARRAY_LENGTH / (float)BLOCK_SIZE);
    
    // update gpu buffers and get octree data
    dm_vec3 min = { 0 };
    dm_vec3 max = { 0 };
    
    for (uint32_t i = 0; i < ARRAY_LENGTH; i++)
    {
        app_data->transform_gpu_data[i].pos[0] = app_data->star_data->pos_x[i];
        app_data->transform_gpu_data[i].pos[1] = app_data->star_data->pos_y[i];
        app_data->transform_gpu_data[i].pos[2] = app_data->star_data->pos_z[i];
        
        app_data->physics_gpu_data[i].vel[0] = app_data->star_data->vel_x[i];
        app_data->physics_gpu_data[i].vel[1] = app_data->star_data->vel_y[i];
        app_data->physics_gpu_data[i].vel[2] = app_data->star_data->vel_z[i];
        app_data->physics_gpu_data[i].mass = app_data->star_data->mass[i];
        app_data->physics_gpu_data[i].force[0] = 0;
        app_data->physics_gpu_data[i].force[1] = 0;
        app_data->physics_gpu_data[i].force[2] = 0;
        
        // octree
        min[0] = app_data->star_data->pos_x[i] < min[0] ? app_data->star_data->pos_x[i] : min[0];
        min[1] = app_data->star_data->pos_y[i] < min[1] ? app_data->star_data->pos_y[i] : min[1];
        min[2] = app_data->star_data->pos_z[i] < min[2] ? app_data->star_data->pos_z[i] : min[2];
        
        max[0] = app_data->star_data->pos_x[i] > max[0] ? app_data->star_data->pos_x[i] : max[0];
        max[1] = app_data->star_data->pos_y[i] > max[1] ? app_data->star_data->pos_y[i] : max[1];
        max[2] = app_data->star_data->pos_z[i] > max[2] ? app_data->star_data->pos_z[i] : max[2];
    }
    
    dm_vec3 extents;
    dm_vec3_sub_vec3(max, min, extents);
    dm_vec3_fabs(extents, extents);
    dm_vec3_scale(extents, 0.5f, extents);
    app_data->octree[0].half_extents = extents[0];
    app_data->octree[0].half_extents = app_data->octree[0].half_extents > extents[1] ? app_data->octree[0].half_extents : extents[1];
    app_data->octree[0].half_extents = app_data->octree[0].half_extents > extents[2] ? app_data->octree[0].half_extents : extents[2];
    dm_vec3_sub_vec3(max, extents, app_data->octree[0].center);
    app_data->octree[0].first_child = -1;
    app_data->octree[0].obj_index   = -1;
    
    app_data->octree = dm_realloc(app_data->octree, sizeof(octree_node));
    app_data->octree_node_count = 1;
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        octree_insert(i, 0, 0, &app_data->octree_depth, &app_data->octree, &app_data->octree_node_count, app_data->star_data->pos_x, app_data->star_data->pos_y, app_data->star_data->pos_z);
    }
    
    // gravity force calculation
    {
        if(!dm_compute_command_bind_shader(app_data->gravity, context)) return false;
        
        if (!dm_compute_command_update_buffer(app_data->transform_b, app_data->transform_gpu_data, sizeof(transform_gpu_element) * ARRAY_LENGTH, 0, context)) return false;
        if (!dm_compute_command_update_buffer(app_data->physics_b, app_data->physics_gpu_data, sizeof(physics_gpu_element) * ARRAY_LENGTH, 0, context)) return false;
        
        if (!dm_compute_command_bind_buffer(app_data->transform_b, 0, 0, context)) return false;
        if (!dm_compute_command_bind_buffer(app_data->physics_b, 0, 1, context)) return false;
        
        dm_timer t = { 0 };
        dm_timer_start(&t, context);
        if(!dm_compute_command_dispatch(group_count,1,1, BLOCK_SIZE,1,1, context)) return false;
        app_data->gravity_timing = dm_timer_elapsed_ms(&t, context);
        
        if(!app_data->physics_gpu_data) return false;
    }
    
    // physics update
    while(app_data->accumulated_time >= FIXED_DT)
    {
        app_data->physics_iters++;
        
        if(!dm_compute_command_bind_shader(app_data->physics, context)) return false;
        
        if (!dm_compute_command_bind_buffer(app_data->transform_b, 0, 0, context)) return false;
        if (!dm_compute_command_bind_buffer(app_data->physics_b, 0, 1, context)) return false;
        
        dm_timer t = { 0 };
        dm_timer_start(&t, context);
        if(!dm_compute_command_dispatch(group_count,1,1, BLOCK_SIZE,1,1, context)) return false;
        app_data->physics_timing += dm_timer_elapsed_ms(&t, context);
        
        app_data->accumulated_time -= FIXED_DT;
    }
    
    dm_memcpy(app_data->transform_gpu_data, dm_compute_command_get_buffer_data(app_data->transform_b, context), sizeof(transform_gpu_element) * ARRAY_LENGTH);
    dm_memcpy(app_data->physics_gpu_data, dm_compute_command_get_buffer_data(app_data->physics_b, context), sizeof(physics_gpu_element) * ARRAY_LENGTH);
    
    if (!app_data->transform_gpu_data) return false;
    if (!app_data->physics_gpu_data) return false;
    
    // get the star data in order
    transform_gpu_element* transform = NULL;
    physics_gpu_element*   physics = NULL;
    for (uint32_t i = 0; i < ARRAY_LENGTH; i++)
    {
        transform = &app_data->transform_gpu_data[i];
        physics   = &app_data->physics_gpu_data[i];
        
        app_data->star_data->pos_x[i] = transform->pos[0];
        app_data->star_data->pos_y[i] = transform->pos[1];
        app_data->star_data->pos_z[i] = transform->pos[2];
        
        app_data->star_data->vel_x[i] = physics->vel[0];
        app_data->star_data->vel_y[i] = physics->vel[1];
        app_data->star_data->vel_z[i] = physics->vel[2];
        
        app_data->star_data->force_x[i] = physics->force[0];
        app_data->star_data->force_y[i] = physics->force[1];
        app_data->star_data->force_z[i] = physics->force[2];
    }
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    if(!default_render_render(ARRAY_LENGTH, app_data->camera.view_proj, app_data->camera.inv_view, app_data->star_data->pos_x, app_data->star_data->pos_y, app_data->star_data->pos_z, &app_data->default_render_data, context)) return false;
    if(!debug_render_pass_render(&app_data->debug_render_data, app_data->camera.inv_view, context)) return false;
    
    // imgui
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    if(nk_begin(ctx, "Timings", nk_rect(100, 100, 250, 250), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Num objects", ARRAY_LENGTH);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Gravity compute (ms)", app_data->gravity_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Physics compute (average) (ms)", app_data->physics_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Physics iterations", app_data->physics_iters);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Instance creation (ms)", app_data->instance_creation_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Previous frame (ms)", context->delta * 1000.0f);
    }
    nk_end(ctx);
    
    return true;
}
