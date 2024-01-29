#include "dm.h"
#include "camera.h"

#include "gravity.h"
#include "physics.h"

#include "default_render.h"
#include "debug_render.h"

#define OCTREE_LIMIT_DEPTH
#include "octree.h"

#include <assert.h>
#include <string.h>
#include <float.h>

#define ARRAY_LENGTH 30000
#define BLOCK_SIZE   256

#define OCTREE_FUDGE_FACTOR 0.001f;

//#define USE_ASTRO_UNITS

#ifndef USE_ASTRO_UNITS
#define G 6.67e-11f                        // N m^2 / kg^2

#define WORLD_CUBE_SIZE 50                 // m

#define MASS_SCALE      1e10f              // kg
#define SMBH_MASS       MASS_SCALE * 1e4f  // kg
#else
#define G  4.3e-3f                         // pc Msun^-1 km^2 / s^2

#define WORLD_CUBE_SIZE 10                 // pc

#define MASS_SCALE 1.f                     // Msun
#define SMBH_MASS  MASS_SCALE * 1e3f       // MSUN
#endif

#define DRAW_BILBOARDS

/*
 object data
 */
typedef struct transform_soa_t
{
    DM_ALIGN(16) float pos_x[ARRAY_LENGTH];
    DM_ALIGN(16) float pos_y[ARRAY_LENGTH];
    DM_ALIGN(16) float pos_z[ARRAY_LENGTH];
} transform_soa;

typedef struct physics_soa_t
{
    DM_ALIGN(16) float vel_x[ARRAY_LENGTH];
    DM_ALIGN(16) float vel_y[ARRAY_LENGTH];
    DM_ALIGN(16) float vel_z[ARRAY_LENGTH];
    
    DM_ALIGN(16) float mass[ARRAY_LENGTH];
    DM_ALIGN(16) float inv_mass[ARRAY_LENGTH];
    
    DM_ALIGN(16) float force_x[ARRAY_LENGTH];
    DM_ALIGN(16) float force_y[ARRAY_LENGTH];
    DM_ALIGN(16) float force_z[ARRAY_LENGTH];
    
    DM_ALIGN(16) float accel_x[ARRAY_LENGTH];
    DM_ALIGN(16) float accel_y[ARRAY_LENGTH];
    DM_ALIGN(16) float accel_z[ARRAY_LENGTH];
} physics_soa;

typedef struct transform_t
{
    dm_vec3 pos;
    float padding;
} transform;

typedef struct physics_t
{
    dm_vec3 vel;
    float   mass;
    
    dm_vec3 force;
    float   inv_mass;
    
    dm_vec3 accel;
    float   padding;
} physics;

typedef struct transform_aos_t
{
    transform data[ARRAY_LENGTH];
} transform_aos;

typedef struct physics_aos_t
{
    physics data[ARRAY_LENGTH];
} physics_aos;

/*
 app data
 */
#define MAX_INSTS_PER_FRAME 2048
typedef struct application_data_t
{
    dm_compute_handle transform_b, physics_b;
    
    basic_camera camera;
    
    bool pause;
    
    uint32_t     octree_node_count;
    dm_vec3      octree_center;
    float        octree_half_extents;
    uint32_t     octree_depth;
    octree_node* octree;
    
    // data
    transform_soa* transform_cpu;
    physics_soa*   physics_cpu;
    
    transform_aos* transform_gpu;
    physics_aos*   physics_gpu;
    
    // compute passes
    void* grav_data;
    void* physics_data;
    
    // render passes
    void* default_render_data;
    void* debug_render_data;
} application_data;

void gravity_bh(transform_soa* transforms, physics_soa* physics, octree_node** octree);

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
    
    app_data->transform_cpu = dm_alloc(sizeof(transform_soa));
    app_data->physics_cpu   = dm_alloc(sizeof(physics_soa));
    app_data->transform_gpu = dm_alloc(sizeof(transform_aos));
    app_data->physics_gpu   = dm_alloc(sizeof(physics_aos));
    
    // compute data
    {
        if(!gravity_init(&app_data->grav_data, context))    return false;
        if(!physics_init(&app_data->physics_data, context)) return false;
        
        static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
        
        if(!dm_compute_create_buffer(sizeof(transform_aos), sizeof(transform), DM_COMPUTE_BUFFER_TYPE_READ_WRITE, &app_data->transform_b, context)) return false;
        if(!dm_compute_create_buffer(sizeof(physics_aos), sizeof(physics), DM_COMPUTE_BUFFER_TYPE_READ_WRITE, &app_data->physics_b, context)) return false;
        
        dm_vec3 p, dir;
        float radius;
        const dm_vec3 up = { 0,1,0 };
        
        app_data->physics_cpu->mass[0]     = SMBH_MASS;
        app_data->physics_cpu->inv_mass[0] = 1.f / SMBH_MASS;
        
        app_data->transform_cpu->pos_x[0] = 0;
        app_data->transform_cpu->pos_y[0] = 0;
        app_data->transform_cpu->pos_z[0] = 0;
        
        app_data->physics_cpu->vel_x[0] = 0;
        app_data->physics_cpu->vel_y[0] = 0;
        app_data->physics_cpu->vel_z[0] = 0;
        
        app_data->physics_cpu->accel_x[0] = 0;
        app_data->physics_cpu->accel_y[0] = 0;
        app_data->physics_cpu->accel_z[0] = 0;
        
        float r_i, vc;
        
        dm_vec3 min_p = { FLT_MAX,FLT_MAX,FLT_MAX };
        dm_vec3 max_p = { -FLT_MAX,-FLT_MAX,-FLT_MAX };
        
        for(uint32_t i=1; i<ARRAY_LENGTH; i++)
        {
            p[0] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[1] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[2] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            
            min_p[0] = p[0] <= min_p[0] ? p[0] : min_p[0];
            min_p[1] = p[1] <= min_p[1] ? p[1] : min_p[1];
            min_p[2] = p[2] <= min_p[2] ? p[2] : min_p[2];
            
            max_p[0] = p[0] > max_p[0] ? p[0] : max_p[0];
            max_p[1] = p[1] > max_p[1] ? p[1] : max_p[1];
            max_p[2] = p[2] > max_p[2] ? p[2] : max_p[2];
            
            app_data->transform_cpu->pos_x[i] = p[0];
            app_data->transform_cpu->pos_y[i] = p[1];
            app_data->transform_cpu->pos_z[i] = p[2];
            
            app_data->physics_cpu->mass[i]     = dm_random_float(context) * MASS_SCALE;
            app_data->physics_cpu->inv_mass[i] = 1.f / app_data->physics_cpu->mass[i]; 
            
            //p[1] = 0;
            dm_vec3_negate(p, p);
            
            dm_vec3_cross(p, up, dir);
            dm_vec3_norm(dir, dir);
            
            r_i  = app_data->transform_cpu->pos_x[i] * app_data->transform_cpu->pos_x[i];
            r_i += app_data->transform_cpu->pos_y[i] * app_data->transform_cpu->pos_y[i];
            r_i += app_data->transform_cpu->pos_z[i] * app_data->transform_cpu->pos_z[i];
            r_i  = dm_sqrtf(r_i);
            
            vc  = G * app_data->physics_cpu->mass[0];
            vc /= r_i;
            vc  = dm_sqrtf(vc);
            
            app_data->physics_cpu->vel_x[i] = dir[0] * vc;
            app_data->physics_cpu->vel_y[i] = dir[1] * vc;
            app_data->physics_cpu->vel_z[i] = dir[2] * vc;
            
            app_data->physics_cpu->accel_x[i] = 0;
            app_data->physics_cpu->accel_y[i] = 0;
            app_data->physics_cpu->accel_z[i] = 0;
        }
        
        // set up octree
        app_data->octree = dm_alloc(sizeof(octree_node));
        
        dm_vec3 extents, center;
        dm_vec3_sub_vec3(max_p, min_p, extents);
        dm_vec3_fabs(extents, extents);
        dm_vec3_scale(extents, 0.5f, extents);
        
        float half_extents = extents[0];
        half_extents = DM_MAX(half_extents, extents[1]);
        half_extents = DM_MAX(half_extents, extents[2]);
        // fudge factor
        half_extents += OCTREE_FUDGE_FACTOR;
        
        dm_vec3_sub_vec3(max_p, extents, app_data->octree[0].center);
        app_data->octree[0].half_extents = half_extents;
        app_data->octree[0].first_child = -1;
        
        app_data->octree_node_count = 1;
        app_data->octree_depth = 0;
        
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
            if(!octree_insert(i, 0, 0, &app_data->octree_depth, &app_data->octree, &app_data->octree_node_count, app_data->transform_cpu->pos_x, app_data->transform_cpu->pos_y, app_data->transform_cpu->pos_z)) return false;
        }
    }
    
    // render passes
    {
        if(!default_render_init(ARRAY_LENGTH, &app_data->default_render_data, context)) return false;
        if(!debug_render_pass_init(&app_data->debug_render_data, context))              return false;
    }
    
    // camera
    dm_vec3 cam_pos = { 0,0,WORLD_CUBE_SIZE };
    float move_sens = 30.f;
    dm_vec3 cam_for = { 0,0,-1 };
    
    camera_init(cam_pos, cam_for, 0.001f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), move_sens, 1.0f, &app_data->camera);
    
    app_data->pause = true;
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_free(app_data->transform_cpu);
    dm_free(app_data->transform_gpu);
    
    dm_free(app_data->physics_cpu);
    dm_free(app_data->physics_gpu);
    
    if(app_data->octree) dm_free(app_data->octree);
    
    gravity_shutdown(&app_data->grav_data, context);
    physics_shutdown(&app_data->physics_data, context);
    
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
    
    if(app_data->pause) return true;
    
    dm_timer timer = { 0 };
    
    const uint32_t group_count = (uint32_t)dm_ceil((float)ARRAY_LENGTH / (float)BLOCK_SIZE);
    
    // update gpu buffers and get octree data
    dm_vec3 min = { FLT_MAX,FLT_MAX,FLT_MAX };
    dm_vec3 max = { -FLT_MAX,-FLT_MAX,-FLT_MAX };
    
    for (uint32_t i = 0; i < ARRAY_LENGTH; i++)
    {
        app_data->transform_gpu->data[i].pos[0] = app_data->transform_cpu->pos_x[i];
        app_data->transform_gpu->data[i].pos[1] = app_data->transform_cpu->pos_y[i];
        app_data->transform_gpu->data[i].pos[2] = app_data->transform_cpu->pos_z[i];
        
        app_data->physics_gpu->data[i].vel[0]   = app_data->physics_cpu->vel_x[i];
        app_data->physics_gpu->data[i].vel[1]   = app_data->physics_cpu->vel_y[i];
        app_data->physics_gpu->data[i].vel[2]   = app_data->physics_cpu->vel_z[i];
        app_data->physics_gpu->data[i].mass     = app_data->physics_cpu->mass[i];
        app_data->physics_gpu->data[i].inv_mass = 1.f / app_data->physics_cpu->mass[i];
        
        app_data->physics_gpu->data[i].accel[0] = app_data->physics_cpu->accel_x[i];
        app_data->physics_gpu->data[i].accel[1] = app_data->physics_cpu->accel_y[i];
        app_data->physics_gpu->data[i].accel[2] = app_data->physics_cpu->accel_z[i];
        
        app_data->physics_gpu->data[i].force[0] = 0;
        app_data->physics_gpu->data[i].force[1] = 0;
        app_data->physics_gpu->data[i].force[2] = 0;
        
        // octree
        min[0] = app_data->transform_cpu->pos_x[i] < min[0] ? app_data->transform_cpu->pos_x[i] : min[0];
        min[1] = app_data->transform_cpu->pos_y[i] < min[1] ? app_data->transform_cpu->pos_y[i] : min[1];
        min[2] = app_data->transform_cpu->pos_z[i] < min[2] ? app_data->transform_cpu->pos_z[i] : min[2];
        
        max[0] = app_data->transform_cpu->pos_x[i] > max[0] ? app_data->transform_cpu->pos_x[i] : max[0];
        max[1] = app_data->transform_cpu->pos_y[i] > max[1] ? app_data->transform_cpu->pos_y[i] : max[1];
        max[2] = app_data->transform_cpu->pos_z[i] > max[2] ? app_data->transform_cpu->pos_z[i] : max[2];
    }
    
    float half_extents;
    dm_vec3 extents, center;
    dm_vec3_sub_vec3(max, min, extents);
    dm_vec3_fabs(extents, extents);
    dm_vec3_scale(extents, 0.5f, extents);
    half_extents = extents[0];
    half_extents = DM_MAX(half_extents, extents[1]);
    half_extents = DM_MAX(half_extents, extents[2]);
    // fudge factor
    half_extents += OCTREE_FUDGE_FACTOR;
    
    dm_timer_start(&timer, context);
    octree_node_destroy(0, &app_data->octree);
    dm_free(app_data->octree);
    app_data->octree = dm_alloc(sizeof(octree_node));
    app_data->octree_node_count = 1;
    app_data->octree_depth = 0;
    
    app_data->octree[0].half_extents = half_extents;
    dm_vec3_sub_vec3(max, extents, app_data->octree[0].center);
    app_data->octree[0].first_child = -1;
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        if(!octree_insert(i, 0, 0, &app_data->octree_depth, &app_data->octree, &app_data->octree_node_count, app_data->transform_cpu->pos_x, app_data->transform_cpu->pos_y, app_data->transform_cpu->pos_z)) return false;
    }
    
    static bool draw_octree = false;
    if(dm_input_key_just_pressed(DM_KEY_1, context)) draw_octree = !draw_octree;
    // render octree
    if(draw_octree) octree_render(0, 0, app_data->octree, &app_data->debug_render_data, context);
    
    if (!dm_compute_command_update_buffer(app_data->transform_b, app_data->transform_gpu, sizeof(transform_aos), 0, context)) return false;
    if (!dm_compute_command_update_buffer(app_data->physics_b, app_data->physics_gpu, sizeof(physics_aos), 0, context)) return false;
    
    if(!gravity_run(app_data->transform_b, app_data->physics_b, group_count, BLOCK_SIZE, app_data->grav_data, context)) return false;
    if(!physics_run(app_data->transform_b, app_data->physics_b, group_count, BLOCK_SIZE, app_data->physics_data, context)) return false;
    
    dm_memcpy(app_data->transform_gpu, dm_compute_command_get_buffer_data(app_data->transform_b, context), sizeof(transform_aos));
    dm_memcpy(app_data->physics_gpu, dm_compute_command_get_buffer_data(app_data->physics_b, context), sizeof(physics_aos));
    
    if (!app_data->transform_gpu) return false;
    if (!app_data->physics_gpu) return false;
    
    // get the star data in order
    for (uint32_t i = 0; i < ARRAY_LENGTH; i++)
    {
        app_data->transform_cpu->pos_x[i] = app_data->transform_gpu->data[i].pos[0];
        app_data->transform_cpu->pos_y[i] = app_data->transform_gpu->data[i].pos[1];
        app_data->transform_cpu->pos_z[i] = app_data->transform_gpu->data[i].pos[2];
        
        app_data->physics_cpu->vel_x[i] = app_data->physics_gpu->data[i].vel[0];
        app_data->physics_cpu->vel_y[i] = app_data->physics_gpu->data[i].vel[1];
        app_data->physics_cpu->vel_z[i] = app_data->physics_gpu->data[i].vel[2];
        
        app_data->physics_cpu->force_x[i] = app_data->physics_gpu->data[i].force[0];
        app_data->physics_cpu->force_y[i] = app_data->physics_gpu->data[i].force[1];
        app_data->physics_cpu->force_z[i] = app_data->physics_gpu->data[i].force[2];
        
        app_data->physics_cpu->accel_x[i] = app_data->physics_gpu->data[i].accel[0];
        app_data->physics_cpu->accel_y[i] = app_data->physics_gpu->data[i].accel[1];
        app_data->physics_cpu->accel_z[i] = app_data->physics_gpu->data[i].accel[2];
    }
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    if(!default_render_render(ARRAY_LENGTH, app_data->camera.view_proj, app_data->camera.inv_view, app_data->transform_cpu->pos_x, app_data->transform_cpu->pos_y, app_data->transform_cpu->pos_z, &app_data->default_render_data, context)) return false;
    if(!debug_render_pass_render(&app_data->debug_render_data, app_data->camera.view_proj, context)) return false;
    
#if 1
    // imgui
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    if(nk_begin(ctx, "Timings", nk_rect(100, 100, 250, 250), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Num objects", ARRAY_LENGTH);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Octree depth", app_data->octree_depth);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Previous frame (ms)", context->delta * 1000.0f);
    }
    nk_end(ctx);
#endif
    
    return true;
}

void gravity_bh(transform_soa* transforms, physics_soa* physics, octree_node** octree)
{
    
}
