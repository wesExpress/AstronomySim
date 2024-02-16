#include "dm.h"
#include "camera.h"

#include "gravity.h"
#include "physics.h"

#include "default_render.h"
#include "debug_render.h"

#include "octree.h"

#include "simulation_defines.h"

#include <assert.h>
#include <string.h>
#include <float.h>

#define OCTREE_FUDGE_FACTOR 0.001f;

#define WORLD_CUBE_SIZE 50                 // m

#define MASS_SCALE      1e9f               // kg
#define SMBH_MASS       MASS_SCALE * 1e4f  // kg

#define VEL_SCALE       3.5f               // m/s

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
typedef struct application_data_t
{
    dm_compute_handle transform_b, physics_b;
    
    basic_camera camera;
    
    bool pause;
    
    // octree manager
    uint32_t  max_octree_node_count;
    uint32_t  octree_node_count;
    dm_vec3   octree_center;
    float     octree_half_extents;
    
    bh_tree* tree;
    
    double tree_creation_timing, bh_timing;
    
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

void gravity_bh2(const float width, const transform_soa* transforms, physics_soa* physics, bh_tree* tree, void** render_data, dm_context* context);
/*
 DarkMatter interface
 */
void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->vsync = true;
    
    strcpy(init_packet->window_title, "Gravity Sim");
    
    init_packet->window_width  = 1920;
    init_packet->window_height = 1080;
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
        
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
#if 0
            p[0] = dm_random_float_normal(0, 1.5f, context) * WORLD_CUBE_SIZE;
            p[1] = dm_random_float_normal(0, 1.5f, context) * WORLD_CUBE_SIZE;
            p[2] = dm_random_float_normal(0, 1.5f, context) * WORLD_CUBE_SIZE;
#else
            p[0] = dm_random_float(context) - 0.5f ;
            p[1] = dm_random_float(context) - 0.5f;
            p[2] = dm_random_float(context) - 0.5f;
            dm_vec3_norm(p, p);
            dm_vec3_scale(p, WORLD_CUBE_SIZE, p);
#endif
            
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
            
            dm_vec3 dir;
            dm_vec3 up = { 0,1,0 };
            dm_vec3_cross(p, up, dir);
            dm_vec3_norm(dir, dir);
            
            const float v = dm_random_float(context) * VEL_SCALE;
            
#if 0
            app_data->physics_cpu->vel_x[i] = dm_random_float(context) * VEL_SCALE - 0.5f * VEL_SCALE;
            app_data->physics_cpu->vel_y[i] = dm_random_float(context) * VEL_SCALE - 0.5f * VEL_SCALE;
            app_data->physics_cpu->vel_z[i] = dm_random_float(context) * VEL_SCALE - 0.5f * VEL_SCALE;
#else
            app_data->physics_cpu->vel_x[i] = dir[0] * v;
            app_data->physics_cpu->vel_y[i] = dir[1] * v;
            app_data->physics_cpu->vel_z[i] = dir[2] * v;
#endif
            
            app_data->physics_cpu->accel_x[i] = 0;
            app_data->physics_cpu->accel_y[i] = 0;
            app_data->physics_cpu->accel_z[i] = 0;
        }
        
        dm_vec3 extents, center;
        dm_vec3_sub_vec3(max_p, min_p, extents);
        dm_vec3_fabs(extents, extents);
        
        float half_extents = extents[0];
        half_extents = DM_MAX(half_extents, extents[1]);
        half_extents = DM_MAX(half_extents, extents[2]);
        // fudge factor
        half_extents += OCTREE_FUDGE_FACTOR;
        half_extents *= 0.5f;
        
        dm_timer timer = { 0 };
        dm_timer_start(&timer, context);
        app_data->tree = dm_alloc(sizeof(bh_tree));
        bh_tree_init(app_data->tree);
        
        float cen_x = max_p[0] - half_extents;
        float cen_y = max_p[1] - half_extents;
        float cen_z = max_p[2] - half_extents;
        
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
            bh_tree_insert(i,0, cen_x,cen_y,cen_z,half_extents,0, app_data->tree, app_data->transform_cpu->pos_x, app_data->transform_cpu->pos_y, app_data->transform_cpu->pos_z, app_data->physics_cpu->mass);
        }
        app_data->tree_creation_timing = dm_timer_elapsed_ms(&timer, context);
    }
    
    // render passes
    {
        if(!default_render_init(ARRAY_LENGTH, &app_data->default_render_data, context)) return false;
        if(!debug_render_pass_init(&app_data->debug_render_data, context))              return false;
    }
    
    // camera
    dm_vec3 cam_pos = { 0,0,WORLD_CUBE_SIZE * 2.f };
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
    
    gravity_shutdown(&app_data->grav_data, context);
    physics_shutdown(&app_data->physics_data, context);
    
    dm_free(app_data->tree);
    
    default_render_shutdown(&app_data->default_render_data, context);
    debug_render_pass_shutdown(&app_data->debug_render_data, context);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
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
    dm_vec3 extents;
    dm_vec3_sub_vec3(max, min, extents);
    dm_vec3_fabs(extents, extents);
    half_extents = extents[0];
    half_extents = DM_MAX(half_extents, extents[1]);
    half_extents = DM_MAX(half_extents, extents[2]);
    // fudge factor
    half_extents += OCTREE_FUDGE_FACTOR;
    half_extents *= 0.5f;
    
    float cen_x = max[0] - half_extents;
    float cen_y = max[1] - half_extents;
    float cen_z = max[2] - half_extents;
    
    // camera
    camera_update(&app_data->camera, context);
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE, context)) app_data->pause = !app_data->pause;
    
    static bool draw_octree = false;
    if(dm_input_key_just_pressed(DM_KEY_1, context)) draw_octree = !draw_octree;
    if(draw_octree) bh_tree_render(cen_x,cen_y,cen_z,half_extents, app_data->tree, &app_data->debug_render_data, context);
    
    if(app_data->pause) return true;
    
    dm_timer timer = { 0 };
    
    const uint32_t group_count = (uint32_t)dm_ceil((float)ARRAY_LENGTH / (float)BLOCK_SIZE);
    
    // generate tree
    dm_timer_start(&timer, context);
    bh_tree_init(app_data->tree);
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        bh_tree_insert(i,0,cen_x,cen_y,cen_z,half_extents,0, app_data->tree, app_data->transform_cpu->pos_x, app_data->transform_cpu->pos_y, app_data->transform_cpu->pos_z, app_data->physics_cpu->mass);
    }
    app_data->tree_creation_timing = dm_timer_elapsed_ms(&timer, context);
    
    dm_timer_start(&timer, context);
    gravity_bh2(half_extents * 2.f, app_data->transform_cpu, app_data->physics_cpu, app_data->tree, &app_data->debug_render_data, context);
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        app_data->physics_gpu->data[i].force[0] = app_data->physics_cpu->force_x[i];
        app_data->physics_gpu->data[i].force[1] = app_data->physics_cpu->force_y[i];
        app_data->physics_gpu->data[i].force[2] = app_data->physics_cpu->force_z[i];
    }
    app_data->bh_timing = dm_timer_elapsed_ms(&timer, context);
    
    if (!dm_compute_command_update_buffer(app_data->transform_b, app_data->transform_gpu, sizeof(transform_aos), 0, context)) return false;
    if (!dm_compute_command_update_buffer(app_data->physics_b, app_data->physics_gpu, sizeof(physics_aos), 0, context)) return false;
    
    //if(!gravity_run(app_data->transform_b, app_data->physics_b, group_count, BLOCK_SIZE, app_data->grav_data, context)) return false;
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
    
    if(nk_begin(ctx, "Timings", nk_rect(100, 100, 250, 350), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Num objects", ARRAY_LENGTH);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Octree creation (ms)", app_data->tree_creation_timing);
        
        nk_value_float(ctx, "Barnes-Hut run (ms)", app_data->bh_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Octree max depth", app_data->tree->max_depth);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Octree node count (millions)", (float)app_data->tree->node_count / 1e6f);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Octree mem size (Mb)", (float)sizeof(bh_tree) / 1024 / 1024);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Previous frame (ms)", context->delta * 1000.0f);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "FPS", 1.f / context->delta);
    }
    nk_end(ctx);
#endif
    
    return true;
}

/**********
BARNES HUT
************/
#define BH_THETA 0.5f

// non-recursive, SIMD barnes-hut
void gravity_bh3(const float width, const transform_soa* transforms, physics_soa* physics, bh_tree* tree, void** render_data, dm_context* context)
{
    int* cur_child = dm_alloc(sizeof(int) * OCTREE_MAX_NODE_COUNT);
    uint32_t (*valid_children)[OCTREE_CHILDREN_COUNT] = dm_alloc(sizeof(uint32_t) * OCTREE_MAX_NODE_COUNT * OCTREE_CHILDREN_COUNT);
    
    dm_memset(cur_child, -1, sizeof(int) * OCTREE_MAX_NODE_COUNT);
    
    dm_simd_float pos_x_i, pos_y_i, pos_z_i;
    dm_simd_float com_x, com_y, com_z;
    dm_simd_float m_i, m_j, inv_m_j;
    
    dm_simd_float r_x, r_y, r_z;
    
    dm_simd_float dis, w, ratio;
    dm_simd_float dis2, dis6, inv_dis;
    dm_simd_float grav;
    
    dm_simd_float f_x, f_y, f_z;
    
    dm_simd_float ratio_mask;
    
    dm_simd_int object_indices_j, object_index_i, is_not_object_mask; 
    dm_simd_int first_child_j, has_children_mask;
    
    dm_simd_float valid_mask;
    
    const dm_simd_float grav_const = dm_simd_set1_float(G);
    const dm_simd_float theta      = dm_simd_set1_float(BH_THETA);
    const dm_simd_float soften_2   = dm_simd_set1_float(SOFTENING_2);
    const dm_simd_float ones       = dm_simd_set1_float(1.f);
    
    const dm_simd_int invalid_child = dm_simd_set1_i(OCTREE_INVALID_CHILD);
    
    w = dm_simd_set1_float(width);
    
    // for each object
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        pos_x_i = dm_simd_set1_float(transforms->pos_x[i]);
        pos_y_i = dm_simd_set1_float(transforms->pos_y[i]);
        pos_z_i = dm_simd_set1_float(transforms->pos_z[i]);
        m_i     = dm_simd_set1_float(physics->mass[i]);
        
        object_index_i = dm_simd_set1_i(i);
        
        uint32_t cur_index   = 0;
        
        // traverse octree
        while(true)
        {
            uint32_t child_index = tree->first_child[cur_index];
            
            // first time reaching this node
            // determine its valid children
            if(cur_child[cur_index]==-1 || child_index!=OCTREE_INVALID_CHILD)
            {
                // first four
                object_indices_j = dm_simd_load_i(tree->object_index + child_index);
                first_child_j    = dm_simd_load_i(tree->first_child  + child_index);
                
                com_x = dm_simd_load_float(tree->com_x + child_index);
                com_y = dm_simd_load_float(tree->com_y + child_index);
                com_z = dm_simd_load_float(tree->com_z + child_index);
                
                m_j     = dm_simd_load_float(tree->total_mass + child_index);
                inv_m_j = dm_simd_inv_float(m_j);
                
                com_x = dm_simd_mul_float(com_x, inv_m_j);
                com_y = dm_simd_mul_float(com_y, inv_m_j);
                com_z = dm_simd_mul_float(com_z, inv_m_j);
                
                r_x = dm_simd_sub_float(com_x, pos_x_i);
                r_y = dm_simd_sub_float(com_x, pos_x_i);
                r_z = dm_simd_sub_float(com_x, pos_x_i);
                
                dis2 = dm_simd_mul_float(r_x, r_x);
                dis2 = dm_simd_fmadd_float(r_y,r_y, dis2);
                dis2 = dm_simd_fmadd_float(r_z,r_z, dis2);
                dis  = dm_simd_sqrt_float(dis2);
                
                dis2 = dm_simd_add_float(dis2, soften_2);
                dis6 = dm_simd_mul_float(dis2, dis2);
                dis6 = dm_simd_mul_float(dis6, dis2);
                dis6 = dm_simd_sqrt_float(dis6);
                inv_dis = dm_simd_inv_float(dis6);
                
                grav = dm_simd_mul_float(m_i, m_j);
                grav = dm_simd_mul_float(grav, grav_const);
                grav = dm_simd_mul_float(grav, inv_dis);
                
                ratio = dm_simd_div_float(w, dis);
                
                ratio_mask         = dm_simd_lt_float(ratio, theta);
                is_not_object_mask = dm_simd_neq_i(object_index_i, object_indices_j);
                has_children_mask  = dm_simd_neq_i(first_child_j, invalid_child);
                
                valid_mask = dm_simd_and_float(ratio_mask, dm_simd_cast_int_to_float(is_not_object_mask));
                
                grav = dm_simd_mul_float(grav, valid_mask);
                
                f_x = dm_simd_mul_float(grav, r_x);
                f_y = dm_simd_mul_float(grav, r_y);
                f_z = dm_simd_mul_float(grav, r_z);
                
                // not an end point
                valid_mask = dm_simd_neq_float(ratio, ones);
                // has children
                valid_mask = dm_simd_and_float(valid_mask, dm_simd_cast_int_to_float(has_children_mask));
                valid_mask = dm_simd_and_float(valid_mask, ones);
                
                dm_simd_store_i(valid_children[cur_index], dm_simd_cast_float_to_int(valid_mask));
                
                // second four
                child_index += 4;
                object_indices_j = dm_simd_load_i(tree->object_index + child_index);
                first_child_j    = dm_simd_load_i(tree->first_child  + child_index);
                
                com_x = dm_simd_load_float(tree->com_x + child_index);
                com_y = dm_simd_load_float(tree->com_y + child_index);
                com_z = dm_simd_load_float(tree->com_z + child_index);
                
                m_j     = dm_simd_load_float(tree->total_mass + child_index);
                inv_m_j = dm_simd_div_float(ones, m_j);
                
                com_x = dm_simd_mul_float(com_x, inv_m_j);
                com_y = dm_simd_mul_float(com_y, inv_m_j);
                com_z = dm_simd_mul_float(com_z, inv_m_j);
                
                r_x = dm_simd_sub_float(com_x, pos_x_i);
                r_y = dm_simd_sub_float(com_x, pos_x_i);
                r_z = dm_simd_sub_float(com_x, pos_x_i);
                
                dis2 = dm_simd_mul_float(r_x, r_x);
                dis2 = dm_simd_fmadd_float(r_y,r_y, dis2);
                dis2 = dm_simd_fmadd_float(r_z,r_z, dis2);
                dis  = dm_simd_sqrt_float(dis2);
                
                dis2 = dm_simd_add_float(dis2, soften_2);
                dis6 = dm_simd_mul_float(dis2, dis2);
                dis6 = dm_simd_mul_float(dis6, dis2);
                dis6 = dm_simd_sqrt_float(dis6);
                inv_dis = dm_simd_inv_float(dis6);
                
                grav = dm_simd_mul_float(m_i, m_j);
                grav = dm_simd_mul_float(grav, grav_const);
                grav = dm_simd_mul_float(grav, inv_dis);
                
                ratio = dm_simd_div_float(w, dis);
                
                ratio_mask         = dm_simd_lt_float(ratio, theta);
                is_not_object_mask = dm_simd_neq_i(object_index_i, object_indices_j); 
                has_children_mask  = dm_simd_neq_i(first_child_j, invalid_child); 
                
                valid_mask = dm_simd_and_float(ratio_mask, dm_simd_cast_int_to_float(is_not_object_mask));
                
                grav = dm_simd_mul_float(grav, valid_mask);
                
                f_x = dm_simd_mul_float(grav, r_x);
                f_y = dm_simd_mul_float(grav, r_y);
                f_z = dm_simd_mul_float(grav, r_z);
                
                // not an end point
                valid_mask = dm_simd_neq_float(ratio, ones);
                // has children
                valid_mask = dm_simd_and_float(valid_mask, dm_simd_cast_int_to_float(has_children_mask));
                valid_mask = dm_simd_and_float(valid_mask, ones);
                
                dm_simd_store_i(valid_children[cur_index] + 4, dm_simd_cast_float_to_int(valid_mask));
                
                // determine valid child to start with
                for(uint32_t j=0; j<OCTREE_CHILDREN_COUNT; j++)
                {
                    if(valid_children[cur_index][j]==0) continue; 
                    cur_child[cur_index] = j;
                    break;
                }
                
                if(cur_child[cur_index]==-1) cur_index = tree->parent[cur_index];
                else                         cur_index = tree->first_child[cur_index] + cur_child[cur_index];
            }
            // we have returned to this parent node
            else
            {
                // no more children to visit, go to parent
                if(cur_child[cur_index]>=OCTREE_CHILDREN_COUNT)
                {
                    cur_index = tree->parent[cur_index];
                }
                else
                {
                    // determine next valid child
                    while(true)
                    {
                        cur_child[cur_index]++;
                        
                        if(valid_children[cur_index][cur_child[cur_index]]!=0 || cur_child[cur_index]>=OCTREE_CHILDREN_COUNT) break;
                        
                    }
                }
            }
        }
    }
    
    dm_free(cur_child);
    dm_free(valid_children);
}

// non-recursive octree traversal method
void gravity_bh2(const float width, const transform_soa* transforms, physics_soa* physics, bh_tree* tree, void** render_data, dm_context* context)
{
    //gravity_bh3(width, transforms, physics, tree, render_data, context);
    
    uint32_t* cur_child = dm_alloc(sizeof(uint32_t) * OCTREE_MAX_NODE_COUNT);
    uint32_t (*valid_children)[OCTREE_CHILDREN_COUNT] = dm_alloc(sizeof(uint32_t) * OCTREE_MAX_NODE_COUNT * OCTREE_CHILDREN_COUNT);
    
    float w = width;
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        uint32_t cur_index = 0;
        uint32_t nodes_hit = 0;
        
        float f_x,f_y,f_z;
        f_x = f_y = f_z = 0;
        
        const float p_x = transforms->pos_x[i];
        const float p_y = transforms->pos_y[i];
        const float p_z = transforms->pos_z[i];
        const float m_i = physics->mass[i];
        
        physics->force_x[i] = 0;
        physics->force_y[i] = 0;
        physics->force_z[i] = 0;
        
        while(true)
        {
            assert(nodes_hit<tree->node_count);
            // keep going through children
            if(tree->first_child[cur_index]!=OCTREE_INVALID_CHILD && cur_child[cur_index]<OCTREE_CHILDREN_COUNT)
            {
                cur_index = tree->first_child[cur_index] + cur_child[cur_index];
                bool valid_index = tree->object_index[cur_index]!=i;
                bool valid_object = tree->object_index[cur_index]!=OCTREE_NO_OBJECT;
                
                w *= 0.5f;
                nodes_hit++;
                
                // gravity calc
                float com_x = tree->com_x[cur_index];
                float com_y = tree->com_y[cur_index];
                float com_z = tree->com_z[cur_index];
                
                float m_j = tree->total_mass[cur_index]!=0 ? tree->total_mass[cur_index] : 1.f;
                float inv_m_j = 1.f / m_j;
                
                com_x *= inv_m_j;
                com_y *= inv_m_j;
                com_z *= inv_m_j;
                
                float r_x = com_x - p_x;
                float r_y = com_y - p_y;
                float r_z = com_z - p_z;
                
                float d, d2;
                d2  = r_x * r_x;
                d2 += r_y * r_y;
                d2 += r_z * r_z;
                d = dm_sqrtf(d2);
                d2 += SOFTENING_2;
                
                float ratio = w / d;
                
                float d6 = d2 * d2 * d2;
                d6 = dm_sqrtf(d6);
                float inv_d = 1.f / d6;
                float grav = G * m_i * m_j;
                grav *= inv_d;
                grav = valid_index ? grav : 0;
                
                bool small_ratio = ratio < BH_THETA;
                
                grav = (small_ratio || valid_object) ? grav : 0;
                
                f_x += r_x * grav;
                f_y += r_y * grav;
                f_z += r_z * grav;
                
                // barnes hut algorithm
                if(small_ratio || valid_object)
                {
                    // no longer progress down this node
                    cur_child[cur_index] = 0;
                    cur_index = tree->parent[cur_index];
                    cur_child[cur_index]++;
                    w *= 2.f;
                }
            }
            // we either have no children or cur_child[index] is 8
            else
            {
                if(tree->parent[cur_index]==OCTREE_INVALID_CHILD && cur_child[cur_index]==OCTREE_CHILDREN_COUNT) break; // end condition
                cur_child[cur_index] = 0;
                cur_index = tree->parent[cur_index];
                cur_child[cur_index]++;
                w *= 2.f;
            }
        }
        
        cur_child[0] = 0;
        
        physics->force_x[i] += f_x;
        physics->force_y[i] += f_y;
        physics->force_z[i] += f_z;
    }
    
    dm_free(valid_children);
    dm_free(cur_child);
}
