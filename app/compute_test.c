#include "dm.h"
#include "camera.h"

#include <assert.h>
#include <string.h>

#define BIG_SIM

#ifdef BIG_SIM
#define ARRAY_LENGTH 16000
#define BLOCK_SIZE   512
#else
#define ARRAY_LENGTH 3

#define NO_COMPUTE
#endif

#define FIXED_DT 0.008333

//#define USE_ASTRO_UNITS

#ifndef USE_ASTRO_UNITS
#define G 6.67e-11f                        // N m^2 / kg^2

#define WORLD_CUBE_SIZE 50                 // m

#define MASS_SCALE      1e10f              // kg
#define SMBH_MASS       MASS_SCALE * 1.f  // kg
#else
#define G          4.3e-3f                 // pc Msun^-1 km^2 / s^2

#define WORLD_CUBE_SIZE 10                 // pc

#define MASS_SCALE 1.f                     // Msun
#define SMBH_MASS  MASS_SCALE * 1e3f       // MSUN
#endif

#define DRAW_BILBOARDS

typedef struct render_vertex_t
{
    dm_vec3 pos;
    dm_vec2 tex_coords;
} render_vertex;

typedef struct render_instance_t
{
    dm_mat4 model;
    dm_vec4 color;
} render_instance;

typedef struct uniform_data_t
{
    dm_mat4 view_proj;
} uniform_data;

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

typedef struct application_data_t
{
    dm_compute_handle gravity, physics;
    dm_compute_handle transform_b, physics_b;
    
    dm_render_handle vb, instb, ib, uniform;
    dm_render_handle shader, pipeline, star_texture;
    
    basic_camera camera;
    
    double gravity_timing, physics_timing, instance_creation_timing;
    double accumulated_time;
    
    uint32_t physics_iters, mesh_index_count, mesh_vertex_count;
    
    bool pause;
    
    star_data_soa* star_data;
    transform_gpu_element* transform_gpu_data;
    physics_gpu_element* physics_gpu_data;
} application_data;

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
    
#ifndef NO_COMPUTE
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
        
        for(uint32_t i=1; i<ARRAY_LENGTH; i++)
        {
            p[0] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[1] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            p[2] = dm_random_float_normal(0, .5f, context) * WORLD_CUBE_SIZE;
            
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
    }
#else
    {
#if 0
        r[0][0] = 0.9700436;
        r[0][1] = -0.24308753;
        r[0][2] = 0;
        v[0][0] = 0.466203685;
        v[0][1] = 0.43236573;
        v[0][2] = 0;
        
        r[1][0] = -r[0][0];
        r[1][1] = -r[0][1];
        r[1][2] = -r[0][2];
        v[1][0] = v[0][0];
        v[1][1] = v[0][1];
        v[1][2] = v[0][2];
        
        r[2][0] = 0;
        r[2][1] = 0;
        r[2][2] = 0;
        v[2][0] = -2*v[0][0];
        v[2][1] = -2*v[0][1];
        v[2][2] = -2*v[0][2];
#endif
        app_data->star_data->pos_x[0] =  0.9700436f;
        app_data->star_data->pos_y[0] =  0.0f;
        app_data->star_data->pos_z[0] =  -0.24308753f;
        app_data->star_data->mass[0]  =  MASS_SCALE;
        
        app_data->star_data->vel_x[0] = 0.466203685f;
        app_data->star_data->vel_z[0] = 0.43236573f;
        
        app_data->star_data->pos_x[1] =  -0.9700436f;
        app_data->star_data->pos_y[1] =  0.0f;
        app_data->star_data->pos_z[1] =  0.24308753f;
        app_data->star_data->mass[1]  =  MASS_SCALE;
        
        app_data->star_data->vel_x[1] = 0.466203685f;
        app_data->star_data->vel_z[1] = 0.43236573f;
        
        app_data->star_data->pos_x[2] =  0;
        app_data->star_data->pos_y[2] =  0;
        app_data->star_data->pos_z[2] =  0;
        app_data->star_data->mass[2]  =  MASS_SCALE;
        
        app_data->star_data->vel_x[2] = -2.f * 0.466203685f;
        app_data->star_data->vel_z[2] = -2.f * 0.43236573f;
    }
#endif
    
    // render data
    {
        render_vertex vertices[] = {
            { { -0.5f,-0.5f,0.0f }, {  0,0 } },
            { {  0.5f,-0.5f,0.0f }, {  1,0 } },
            { {  0.5f, 0.5f,0.0f }, {  1,1 } },
            { { -0.5f, 0.5f,0.0f }, {  0,1 } },
        };
        
        uint32_t indices[] = {
            0,1,2,
            2,3,0
        };
        
        //size_t data_size = sizeof(float) * 3 * app_data->mesh_vertex_count;
        if(!dm_renderer_create_static_index_buffer(indices, sizeof(uint32_t) * 6, sizeof(uint32_t), &app_data->ib, context)) return false;
        if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(render_vertex) * 4, sizeof(render_vertex), &app_data->vb, context)) return false;
        if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(render_instance) * ARRAY_LENGTH, sizeof(render_instance), &app_data->instb, context)) return false;
        if(!dm_renderer_create_uniform(sizeof(uniform_data), DM_UNIFORM_STAGE_VERTEX, &app_data->uniform, context)) return false;
        
        dm_shader_desc shader_desc = { 0 };
#ifdef DM_METAL
        strcpy(shader_desc.master, "assets/shaders/persp.metallib");
        strcpy(shader_desc.vertex, "vertex_main");
        strcpy(shader_desc.pixel, "fragment_main");
        shader_desc.vb[0] = app_data->vb;
        shader_desc.vb[1] = app_data->instb;
        shader_desc.vb_count = 2;
#elif defined(DM_DIRECTX)
        strcpy(shader_desc.vertex, "assets/shaders/persp_vertex.fxc");
        strcpy(shader_desc.pixel, "assets/shaders/persp_pixel.fxc");
#endif
        
        dm_pipeline_desc pipe_desc = dm_renderer_default_pipeline();
        
        dm_vertex_attrib_desc attrib_descs[] = {
            { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(render_vertex), .offset=offsetof(render_vertex, pos), .count=3, .index=0, .normalized=false },
            { .name="TEXCOORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(render_vertex), .offset=offsetof(render_vertex, tex_coords), .count=2, .index=0, .normalized=false },
            { .name="OBJ_MODEL", .data_t=DM_VERTEX_DATA_T_MATRIX_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(render_instance), .offset=offsetof(render_instance, model), .count=4, .index=0, .normalized=false },
            { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(render_instance), .offset=offsetof(render_instance, color), .count=4, .index=0, .normalized=false },
        };
        
        if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipe_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &app_data->shader, &app_data->pipeline, context)) return false;
        
        
        static const uint32_t tex_width  = 500;
        static const uint32_t tex_height = 500;
        const int half_w = tex_width / 2;
        const int half_h = tex_height / 2;
        const size_t size = tex_width * tex_height * sizeof(uint32_t);
        
        uint32_t* tex_data = dm_alloc(size);
        dm_memzero(tex_data, size);
        
        int yi, xi;
        for(uint32_t y=0; y<tex_height; y++)
        {
            yi = y - half_h;
            for(uint32_t x=0; x<tex_width; x++)
            {
                xi = x - half_w;
                if(yi * yi + xi * xi <= 250 * 250) tex_data[x + y * tex_width] = 0xffffffff;
            }
        }
        
        if(!dm_renderer_create_texture_from_data(tex_width, tex_height, 4, tex_data, "star_texture", &app_data->star_texture, context)) return false;
    }
    
    // camera
#ifdef NO_COMPUTE
    dm_vec3 cam_pos = { 0,0,2 };
    float move_sens = 5.f;
#else
    dm_vec3 cam_pos = { 0,0,WORLD_CUBE_SIZE };
    float move_sens = 10.f;
#endif
    dm_vec3 cam_for = { 0,0,-1 };
    
    camera_init(cam_pos, cam_for, 0.001f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), move_sens, 1.0f, &app_data->camera);
    
    app_data->pause = true;
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
#ifndef NO_COMPUTE
    dm_free(app_data->star_data);
    dm_free(app_data->transform_gpu_data);
    dm_free(app_data->physics_gpu_data);
#endif
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // camera
    camera_update(&app_data->camera, context);
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE, context)) app_data->pause = !app_data->pause;
    
    if(app_data->pause) return true;
    
    app_data->accumulated_time += context->delta;
    app_data->physics_timing = 0;
    app_data->physics_iters  = 0;
    
    const int group_count = (int)dm_ceil((float)ARRAY_LENGTH / (float)BLOCK_SIZE);
    
#ifndef NO_COMPUTE
    // update gpu buffers
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
    
#else
    float f_x, f_y, f_z;
    float r_x, r_y, r_z;
    float dis_2, dis_6, inv_dis;
    float grav;
    
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        for(uint32_t j=i+1; j<ARRAY_LENGTH; j++)
        {
            r_x = app_data->star_data->pos_x[j] - app_data->star_data->pos_x[i];
            r_y = app_data->star_data->pos_y[j] - app_data->star_data->pos_y[i];
            r_z = app_data->star_data->pos_z[j] - app_data->star_data->pos_z[i];
            
            dis_2  = r_x * r_x;
            dis_2 += r_y * r_y;
            dis_2 += r_z * r_z;
            
            dis_6   = dis_2 * dis_2 * dis_2 + 0.1f;
            dis_6   = dm_sqrtf(dis_6);
            inv_dis = 1.f / dis_6;
            
            grav  = G;
            grav *= app_data->star_data->mass[i] * app_data->star_data->mass[j];
            grav *= inv_dis;
            
            f_x = r_x * grav;
            f_y = r_y * grav;
            f_z = r_z * grav;
            
            app_data->star_data->force_x[i] += f_x;
            app_data->star_data->force_y[i] += f_y;
            app_data->star_data->force_z[i] += f_z;
            
            app_data->star_data->force_x[j] -= f_x;
            app_data->star_data->force_y[j] -= f_y;
            app_data->star_data->force_z[j] -= f_z;
        }
    }
    
    while(app_data->accumulated_time >= FIXED_DT)
    {
        float dt_mass;
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
            dt_mass = FIXED_DT / app_data->star_data->mass[i];
            
            app_data->star_data->pos_x[i] += app_data->star_data->vel_x[i] * FIXED_DT;
            app_data->star_data->pos_y[i] += app_data->star_data->vel_y[i] * FIXED_DT;
            app_data->star_data->pos_z[i] += app_data->star_data->vel_z[i] * FIXED_DT;
            
            app_data->star_data->vel_x[i] += app_data->star_data->force_x[i] * dt_mass;
            app_data->star_data->vel_y[i] += app_data->star_data->force_y[i] * dt_mass;
            app_data->star_data->vel_z[i] += app_data->star_data->force_z[i] * dt_mass;
        }
        
        app_data->accumulated_time -= FIXED_DT;
        app_data->physics_iters++;
        
        dm_memzero(app_data->star_data->force_x, data_size);
        dm_memzero(app_data->star_data->force_y, data_size);
        dm_memzero(app_data->star_data->force_z, data_size);
    }
    
    app_data->physics_timing /= (double)app_data->physics_iters;
#endif
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // render
    render_instance* instances = dm_alloc(sizeof(render_instance) * ARRAY_LENGTH);
    render_instance* inst = NULL;
    
    app_data->instance_creation_timing = 0;
    
    dm_vec3 pos;
    dm_vec3 scale;
    
    dm_timer t = { 0 };
    
    dm_timer_start(&t, context);
    for(uint32_t i=0; i<ARRAY_LENGTH; i++)
    {
        inst = &instances[i];
        
        pos[0] = app_data->star_data->pos_x[i];
        pos[1] = app_data->star_data->pos_y[i];
        pos[2] = app_data->star_data->pos_z[i];
        
#ifndef NO_COMPUTE
        switch(i)
        {
            case 0:
            scale[0] = 0.5f;
            scale[1] = 0.5f;
            scale[2] = 0.5f;
            
            inst->color[0] = 1;
            inst->color[1] = 1;
            inst->color[2] = 0;
            break;
            
            default:
            scale[0] = 0.15f;
            scale[1] = 0.15f;
            scale[2] = 0.15f;
            
            inst->color[0] = 1;
            inst->color[1] = 1;
            inst->color[2] = 1;
            break;
        }
        
#else
        switch(i)
        {
            case 0:
            inst->color[0] = 1;
            inst->color[1] = 0;
            inst->color[2] = 0;
            break;
            
            case 1:
            inst->color[1] = 1;
            inst->color[0] = 0;
            inst->color[2] = 0;
            break;
            
            case 2:
            inst->color[2] = 1;
            inst->color[0] = 0;
            inst->color[1] = 0;
            break;
        }
        
        scale[0] = 0.2f;
        scale[1] = 0.2f;
        scale[2] = 0.2f;
#endif
        inst->color[3] = 1;
        
#ifndef DRAW_BILBOARDS
        dm_mat_scale_make(scale, inst->model);
        dm_mat_translate(inst->model, pos, inst->model);
#else
        dm_mat_scale(app_data->camera.inv_view, scale, inst->model);
        
        // copy over position
        inst->model[3][0] = pos[0];
        inst->model[3][1] = pos[1];
        inst->model[3][2] = pos[2];
#endif
#ifdef DM_DIRECTX
        dm_mat4_transpose(inst->model, inst->model);
#endif
    }
    
    app_data->instance_creation_timing = dm_timer_elapsed_ms(&t, context);
    
    uniform_data uniform = { 0 };
#ifdef DM_DIRECTX
    dm_mat4_transpose(app_data->camera.view_proj, uniform.view_proj);
#else
    dm_memcpy(uniform.view_proj, app_data->camera.view_proj, sizeof(dm_mat4));
#endif
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    dm_render_command_bind_shader(app_data->shader, context);
    dm_render_command_bind_pipeline(app_data->pipeline, context);
    
    dm_render_command_bind_texture(app_data->star_texture, 0, context);
    
    dm_render_command_bind_buffer(app_data->ib, 0, context);
    dm_render_command_bind_buffer(app_data->vb, 0, context);
    dm_render_command_bind_buffer(app_data->instb, 1, context);
    dm_render_command_update_buffer(app_data->instb, instances, sizeof(render_instance) * ARRAY_LENGTH, 0, context);
    dm_render_command_bind_uniform(app_data->uniform, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(app_data->uniform, &uniform, sizeof(uniform), context);
    
    dm_render_command_draw_instanced(6, ARRAY_LENGTH, 0, 0, 0, context);
    
    dm_free(instances);
    
    // test grav calc
    uint32_t index = 1;
    
    // imgui
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    float test = app_data->star_data->force_x[index] / app_data->star_data->mass[index];
    
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
