#include "dm.h"
#include "camera.h"

#include <assert.h>
#include <string.h>

#define ARRAY_LENGTH 15000
#define FIXED_DT     0.016666f

#ifdef DM_DIRECTX
#define NO_COMPUTE
#endif

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

typedef struct application_data_t
{
    dm_compute_handle gravity, physics;
    dm_compute_handle xb, yb, zb, mb;
    dm_compute_handle fx_b, fy_b, fz_b;
    dm_compute_handle vx_b, vy_b, vz_b;
    
    dm_render_handle vb, instb, ib, uniform;
    dm_render_handle shader, pipeline, star_texture;
    
    basic_camera camera;
    
    double gravity_timing, physics_timing;
    double accumulated_time;
    
    uint32_t physics_iters, mesh_index_count, mesh_vertex_count;
    
    DM_ALIGN(16) float* x_buffer;
    DM_ALIGN(16) float* y_buffer;
    DM_ALIGN(16) float* z_buffer;
    
    DM_ALIGN(16) float* m_buffer;
    
    DM_ALIGN(16) float* fx_buffer;
    DM_ALIGN(16) float* fy_buffer;
    DM_ALIGN(16) float* fz_buffer;
    
    DM_ALIGN(16) float* vx_buffer;
    DM_ALIGN(16) float* vy_buffer;
    DM_ALIGN(16) float* vz_buffer;
} application_data;

void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->vsync = true;
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
#ifndef NO_COMPUTE
    // compute data
    {
        dm_compute_shader_desc gravity_desc = { 0 };
#ifdef DM_METAL
        strcpy(gravity_desc.path, "assets/shaders/gravity_compute.metallib");
        strcpy(gravity_desc.function, "gravity_calc");
#elif defined(DM_DIRECTX)
        strcpy(gravity_desc.path, "assets/shaders/test_compute.fxc");
#else
        DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
        assert(false);
#endif
        
        dm_compute_shader_desc physics_desc = { 0 };
#ifdef DM_METAL
        strcpy(physics_desc.path, "assets/shaders/physics_compute.metallib");
        strcpy(physics_desc.function, "physics_update");
#elif defined(DM_DIRECTX)
        strcpy(physics_desc.path, "assets/shaders/test_compute.fxc");
#else
        DM_LOG_FATAL("Compute shader isn't supported for this backend yet");
        assert(false);
#endif
        
        if(!dm_compute_create_shader(gravity_desc, &app_data->gravity, context)) return false;
        if(!dm_compute_create_shader(physics_desc, &app_data->physics, context)) return false;
        
        static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
        
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->xb, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->yb, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->zb, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_INPUT, &app_data->mb, context))   return false;
        
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fx_b, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fy_b, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->fz_b, context))   return false;
        
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->vx_b, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->vy_b, context))   return false;
        if(!dm_compute_create_buffer(data_size, sizeof(float), DM_COMPUTE_BUFFER_TYPE_OUTPUT, &app_data->vz_b, context))   return false;
        
        app_data->x_buffer  = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->y_buffer  = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->z_buffer  = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->m_buffer  = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        
        app_data->fx_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->fy_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->fz_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        
        app_data->vx_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->vy_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        app_data->vz_buffer = dm_alloc(sizeof(float) * ARRAY_LENGTH);
        
        for(uint32_t i=0; i<ARRAY_LENGTH; i++)
        {
            app_data->x_buffer[i] = dm_random_float(context) * 100.f - 50.f;
            app_data->y_buffer[i] = dm_random_float(context) * 100.f - 50.f;
            app_data->z_buffer[i] = dm_random_float(context) * 100.f - 50.f;
            
            app_data->m_buffer[i] = dm_random_float(context) * 1e8f;
            
            app_data->vx_buffer[i] = 0.f;
            app_data->vy_buffer[i] = 0.f;
            app_data->vz_buffer[i] = 0.f;
        }
        
        if(!dm_compute_command_update_buffer(app_data->mb, app_data->m_buffer, data_size, 0, context)) return false;
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
    dm_vec3 cam_pos = { 0,0,5 };
    dm_vec3 cam_for = { 0,0,-1 };
    
    camera_init(cam_pos, cam_for, 0.001f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), 10.0f, 5.0f, &app_data->camera);
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
#ifndef NO_COMPUTE
    dm_free(app_data->x_buffer);
    dm_free(app_data->y_buffer);
    dm_free(app_data->z_buffer);
    dm_free(app_data->m_buffer);
    
    dm_free(app_data->fx_buffer);
    dm_free(app_data->fy_buffer);
    dm_free(app_data->fz_buffer);
    
    dm_free(app_data->vx_buffer);
    dm_free(app_data->vy_buffer);
    dm_free(app_data->vz_buffer);
#endif
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    app_data->accumulated_time += context->delta;
    
    static const size_t data_size = sizeof(float) * ARRAY_LENGTH;
    
#ifndef NO_COMPUTE
    // gravity force calculation
    {
        if(!dm_compute_command_bind_shader(app_data->gravity, context)) return false;
        
        if(!dm_compute_command_update_buffer(app_data->xb, app_data->x_buffer, data_size, 0, context)) return false;
        if(!dm_compute_command_update_buffer(app_data->yb, app_data->y_buffer, data_size, 0, context)) return false;
        if(!dm_compute_command_update_buffer(app_data->zb, app_data->z_buffer, data_size, 0, context)) return false;
        
        if(!dm_compute_command_bind_buffer(app_data->xb, 0, 0, context))   return false;
        if(!dm_compute_command_bind_buffer(app_data->yb, 0, 1, context))   return false;
        if(!dm_compute_command_bind_buffer(app_data->zb, 0, 2, context))   return false;
        if(!dm_compute_command_bind_buffer(app_data->mb, 0, 3, context))   return false;
        if(!dm_compute_command_bind_buffer(app_data->fx_b, 0, 4, context)) return false;
        if(!dm_compute_command_bind_buffer(app_data->fy_b, 0, 5, context)) return false;
        if(!dm_compute_command_bind_buffer(app_data->fz_b, 0, 6, context)) return false;
        
        dm_timer t = { 0 };
        dm_timer_start(&t, context);
#ifdef DM_METAL
        if(!dm_compute_command_dispatch(ARRAY_LENGTH,1,1, 1024,1,1, context)) return false;
#elif defined(DM_DIRECTX)
        if(!dm_compute_command_dispatch(65535,1,1, 1024,1,1, context)) return false;
#endif
        app_data->gravity_timing = dm_timer_elapsed_ms(&t, context);\
        
        if(!app_data->fx_buffer || !app_data->fy_buffer || !app_data->fz_buffer) return false;
    }
    
    // physics update
    app_data->physics_timing = 0;
    app_data->physics_iters = 0;
    while(app_data->accumulated_time > FIXED_DT)
    {
        app_data->physics_iters++;
        
        if(!dm_compute_command_update_buffer(app_data->vx_b, app_data->vx_buffer, data_size, 0, context)) return false;
        if(!dm_compute_command_update_buffer(app_data->vy_b, app_data->vy_buffer, data_size, 0, context)) return false;
        if(!dm_compute_command_update_buffer(app_data->vz_b, app_data->vz_buffer, data_size, 0, context)) return false;
        
        if(!dm_compute_command_bind_shader(app_data->physics, context)) return false;
        
        if(!dm_compute_command_bind_buffer(app_data->fx_b, 0, 0, context)) return false;
        if(!dm_compute_command_bind_buffer(app_data->fy_b, 0, 1, context)) return false;
        if(!dm_compute_command_bind_buffer(app_data->fz_b, 0, 2, context)) return false;
        if(!dm_compute_command_bind_buffer(app_data->mb,   0, 3, context)) return false;
        
        if(!dm_compute_command_bind_buffer(app_data->vx_b, 0, 4, context))  return false;
        if(!dm_compute_command_bind_buffer(app_data->vy_b, 0, 5, context))  return false;
        if(!dm_compute_command_bind_buffer(app_data->vz_b, 0, 6, context))  return false;
        
        if(!dm_compute_command_bind_buffer(app_data->xb, 0, 7, context))  return false;
        if(!dm_compute_command_bind_buffer(app_data->yb, 0, 8, context))  return false;
        if(!dm_compute_command_bind_buffer(app_data->zb, 0, 9, context))  return false;
        
        dm_timer t = { 0 };
        dm_timer_start(&t, context);
#ifdef DM_METAL
        if(!dm_compute_command_dispatch(ARRAY_LENGTH,1,1, 1024,1,1, context)) return false;
#elif defined(DM_DIRECTX)
        if(!dm_compute_command_dispatch(65535,1,1, 1024,1,1, context)) return false;
#endif
        app_data->physics_timing += dm_timer_elapsed_ms(&t, context);
        
        app_data->x_buffer = dm_compute_command_get_buffer_data(app_data->xb, context);
        app_data->y_buffer = dm_compute_command_get_buffer_data(app_data->yb, context);
        app_data->z_buffer = dm_compute_command_get_buffer_data(app_data->zb, context);
        
        app_data->vx_buffer = dm_compute_command_get_buffer_data(app_data->vx_b, context);
        app_data->vy_buffer = dm_compute_command_get_buffer_data(app_data->vy_b, context);
        app_data->vz_buffer = dm_compute_command_get_buffer_data(app_data->vz_b, context);
        
        if(!app_data->x_buffer || !app_data->y_buffer || !app_data->z_buffer)    return false;
        if(!app_data->vx_buffer || !app_data->vy_buffer || !app_data->vz_buffer) return false;
        
        app_data->accumulated_time -= FIXED_DT;
    }
    
    app_data->physics_timing /= (double)app_data->physics_iters;
#endif
    
    // camera
    camera_update(&app_data->camera, context);
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // render
    render_instance* instances = dm_alloc(sizeof(render_instance) * ARRAY_LENGTH);
    render_instance* inst = NULL;
    
    dm_vec4 pos = { 0,0,0};
    dm_vec3 scale = { 1,1,1 };
    for(uint32_t i=0; i<1; i++)
    {
        inst = &instances[i];
        
        //pos[0] = app_data->x_buffer[i];
        //pos[1] = app_data->y_buffer[i];
        //pos[2] = app_data->z_buffer[i];
        
        dm_mat_scale_make(scale, inst->model);
        //dm_mat_scale(app_data->camera.inv_view, scale, inst->model);
        dm_mat_scale(inst->model, scale, inst->model);
        dm_mat_translate(inst->model, pos, inst->model);
#ifdef DM_DIRECTX
        dm_mat4_transpose(inst->model, inst->model);
#endif
        
        inst->color[0] = 1; inst->color[1] = 1; inst->color[2] = 1; inst->color[3] = 1;
    }
    
    uniform_data uniform = { 0 };
    dm_memcpy(uniform.view_proj, app_data->camera.view_proj, sizeof(dm_mat4));
#ifdef DM_DIRECTX
    dm_mat4_transpose(uniform.view_proj, uniform.view_proj);
#endif
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    dm_render_command_bind_shader(app_data->shader, context);
    dm_render_command_bind_pipeline(app_data->pipeline, context);
    
    dm_render_command_bind_texture(app_data->star_texture, 0, context);
    
    dm_render_command_bind_buffer(app_data->ib, 0, context);
    dm_render_command_bind_buffer(app_data->vb, 0, context);
    dm_render_command_bind_buffer(app_data->instb, 1, context);
    dm_render_command_update_buffer(app_data->instb, instances, sizeof(render_instance) * 1, 0, context);
    dm_render_command_bind_uniform(app_data->uniform, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(app_data->uniform, &uniform, sizeof(uniform), context);
    
    dm_render_command_draw_instanced(6, 1, 0, 0, 0, context);
    
    dm_free(instances);
    
    // imgui
    dm_imgui_nuklear_context* imgui_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_ctx->ctx;
    
    if(nk_begin(ctx, "Timings", nk_rect(100, 100, 250, 200), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE |
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Gravity compute took", app_data->gravity_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Physics compute (average) took", app_data->physics_timing);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Physics iterations", app_data->physics_iters);
    }
    nk_end(ctx);
    
    return true;
}
