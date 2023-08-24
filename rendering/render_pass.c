#include "render_pass.h"
#include "debug_render_pass.h"
#include "imgui_render_pass.h"

#include "app/app.h"
#include "app/components.h"

typedef struct vertex_t
{
    float pos[N3];
    float tex_coords[N2];
} vertex;

typedef struct inst_vertex_t
{
    float model[M4];
    float color[N4];
} inst_vertex;

typedef struct uniform_t
{
    float view_proj[M4];
} uniform;

#define MAX_ENTITIES_PER_FRAME MAX_ENTITIES
typedef struct render_pass_data_t
{
    dm_render_handle vb, instb, ib, shader, pipe, uni;
    dm_render_handle tex;
    
    uint32_t         entity_count, instance_count;
    
    dm_entity        entities[MAX_ENTITIES_PER_FRAME];
    inst_vertex      insts[MAX_ENTITIES_PER_FRAME];
} render_pass_data;

bool render_pass_init(dm_context* context)
{
    application_data* app_data = context->app_data;
    app_data->render_pass_data = dm_alloc(sizeof(render_pass_data));
    render_pass_data* pass_data = app_data->render_pass_data;
    
    {
        // cube
        vertex vertices[] = {
            // front face
            { { -0.5f,-0.5f, 0.5f }, { 0,0 } },
            { {  0.5f,-0.5f, 0.5f }, { 1,0 } },
            { {  0.5f, 0.5f, 0.5f }, { 1,1 } },
            { { -0.5f, 0.5f, 0.5f }, { 0,1 } },
            
            // back face
            { {  0.5f,-0.5f,-0.5f }, { 0,0 } }, 
            { { -0.5f,-0.5f,-0.5f }, { 1,0 } },
            { { -0.5f, 0.5f,-0.5f }, { 1,1 } },
            { {  0.5f, 0.5f,-0.5f }, { 0,1 } },
            
            // right
            { {  0.5f,-0.5f, 0.5f }, { 0,0 } },
            { {  0.5f,-0.5f,-0.5f }, { 1,0 } },
            { {  0.5f, 0.5f,-0.5f }, { 1,1 } },
            { {  0.5f, 0.5f, 0.5f }, { 0,1 } },
            
            // left
            { { -0.5f, 0.5f, 0.5f }, { 0,0 } },
            { { -0.5f, 0.5f,-0.5f }, { 1,0 } },
            { { -0.5f,-0.5f,-0.5f }, { 1,1 } },
            { { -0.5f,-0.5f, 0.5f }, { 0,1 } },
            
            // bottom
            { { -0.5f,-0.5f,-0.5f }, { 0,0 } },
            { {  0.5f,-0.5f,-0.5f }, { 1,0 } },
            { {  0.5f,-0.5f, 0.5f }, { 1,1 } },
            { { -0.5f,-0.5f, 0.5f }, { 0,1 } },
            
            // top
            { { -0.5f, 0.5f, 0.5f }, { 0,0 } },
            { {  0.5f, 0.5f, 0.5f }, { 1,0 } },
            { {  0.5f, 0.5f,-0.5f }, { 1,1 } },
            { { -0.5f, 0.5f,-0.5f }, { 0,1 } },
        };
        
        uint32_t indices[] = {
            0,1,2,
            2,3,0,
            
            4,5,6,
            6,7,4,
            
            8,9,10,
            10,11,8,
            
            12,13,14,
            14,15,12,
            
            16,17,18,
            18,19,16,
            
            20,21,22,
            22,23,20
        };
        
        dm_vertex_attrib_desc attrib_descs[] = {
            { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, pos), .count=3, .index=0, .normalized=false },
            { .name="TEXCOORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, tex_coords), .count=2, .index=0, .normalized=false },
            { .name="MODEL", .data_t=DM_VERTEX_DATA_T_MATRIX_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(inst_vertex), .offset=offsetof(inst_vertex, model), .count=4, .index=0, .normalized=false},
            { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(inst_vertex), .offset=offsetof(inst_vertex, color), .count=4, .index=0, .normalized=false }
        };
        
        dm_pipeline_desc pipeline_desc = dm_renderer_default_pipeline();
        
        // resources
        if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(vertex), &pass_data->vb, context)) return false;
        if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(inst_vertex) * MAX_ENTITIES_PER_FRAME, sizeof(inst_vertex), &pass_data->instb, context)) return false;
        if(!dm_renderer_create_static_index_buffer(indices, sizeof(indices), &pass_data->ib, context)) return false;
        if(!dm_renderer_create_uniform(sizeof(uniform), DM_UNIFORM_STAGE_VERTEX, &pass_data->uni, context)) return false;
        
        dm_shader_desc shader_desc = { 0 };
#ifdef DM_VULKAN
        strcpy(shader_desc.vertex, "assets/shaders/test_vertex.spv");
        strcpy(shader_desc.pixel, "assets/shaders/test_pixel.spv");
#elif defined(DM_OPENGL)
        strcpy(shader_desc.vertex, "assets/shaders/test_vertex.glsl");
        strcpy(shader_desc.pixel, "assets/shaders/test_pixel.glsl");
        
        shader_desc.vb_count = 2;
        shader_desc.vb[0] = pass_data->vb;
        shader_desc.vb[1] = pass_data->instb;
#elif defined(DM_DIRECTX)
        strcpy(shader_desc.vertex, "assets/shaders/test_vertex.fxc");
        strcpy(shader_desc.pixel, "assets/shaders/test_pixel.fxc");
#else
        strcpy(shader_desc.vertex, "vertex_main");
        strcpy(shader_desc.pixel, "fragment_main");
        strcpy(shader_desc.master, "assets/shaders/test.metallib");
#endif
        
        if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipeline_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &pass_data->shader, &pass_data->pipe, context)) return false;
        
        // assets
        if(!dm_renderer_create_texture_from_file("assets/textures/default_texture.png", 4, false, "default", &pass_data->tex, context)) return false;
    }
    
    return true;
}

void render_pass_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    dm_free(app_data->render_pass_data);
}

void render_pass_submit_entity(dm_entity entity, dm_context* context)
{
    application_data* app_data = context->app_data;
    render_pass_data* pass_data = app_data->render_pass_data;
    
    if(pass_data->entity_count+1 > MAX_ENTITIES_PER_FRAME) { DM_LOG_ERROR("Trying to render too many entities"); return; }

    pass_data->entities[pass_data->entity_count++] = entity;
}

bool render_pass_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    render_pass_data* pass_data = app_data->render_pass_data;
    
    float obj_rm[M4];
    component_transform transform = { 0 };
    
    inst_vertex* inst = NULL;
    
    dm_timer t = { 0 };
    dm_timer_start(&t, context);
    for(uint32_t i=0; i<pass_data->entity_count; i++)
    {
        transform = entity_get_transform(pass_data->entities[i], app_data->components.transform, context);
        if(transform.rot[0]==0 && transform.rot[1]==0 && transform.rot[2]==0 && transform.rot[3]==0) return false;

        inst = &pass_data->insts[i];
        
        dm_mat4_rotate_from_quat(transform.rot, obj_rm);
        
        dm_mat_scale_make(transform.scale, inst->model);
        dm_mat4_mul_mat4(inst->model, obj_rm, inst->model);
        dm_mat_translate(inst->model, transform.pos, inst->model);
#ifdef DM_DIRECTX
        dm_mat4_transpose(inst->model, inst->model);
#endif
        
#if 0
        component_collision collision = entity_get_collision(pass_data->entities[i], app_data->components.collision, context);
        
        float dim[] = {
            collision.aabb_global_max[0] - collision.aabb_global_min[0],
            collision.aabb_global_max[1] - collision.aabb_global_min[1],
            collision.aabb_global_max[2] - collision.aabb_global_min[2],
        };
        
        float color[4] = { 0,0,0,1 };
        if(collision.flag==COLLISION_FLAG_YES)
        {
            color[0] = 1;
        }
        else if(collision.flag==COLLISION_FLAG_POSSIBLE)
        {
            color[0] = 1;
            color[1] = 1;
        }
        else
        {
            color[0] = 1;
            color[1] = 1;
            color[2] = 1;
        }
        
        //debug_render_aabb(transform.pos, dim, color, context);
        switch(collision.shape)
        {
            case DM_COLLISION_SHAPE_BOX:
            dim[0] = collision.internal[3] - collision.internal[0];
            dim[1] = collision.internal[4] - collision.internal[1];
            dim[2] = collision.internal[5] - collision.internal[2];
            break;
            
            case DM_COLLISION_SHAPE_SPHERE:
            dim[0] = dim[1] = dim[2] = collision.internal[0] * 2;
            break;
            
            default:
            break;
        }
        
        debug_render_cube(transform.pos, dim, transform.rot, color, context);
#endif
        
        inst->color[0] = 1;
        inst->color[1] = 1;
        inst->color[2] = 1;
        inst->color[3] = 1;
        
        pass_data->instance_count++;
    }
    
    // uniform
    uniform uni = { 0 };
    
    dm_memcpy(uni.view_proj, app_data->camera.view_proj, sizeof(uni.view_proj));
#ifdef DM_DIRECTX
    dm_mat4_transpose(uni.view_proj, uni.view_proj);
#endif
    
    // render
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0.1f,0.3f,0.5f,1,context);
    
    dm_render_command_bind_shader(pass_data->shader, context);
    dm_render_command_bind_pipeline(pass_data->pipe, context);
    dm_render_command_bind_texture(pass_data->tex, 0, context);
    //dm_render_command_toggle_wireframe(true, context);
    dm_render_command_bind_buffer(pass_data->vb, 0, context);
    dm_render_command_bind_buffer(pass_data->instb, 1, context);
    dm_render_command_update_buffer(pass_data->instb, pass_data->insts, sizeof(pass_data->insts), 0, context);
    dm_render_command_bind_uniform(pass_data->uni, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(pass_data->uni, &uni, sizeof(uni), context);
    dm_render_command_bind_buffer(pass_data->ib, 0, context);
    dm_render_command_draw_instanced(36,pass_data->instance_count,0,0,0, context);
    
    imgui_draw_text_fmt(20,140, 0,1,1,1, context, "Renderer took: %0.3lf ms", dm_timer_elapsed_ms(&t, context));
    
    // reset counts back to 0
    pass_data->entity_count = 0;
    pass_data->instance_count = 0;
    
    return true;
}