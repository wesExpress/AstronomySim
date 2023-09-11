#include "render_pass.h"
#include "debug_render_pass.h"
#include "imgui_render_pass.h"

#include "../app/app.h"
#include "../app/components.h"

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

#define MAX_ENTITIES_PER_FRAME DM_ECS_MAX_ENTITIES
#define MAX_MESH_COUNT 10
typedef struct render_pass_data_t
{
    dm_render_handle vb, instb[MAX_MESH_COUNT], ib, shader, pipe, uni;
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
        for(uint32_t i=0; i<MAX_MESH_COUNT; i++)
        {
            if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(inst_vertex) * MAX_ENTITIES_PER_FRAME, sizeof(inst_vertex), &pass_data->instb[i], context)) return false;
        }
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
        shader_desc.vb[1] = pass_data->instb[0];
#elif defined(DM_DIRECTX)
        strcpy(shader_desc.vertex, "assets/shaders/test_vertex.fxc");
        strcpy(shader_desc.pixel, "assets/shaders/test_pixel.fxc");
#else
        strcpy(shader_desc.vertex, "vertex_test_main");
        strcpy(shader_desc.pixel, "fragment_test_main");
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
    
    const dm_ecs_id t_id = app_data->components.transform;
    const dm_ecs_id c_id = app_data->components.collision;
    
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    component_collision* collision = dm_ecs_get_component_block(c_id, context);
    uint32_t t_index, c_index; 
    
    float obj_rm[M4];
    
    inst_vertex* inst = NULL;
    
    float pos[3], scale[3], rot[4];
    
    for(uint32_t i=0; i<pass_data->entity_count; i++)
    {
        dm_entity entity = pass_data->entities[i];
        t_index = dm_ecs_entity_get_component_index(entity, t_id, context);
        c_index = dm_ecs_entity_get_component_index(entity, c_id, context);
        
        pos[0] = transform->pos_x[t_index]; 
        pos[1] = transform->pos_y[t_index]; 
        pos[2] = transform->pos_z[t_index];
        
        scale[0] = transform->scale_x[t_index]; 
        scale[1] = transform->scale_y[t_index]; 
        scale[2] = transform->scale_z[t_index]; 
        
        rot[0] = transform->rot_i[t_index];
        rot[1] = transform->rot_j[t_index];
        rot[2] = transform->rot_k[t_index];
        rot[3] = transform->rot_r[t_index];
        
        inst = &pass_data->insts[i];
        
        dm_mat4_rotate_from_quat(rot, obj_rm);
        
        dm_mat_scale_make(scale, inst->model);
        dm_mat4_mul_mat4(inst->model, obj_rm, inst->model);
        dm_mat_translate(inst->model, pos, inst->model);
#ifdef DM_DIRECTX
        dm_mat4_transpose(inst->model, inst->model);
#endif
        
#if 0
        float c[4];
        if(collision->flag[c_index]==COLLISION_FLAG_POSSIBLE)
        {
            c[0] = 1;
            c[1] = 1;
            c[2] = 0;
            c[3] = 1;
        }
        else if(collision->flag[c_index]==COLLISION_FLAG_YES)
        {
            c[0] = 1;
            c[1] = 0;
            c[2] = 0;
            c[3] = 1;
        }
        else
        {
            c[0] = 1;
            c[1] = 1;
            c[2] = 1;
            c[3] = 1;
        }
        
        float dim[3];
        dim[0] = collision->aabb_global_max_x[c_index] - collision->aabb_global_min_x[c_index];
        dim[1] = collision->aabb_global_max_y[c_index] - collision->aabb_global_min_y[c_index];
        dim[2] = collision->aabb_global_max_z[c_index] - collision->aabb_global_min_z[c_index];
        
        debug_render_aabb(pos, dim, c, context);
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
    dm_render_command_bind_shader(pass_data->shader, context);
    dm_render_command_bind_pipeline(pass_data->pipe, context);
    dm_render_command_bind_texture(pass_data->tex, 0, context);
    //dm_render_command_toggle_wireframe(true, context);
    dm_render_command_bind_buffer(pass_data->vb, 0, context);
    dm_render_command_bind_buffer(pass_data->instb[0], 1, context);
    dm_render_command_update_buffer(pass_data->instb[0], pass_data->insts, sizeof(pass_data->insts), 0, context);
    dm_render_command_bind_uniform(pass_data->uni, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(pass_data->uni, &uni, sizeof(uni), context);
    dm_render_command_bind_buffer(pass_data->ib, 0, context);
    dm_render_command_draw_instanced(36,pass_data->instance_count,0,0,0, context);

    // reset counts back to 0
    pass_data->entity_count = 0;
    pass_data->instance_count = 0;
    
    return true;
}
