#include "render_pass.h"
#include "debug_render_pass.h"
#include "imgui_render_pass.h"

#include "../app/app.h"
#include "../app/components.h"

typedef enum render_pass_flags_t
{
    RENDER_PASS_FLAG_WIREFRAME = 1 << 0,
} render_pass_flags;

typedef struct vertex_t
{
    float pos[N3];
    float tex_coords[N2];
    float normal[N3];
} vertex;

typedef struct inst_vertex_t
{
    float obj_model[M4];
    float obj_normal[M4];
    float color[N4];
} inst_vertex;

typedef struct uniform_t
{
    float view_proj[M4];
    float view_pos[N3];
} uniform;

typedef struct mesh_t
{
    uint32_t vertex_offset, index_offset;
    uint32_t vertex_count, index_count;
    
    dm_render_handle vb, ib, instb;
    dm_render_handle texture;
} mesh;

#define MAX_ENTITIES_PER_FRAME DM_ECS_MAX_ENTITIES
#define MAX_MESH_COUNT 100
typedef struct render_pass_data_t
{
    dm_render_handle shader, pipe, uni;
    dm_render_handle default_tex;
    
    uint32_t entity_count[MAX_MESH_COUNT], mesh_count;
    
    render_pass_flags flags;
    
    mesh meshes[MAX_MESH_COUNT];
    
    dm_entity        entities[MAX_MESH_COUNT][MAX_ENTITIES_PER_FRAME];
    inst_vertex      insts[MAX_MESH_COUNT][MAX_ENTITIES_PER_FRAME];
} render_pass_data;

void render_pass_draw_transform(float pos[3], float rot[4], dm_context* context)
{
    static const float x_axis[3] = { 1,0,0 };
    static const float y_axis[3] = { 0,1,0 };
    static const float z_axis[3] = { 0,0,1 };
    
    float x_rot[3], y_rot[3], z_rot[3];
    
    dm_vec3_rotate(x_axis, rot, x_rot);
    dm_vec3_rotate(y_axis, rot, y_rot);
    dm_vec3_rotate(z_axis, rot, z_rot);
    
    dm_vec3_add_vec3(pos, x_rot, x_rot);
    dm_vec3_add_vec3(pos, y_rot, y_rot);
    dm_vec3_add_vec3(pos, z_rot, z_rot);
    
    debug_render_arrow(pos, x_rot, COLOR_RED,   context);
    debug_render_arrow(pos, y_rot, COLOR_GREEN, context);
    debug_render_arrow(pos, z_rot, COLOR_BLUE,  context);
}

bool render_pass_init(dm_context* context)
{
    application_data* app_data = context->app_data;
    app_data->render_pass_data = dm_alloc(sizeof(render_pass_data));
    render_pass_data* pass_data = app_data->render_pass_data;
    
    {
        // cube
        vertex cube_vertices[] = {
            // front face
            { { -0.5f,-0.5f, 0.5f }, { 0,0 }, { 0,0,1 } },
            { {  0.5f,-0.5f, 0.5f }, { 1,0 }, { 0,0,1 } },
            { {  0.5f, 0.5f, 0.5f }, { 1,1 }, { 0,0,1 } },
            { { -0.5f, 0.5f, 0.5f }, { 0,1 }, { 0,0,1 } },
            
            // back face
            { {  0.5f,-0.5f,-0.5f }, { 0,0 }, { 0,0,-1 } }, 
            { { -0.5f,-0.5f,-0.5f }, { 1,0 }, { 0,0,-1 } },
            { { -0.5f, 0.5f,-0.5f }, { 1,1 }, { 0,0,-1 } },
            { {  0.5f, 0.5f,-0.5f }, { 0,1 }, { 0,0,-1 } },
            
            // right
            { {  0.5f,-0.5f, 0.5f }, { 0,0 }, { 1,0,0 } },
            { {  0.5f,-0.5f,-0.5f }, { 1,0 }, { 1,0,0 } },
            { {  0.5f, 0.5f,-0.5f }, { 1,1 }, { 1,0,0 } },
            { {  0.5f, 0.5f, 0.5f }, { 0,1 }, { 1,0,0 } },
            
            // left
            { { -0.5f, 0.5f, 0.5f }, { 0,0 }, { -1,0,0 } },
            { { -0.5f, 0.5f,-0.5f }, { 1,0 }, { -1,0,0 } },
            { { -0.5f,-0.5f,-0.5f }, { 1,1 }, { -1,0,0 } },
            { { -0.5f,-0.5f, 0.5f }, { 0,1 }, { -1,0,0 } },
            
            // bottom
            { { -0.5f,-0.5f,-0.5f }, { 0,0 }, { 0,-1,0 } },
            { {  0.5f,-0.5f,-0.5f }, { 1,0 }, { 0,-1,0 } },
            { {  0.5f,-0.5f, 0.5f }, { 1,1 }, { 0,-1,0 } },
            { { -0.5f,-0.5f, 0.5f }, { 0,1 }, { 0,-1,0 } },
            
            // top
            { { -0.5f, 0.5f, 0.5f }, { 0,0 }, { 0,1,0 } },
            { {  0.5f, 0.5f, 0.5f }, { 1,0 }, { 0,1,0 } },
            { {  0.5f, 0.5f,-0.5f }, { 1,1 }, { 0,1,0 } },
            { { -0.5f, 0.5f,-0.5f }, { 0,1 }, { 0,1,0 } },
        };
        
        uint32_t cube_indices[] = {
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
        
        mesh cube = {
            .vertex_count=DM_ARRAY_LEN(cube_vertices),
            .index_count=DM_ARRAY_LEN(cube_indices),
            .texture=DM_TEXTURE_INVALID
        };
        
        if(!dm_renderer_create_static_vertex_buffer(cube_vertices, sizeof(cube_vertices), sizeof(vertex), &cube.vb, context)) return false;
        if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(inst_vertex) * MAX_ENTITIES_PER_FRAME, sizeof(inst_vertex), &cube.instb, context)) return false;
        if(!dm_renderer_create_static_index_buffer(cube_indices, sizeof(cube_indices), sizeof(uint32_t), &cube.ib, context)) return false;
        
        dm_memcpy(pass_data->meshes + pass_data->mesh_count++, &cube, sizeof(cube));
    }
    
    // planet meshes
    {
        const dm_mesh_vertex_attrib mesh_attribs[] = {
            DM_MESH_VERTEX_ATTRIB_POSITION,
            DM_MESH_VERTEX_ATTRIB_TEXCOORD,
            DM_MESH_VERTEX_ATTRIB_NORMAL,
        };
        
        dm_render_handle planet_texture;
        if(!dm_renderer_create_texture_from_file("assets/textures/planets_texture.png", 4, false, "default", &planet_texture, context)) return false;
        
        for(uint32_t i=21; i<NUM_PLANETS; i++)
        {
            float*    mesh_vertices;
            uint32_t* mesh_indices;
            
            uint32_t mesh_vertex_count;
            uint32_t mesh_index_count;
            
            char buffer[512];
#if 0
            snprintf(buffer, sizeof(buffer), "assets/models/Planets_%u.obj", i+1);
#else
            snprintf(buffer, sizeof(buffer), "assets/models/Planet_%u.glb", i+1);
#endif
            if(!dm_renderer_load_model(buffer, mesh_attribs, DM_ARRAY_LEN(mesh_attribs), DM_MESH_INDEX_TYPE_UINT16, &mesh_vertices, (void**)&mesh_indices, &mesh_vertex_count, &mesh_index_count, 0, context)) return false;
            
            mesh planet = {
                .vertex_count=mesh_vertex_count, 
                .index_count=mesh_index_count
            };
            
            if(!dm_renderer_create_static_vertex_buffer(mesh_vertices, sizeof(float) * mesh_vertex_count * 8, sizeof(vertex), &planet.vb, context)) return false;
            if(!dm_renderer_create_static_index_buffer(mesh_indices, sizeof(uint16_t) * mesh_index_count, sizeof(uint16_t), &planet.ib, context)) return false;
            if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(inst_vertex) * MAX_ENTITIES_PER_FRAME, sizeof(inst_vertex), &planet.instb, context)) return false;
            
            planet.texture = planet_texture;
            
            dm_memcpy(pass_data->meshes + pass_data->mesh_count++, &planet, sizeof(planet));
            
            dm_free(mesh_vertices);
            dm_free(mesh_indices);
        }
    }
    
    // attribs
    dm_vertex_attrib_desc attrib_descs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, pos), .count=3, .index=0, .normalized=false },
        { .name="TEXCOORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, tex_coords), .count=2, .index=0, .normalized=false },
        { .name="NORMAL", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, normal), .count=3, .index=0, .normalized=false },
        { .name="OBJ_MODEL", .data_t=DM_VERTEX_DATA_T_MATRIX_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(inst_vertex), .offset=offsetof(inst_vertex, obj_model), .count=4, .index=0, .normalized=false},
        { .name="OBJ_NORM", .data_t=DM_VERTEX_DATA_T_MATRIX_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(inst_vertex), .offset=offsetof(inst_vertex, obj_normal), .count=4, .index=0, .normalized=false},
        { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(inst_vertex), .offset=offsetof(inst_vertex, color), .count=4, .index=0, .normalized=false }
    };
    
    dm_pipeline_desc pipeline_desc = dm_renderer_default_pipeline();
    
    // resources
    if(!dm_renderer_create_uniform(sizeof(uniform), DM_UNIFORM_STAGE_BOTH, &pass_data->uni, context)) return false;
    
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
    
    // textures
    if(!dm_renderer_create_texture_from_file("assets/textures/default_texture.png", 4, false, "default", &pass_data->default_tex, context)) return false;
    
    return true;
}

void render_pass_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    dm_free(app_data->render_pass_data);
}

void render_pass_submit_entity(dm_entity entity, uint32_t mesh_id, dm_context* context)
{
    application_data* app_data = context->app_data;
    render_pass_data* pass_data = app_data->render_pass_data;
    
    if(pass_data->entity_count[mesh_id]+1 > MAX_ENTITIES_PER_FRAME) { DM_LOG_ERROR("Trying to render too many entities"); return; }
    
    pass_data->entities[mesh_id][pass_data->entity_count[mesh_id]++] = entity;
}

bool render_pass_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    render_pass_data* pass_data = app_data->render_pass_data;
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE, context)) pass_data->flags ^= RENDER_PASS_FLAG_WIREFRAME;
    
    // shader and pipeline
    dm_render_command_bind_shader(pass_data->shader, context);
    dm_render_command_bind_pipeline(pass_data->pipe, context);
    if(pass_data->flags & RENDER_PASS_FLAG_WIREFRAME) dm_render_command_toggle_wireframe(true, context);
    
    // uniform
    uniform uni = { 0 };
    
    dm_memcpy(uni.view_proj, app_data->camera.view_proj, sizeof(uni.view_proj));
#ifdef DM_DIRECTX
    dm_mat4_transpose(uni.view_proj, uni.view_proj);
#endif
    
    DM_VEC3_COPY(uni.view_pos, app_data->camera.pos);
    dm_render_command_bind_uniform(pass_data->uni, 0, DM_UNIFORM_STAGE_BOTH, 0, context);
    dm_render_command_update_uniform(pass_data->uni, &uni, sizeof(uni), context);
    
    // update instance buffers
    const dm_ecs_id t_id = app_data->components.transform;
    
    component_transform* transform = dm_ecs_get_component_block(t_id, context);
    uint32_t t_index;
    
    float obj_rm[M4];
    
    inst_vertex* inst = NULL;
    
    float pos[3], scale[3], rot[4];
    
    mesh mesh;
    
    for(uint32_t m=0; m<pass_data->mesh_count; m++)
    {
        if(pass_data->entity_count[m]==0) continue;
        
        mesh = pass_data->meshes[m];
        
        if(mesh.texture==DM_TEXTURE_INVALID) dm_render_command_bind_texture(pass_data->default_tex, 0, context);
        else                                 dm_render_command_bind_texture(mesh.texture, 0, context);
        
        dm_render_command_bind_buffer(mesh.vb, 0, context);
        dm_render_command_bind_buffer(mesh.ib, 0, context);
        
        for(uint32_t i=0; i<pass_data->entity_count[m]; i++)
        {
            dm_entity entity = pass_data->entities[m][i];
            t_index = dm_ecs_entity_get_component_index(entity, t_id, context);
            
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
            
            inst = &pass_data->insts[m][i];
            
            dm_mat4_rotate_from_quat(rot, obj_rm);
            
            dm_mat_scale_make(scale, inst->obj_model);
            dm_mat4_mul_mat4(inst->obj_model, obj_rm, inst->obj_model);
            dm_mat_translate(inst->obj_model, pos, inst->obj_model);
#ifdef DM_DIRECTX
            dm_mat4_transpose(inst->obj_model, inst->obj_model);
#endif
            
            dm_mat4_inverse(inst->obj_model, inst->obj_normal);
            dm_mat4_transpose(inst->obj_normal, inst->obj_normal);
            
            inst->color[0] = 1;
            inst->color[1] = 1;
            inst->color[2] = 1;
            inst->color[3] = 1;
            
#if 0
            render_pass_draw_transform(pos, rot, context);
#endif
        }
        
        dm_render_command_bind_buffer(mesh.instb, 1, context);
        dm_render_command_update_buffer(mesh.instb, pass_data->insts[m], sizeof(pass_data->insts[m]), 0, context);
        dm_render_command_draw_instanced(mesh.index_count, pass_data->entity_count[m], mesh.index_offset,0,0, context);
    }
    
    // reset counts back to 0
    dm_memzero(pass_data->entity_count, sizeof(pass_data->entity_count));
    dm_memzero(pass_data->entities, sizeof(pass_data->entities));
    
    return true;
}
