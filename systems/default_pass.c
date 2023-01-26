#define DM_ECS_DEFAULT_COMPONENTS
#include "../dm.h"
#include "default_pass.h"

#define DEFAULT_MAX_MESHES 100

typedef struct default_vertex_t
{
    dm_vec3 pos;
    dm_vec3 normal;
    dm_vec2 tex_coords;
} default_vertex;

typedef struct default_instance_t
{
    dm_mat4 model;
    dm_mat4 normal;
    dm_vec4 diffuse_color;
    dm_vec4 specular_color;
} default_instance;

typedef struct default_scene_uni_t
{
    dm_mat4 view_proj;
    dm_vec4 light_color;
    dm_vec4 ambient_color;
    dm_vec3 light_pos;
    float   fcoef_inv;
    dm_vec3 view_pos;
} default_scene_uni;

typedef struct default_handles_t
{
    dm_render_handle vb, instb, ib;
    dm_render_handle pipeline;
    dm_render_handle pass;
    
    dm_render_handle default_texture;
    
    dm_render_handle meshes[DEFAULT_MAX_MESHES];
    uint32_t num_meshes;
    
    view_camera* camera;
} default_handles;

static default_handles handles = { 0 };

static default_instance instances[DEFAULT_MAX_MESHES][DM_MAX_INSTS] = { 0 };
static uint32_t         num_insts[DEFAULT_MAX_MESHES] = { 0 };

// render system for default pass
bool default_render_pass(dm_entity* entities, uint32_t entity_count)
{
    dm_memzero(num_insts, sizeof(uint32_t) * DEFAULT_MAX_MESHES);
    
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* scale_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_X);
    float* scale_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Y);
    float* scale_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Z);
    float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
    float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
    float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
    float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
    dm_render_handle* mesh_handles = dm_ecs_get_component_member(DM_COMPONENT_MESH, DM_MESH_MEM_HANDLE);
    dm_vec4* diffuses = dm_ecs_get_component_member(DM_COMPONENT_MATERIAL, DM_MATERIAL_MEM_DIFFUSE);
    dm_vec4* speculars = dm_ecs_get_component_member(DM_COMPONENT_MATERIAL, DM_MATERIAL_MEM_SPECULAR);
    
    // update instance buffer
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        dm_vec3 pos   = dm_vec3_set(pos_x[entity], pos_y[entity], pos_z[entity]);
        dm_vec3 scale = dm_vec3_set(scale_x[entity], scale_y[entity], scale_z[entity]);
        dm_quat rot   = dm_quat_set(rot_i[entity], rot_j[entity], rot_k[entity], rot_r[entity]);
        dm_vec4 diffuse = diffuses[entity];
        dm_vec4 specular = speculars[entity];
        dm_render_handle mesh_handle = mesh_handles[entity];
        
        default_instance* inst = &instances[mesh_handle][num_insts[mesh_handle]++];
        
        inst->model = dm_mat_scale(dm_mat4_identity(), scale);
        inst->model = dm_mat4_mul_mat4(inst->model, dm_mat4_rotate_from_quat(rot));
        inst->model = dm_mat_translate(inst->model, pos);
        
        inst->normal = dm_mat4_inverse(inst->model);
#ifdef DM_DIRECTX
        inst->model = dm_mat4_transpose(inst->model);
#else
        inst->normal = dm_mat4_transpose(inst->normal);
#endif
        
        inst->diffuse_color = diffuse;
        inst->specular_color = specular;
    }
    
    // determine uniforms
    dm_vec3 a_c = dm_vec3_set(1,1,1);
    float   a_strength = 0.2f;
    a_c = dm_vec3_scale(a_c, a_strength);
    
    default_scene_uni uni = { 0 };
#ifdef DM_DIRECTX
    uni.view_proj = dm_mat4_transpose(handles.camera->view_proj);
#else
    uni.view_proj = handles.camera->view_proj;
#endif
    uni.light_color = dm_vec4_set(1,1,1,1);
    uni.ambient_color = dm_vec4_set(a_c.x,a_c.y,a_c.z, 1);
    uni.light_pos = dm_vec3_set(0,0,0);
    uni.fcoef_inv = 1.0f / dm_log2f(handles.camera->far_plane + 1);
    uni.view_pos = handles.camera->pos;
    
    // submit all render commands
    dm_render_command_clear(0,0,0,1);
    dm_render_command_set_default_viewport();
    
    dm_render_command_bind_pipeline(handles.pipeline);
    dm_render_command_begin_renderpass(handles.pass);
    dm_render_command_update_uniform(0, &uni, sizeof(uni), handles.pass);
    dm_render_command_bind_uniform(0, 0, handles.pass);
    dm_render_command_bind_texture(handles.default_texture, 0);
    
#ifdef DM_DEBUG
    static bool wireframe = false;
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE)) wireframe = !wireframe;
    dm_render_command_toggle_wireframe(wireframe);
#endif
    
    dm_render_command_bind_buffer(handles.ib, 0);
    dm_render_command_bind_buffer(handles.vb, 0);
    dm_render_command_bind_buffer(handles.instb, 1);
    
    for(uint32_t i=0; i<handles.num_meshes; i++)
    {
        dm_mesh mesh = dm_renderer_get_mesh(handles.meshes[i]);
        
        dm_render_command_update_buffer(handles.instb, &instances[i], sizeof(default_instance) * num_insts[i], 0);
        dm_render_command_draw_instanced(mesh.index_count, num_insts[i], mesh.index_offset, 0, 0);
    }
    
    dm_render_command_end_renderpass(handles.pass);
    
    return true;
}

bool default_pass_init(float* positions, float* normals, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes, view_camera* camera)
{
    // get vertices in workable format
    default_vertex* vertices = dm_alloc(sizeof(default_vertex) * num_vertices);
    for(uint32_t i=0; i<num_vertices; i++)
    {
        uint32_t offset_v3 = i * 3;
        uint32_t offset_v2 = i * 2;
        default_vertex vertex = {
            { positions[offset_v3], positions[offset_v3+1], positions[offset_v3+2] },
            { normals[offset_v3], normals[offset_v3+1], normals[offset_v3+2] },
            { tex_coords[offset_v2], tex_coords[offset_v2+1] },
        };
        vertices[i] = vertex;
    }
    
    // vertex attribs
    dm_vertex_attrib_desc attrib_descs[] = {
        DM_MAKE_VERTEX_ATTRIB("POSITION", default_vertex, pos, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
        DM_MAKE_VERTEX_ATTRIB("NORMAL", default_vertex, normal, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
        DM_MAKE_VERTEX_ATTRIB("TEXCOORD", default_vertex, tex_coords, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 2, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_MODEL", default_instance, model, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_MATRIX_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_NORM", default_instance, normal, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_MATRIX_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_DIFFUSE", default_instance, diffuse_color, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_SPECULAR", default_instance, diffuse_color, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_FLOAT, 4, 0, false),
    };
    uint32_t num_attribs = DM_ARRAY_LEN(attrib_descs);
    
    // uniforms
    dm_uniform unis[] = {
        { .data_size=sizeof(default_scene_uni), .name="scene_uni" }
    };
    uint32_t num_unis = DM_ARRAY_LEN(unis);
    
    // pipeline desc
    dm_pipeline_desc pipeline_desc = { 0 };
    pipeline_desc.cull_mode = DM_CULL_BACK;
    pipeline_desc.winding_order = DM_WINDING_COUNTER_CLOCK;
    pipeline_desc.primitive_topology = DM_TOPOLOGY_TRIANGLE_LIST;
    
    pipeline_desc.depth = true;
    pipeline_desc.depth_comp = DM_COMPARISON_LESS;
    
    pipeline_desc.blend = true;
    pipeline_desc.blend_eq = DM_BLEND_EQUATION_ADD;
    pipeline_desc.blend_src_f = DM_BLEND_FUNC_SRC_ALPHA;
    pipeline_desc.blend_dest_f = DM_BLEND_FUNC_ONE_MINUS_SRC_ALPHA;
    
    // resources
    if(!dm_renderer_create_static_index_buffer(indices, sizeof(uint32_t) * num_indices, &handles.ib)) return false;
    
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(default_vertex) * num_vertices, sizeof(default_vertex), &handles.vb)) return false;
    if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(default_instance) * DM_MAX_INSTS, sizeof(default_instance), &handles.instb)) return false;
    dm_render_handle vb_buffers[] = { handles.vb, handles.instb };
    
    dm_free(vertices);
    
    if(!dm_renderer_create_pipeline(pipeline_desc, &handles.pipeline)) return false;
#ifdef DM_OPENGL
    if(!dm_renderer_create_renderpass("assets/shaders/persp_vertex.glsl", "assets/shaders/persp_pixel.glsl", vb_buffers, 2, unis, num_unis, attrib_descs, num_attribs, &handles.pass)) return false;
#else
    if(!dm_renderer_create_renderpass("assets/shaders/persp_vertex.fxc", "assets/shaders/persp_pixel.fxc", unis, num_unis, attrib_descs, num_attribs, &handles.pass)) return false;
#endif
    
    if(!dm_renderer_create_texture_from_file("assets/textures/default_texture.png", 4, true, "default_texture", &handles.default_texture)) return false;
    
    // mesh handles
    dm_memcpy(handles.meshes, mesh_handles, sizeof(dm_render_handle) * num_meshes);
    handles.num_meshes = num_meshes;
    
    // camera handle
    handles.camera = camera;
    
    // register our render system
    dm_ecs_id render_system_component_ids[] = { DM_COMPONENT_TRANSFORM, DM_COMPONENT_MESH, DM_COMPONENT_MATERIAL };
    dm_ecs_id render_system;
    DM_ECS_REGISTER_SYSTEM(DM_ECS_SYSTEM_TIMING_RENDER, render_system_component_ids, default_render_pass, render_system);
    
    return true;
}