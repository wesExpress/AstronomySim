#ifndef __DEFAULT_PASS_H__
#define __DEFAULT_PASS_H__

#include "../app/camera.h"

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
    dm_vec3 view_pos;
    float   fcoef_inv;
} default_scene_uni;

typedef struct default_point_light_t
{
    dm_vec4 pos;
    dm_vec4 ambient, diffuse, specular;
    dm_vec4 params;
} default_point_light;

typedef struct default_blackbody_t
{
    dm_vec4 pos;
    dm_vec4 color;
    float   brightness;
    float   padding[3]; 
} default_blackbody;

#define MAX_LIGHTS 500
typedef struct default_lights_uni_t
{
    default_point_light point_lights[MAX_LIGHTS];
    default_blackbody   blackbodies[MAX_LIGHTS];
    
    uint32_t            num_point_lights;
    uint32_t            num_blacbodies;
} default_lights_uni;

typedef struct default_handles_t
{
    dm_render_handle vb, instb, ib;
    dm_render_handle pass;
    
    dm_render_handle default_texture;
    
    dm_render_handle meshes[DEFAULT_MAX_MESHES];
    uint32_t num_meshes;
    
    // lights
    dm_entity lights[LIGHT_TYPE_UNKNOWN][MAX_LIGHTS];
    uint32_t  num_lights[LIGHT_TYPE_UNKNOWN];
    
    dm_entity blackbodies[MAX_LIGHTS];
    uint32_t  num_blacbodies;
} default_handles;

static default_handles  d_handles = { 0 };
static default_instance default_instances[DEFAULT_MAX_MESHES][DM_MAX_INSTS] = { 0 };
static uint32_t         default_insts_count[DEFAULT_MAX_MESHES] = { 0 };

// render system for default pass
bool default_render_pass(dm_entity* entities, uint32_t entity_count)
{
    dm_memzero(default_insts_count, sizeof(uint32_t) * DEFAULT_MAX_MESHES);
    
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
        
        // early out if too far away to resolve
        // take max scale as dimension
        dm_vec3 pos = dm_vec3_set(pos_x[entity], pos_y[entity], pos_z[entity]);
        float r = DM_MAX(scale_x[entity], DM_MAX(scale_y[entity], scale_z[entity]));
        float theta = dm_atan(dm_vec3_len(pos), r);
        
        // angular resolution of 'eye'
        if(theta < 0.0003f) continue;
        
        dm_vec3 scale = dm_vec3_set(scale_x[entity], scale_y[entity], scale_z[entity]);
        dm_quat rot   = dm_quat_set(rot_i[entity], rot_j[entity], rot_k[entity], rot_r[entity]);
        dm_vec4 diffuse = diffuses[entity];
        dm_vec4 specular = speculars[entity];
        dm_render_handle mesh_handle = mesh_handles[entity];
        
        default_instance* inst = &default_instances[mesh_handle][default_insts_count[mesh_handle]++];
        
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
    
    // scene uni
    default_scene_uni scene_uni = { 0 };
#ifdef DM_DIRECTX
    scene_uni.view_proj = dm_mat4_transpose(app_data.camera.view_proj);
#else
    scene_uni.view_proj = app_data.camera.view_proj;
#endif
    scene_uni.fcoef_inv = 1.0f / dm_log2f(app_data.camera.far_plane + 1);
    scene_uni.view_pos = app_data.camera.pos;
    
    // light uni
    float* ambient_r  = get_light_member(LIGHT_MEM_AMBIENT_R);
    float* ambient_g  = get_light_member(LIGHT_MEM_AMBIENT_G);
    float* ambient_b  = get_light_member(LIGHT_MEM_AMBIENT_B);
    float* diffuse_r  = get_light_member(LIGHT_MEM_DIFFUSE_R);
    float* diffuse_g  = get_light_member(LIGHT_MEM_DIFFUSE_G);
    float* diffuse_b  = get_light_member(LIGHT_MEM_DIFFUSE_B);
    float* specular_r = get_light_member(LIGHT_MEM_SPECULAR_R);
    float* specular_g = get_light_member(LIGHT_MEM_SPECULAR_G);
    float* specular_b = get_light_member(LIGHT_MEM_SPECULAR_B);
    float* c          = get_light_member(LIGHT_MEM_UNION_0);
    float* l          = get_light_member(LIGHT_MEM_UNION_1);
    float* q          = get_light_member(LIGHT_MEM_UNION_2);
    
    default_lights_uni light_uni = { 0 };
    for(uint32_t i=0; i<d_handles.num_lights[LIGHT_TYPE_POINT]; i++)
    {
        dm_entity entity = d_handles.lights[LIGHT_TYPE_POINT][i];
        light_uni.point_lights[i] = (default_point_light){
            .pos=dm_vec4_set(pos_x[entity], pos_y[entity], pos_z[entity], 0),
            .ambient=dm_vec4_set(ambient_r[entity], ambient_g[entity], ambient_b[entity], 1),
            .diffuse=dm_vec4_set(diffuse_r[entity], diffuse_g[entity], diffuse_b[entity], 1),
            .specular=dm_vec4_set(specular_r[entity], specular_g[entity], specular_b[entity], 1),
            .params.x=c[entity], .params.y=l[entity], .params.z=q[entity]
        };
    }
    light_uni.num_point_lights = d_handles.num_lights[LIGHT_TYPE_POINT];
    
    // blackbodies
    float* brightness = get_blackbody_member(BLACKBODY_MEM_BRIGHTNESS);
    float* color_r    = get_blackbody_member(BLACKBODY_MEM_COLOR_R);
    float* color_g    = get_blackbody_member(BLACKBODY_MEM_COLOR_G);
    float* color_b    = get_blackbody_member(BLACKBODY_MEM_COLOR_B);
    float* color_a    = get_blackbody_member(BLACKBODY_MEM_COLOR_A);
    
    for(uint32_t i=0; i<d_handles.num_blacbodies; i++)
    {
        dm_entity entity = d_handles.blackbodies[i];
        light_uni.blackbodies[i] = (default_blackbody){
            .pos=dm_vec4_set(pos_x[entity], pos_y[entity], pos_z[entity], 1),
            .color=dm_vec4_set(color_r[entity], color_g[entity], color_b[entity], color_a[entity]),
            .brightness=brightness[entity]
        };
    }
    light_uni.num_blacbodies = d_handles.num_blacbodies;
    
    // submit all render commands
    dm_render_command_begin_renderpass(d_handles.pass);
    
    dm_render_command_update_uniform(0, &scene_uni, sizeof(scene_uni), d_handles.pass);
    dm_render_command_bind_uniform(0, 0, d_handles.pass);
    
    dm_render_command_update_uniform(1, &light_uni, sizeof(light_uni), d_handles.pass);
    dm_render_command_bind_uniform(1, 1, d_handles.pass);
    
    dm_render_command_bind_texture(d_handles.default_texture, 0);
    
    //#ifdef DM_DEBUG
#if 1
    static bool wireframe = false;
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE)) wireframe = !wireframe;
    dm_render_command_toggle_wireframe(wireframe);
#endif
    
    dm_render_command_bind_buffer(d_handles.ib, 0);
    dm_render_command_bind_buffer(d_handles.vb, 0);
    dm_render_command_bind_buffer(d_handles.instb, 1);
    
    for(uint32_t i=0; i<d_handles.num_meshes; i++)
    {
        dm_mesh mesh = dm_renderer_get_mesh(d_handles.meshes[i]);
        uint32_t num = default_insts_count[i];
        if(num==0) continue;
        
        dm_render_command_update_buffer(d_handles.instb, &default_instances[i], sizeof(default_instance) * num, 0);
        dm_render_command_draw_instanced(mesh.index_count, num, mesh.index_offset, 0, 0);
    }
    
    dm_render_command_end_renderpass(d_handles.pass);
    
    return true;
}

return_code __default_pass_init(float* positions, float* normals, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes)
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
    
    // uniforms
    dm_uniform unis[] = {
        { .data_size=sizeof(default_scene_uni), .name="scene_uni" },
        { .data_size=sizeof(default_lights_uni), .name="lights_uni" },
    };
    
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
    if(!DM_CREATE_STATIC_INDEX_BUFFER(indices, uint32_t, num_indices, d_handles.ib)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_STATIC_VERTEX_BUFFER(vertices, default_vertex, num_vertices, d_handles.vb)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_DYNAMIC_VERTEX_BUFFER(NULL, default_instance, DM_MAX_INSTS, d_handles.instb)) return RESOURCE_CREATION_FAIL;
    
    dm_free(vertices);
    
#ifdef DM_OPENGL
    dm_render_handle vb_buffers[] = { d_handles.vb, d_handles.instb };
    if(!DM_RENDERER_CREATE_RENDERPASS("assets/shaders/persp_vertex.glsl", "assets/shaders/persp_pixel.glsl", vb_buffers, unis, attrib_descs, pipeline_desc, d_handles.pass)) return RESOURCE_CREATION_FAIL;
#else
#ifdef DM_DIRECTX
    const char* vertex_src = "assets/shaders/persp_vertex.fxc";
    const char* pixel_src = "assets/shaders/persp_pixel.fxc";
#elif defined(DM_METAL)
    const char* vertex_src = "assets/shaders/persp.metallib";
    const char* pixel_src = "assets/shaders/persp.metallib";
#endif
    
    if(!DM_RENDERER_CREATE_RENDERPASS(vertex_src, pixel_src, unis, attrib_descs, pipeline_desc, d_handles.pass)) return RESOURCE_CREATION_FAIL;
#endif
    
    if(!dm_renderer_create_texture_from_file("assets/textures/default_texture.png", 4, true, "default_texture", &d_handles.default_texture)) return RESOURCE_CREATION_FAIL;
    
    // mesh handles
    dm_memcpy(d_handles.meshes, mesh_handles, sizeof(dm_render_handle) * num_meshes);
    d_handles.num_meshes = num_meshes;
    
    // register our render system
    dm_ecs_id render_system_component_ids[] = { DM_COMPONENT_TRANSFORM, DM_COMPONENT_MESH, DM_COMPONENT_MATERIAL };
    dm_ecs_id render_system_exclude_ids[] = { get_light_id(), get_blackbody_id() };
    dm_ecs_id render_system;
    DM_ECS_REGISTER_SYSTEM_EXCLUDES(DM_ECS_SYSTEM_TIMING_RENDER, render_system_component_ids, render_system_exclude_ids, default_render_pass, render_system);
    
    return SUCCESS;
}

#define DEFAULT_PASS_INIT(POSITIONS, NORMALS, TEX_COORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES) __default_pass_init(POSITIONS, NORMALS, TEX_COORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES, DM_ARRAY_LEN(MESHES))

void default_pass_add_point_light(dm_entity entity)
{
    d_handles.lights[LIGHT_TYPE_POINT][d_handles.num_lights[LIGHT_TYPE_POINT]++] = entity;
}

void default_pass_add_blackbody(dm_entity entity)
{
    d_handles.blackbodies[d_handles.num_blacbodies++] = entity;
}

#endif //DEFAULT_PASS_H
