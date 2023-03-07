#ifndef __LIGHT_SRC_PASS_H__
#define __LIGHT_SRC_PASS_H__

#define LIGHT_SRC_MAX_MESHES 100

typedef struct light_src_vertex_t
{
    dm_vec3 pos;
    dm_vec2 tex_coords;
} light_src_vertex;

typedef struct light_src_instance_t
{
    dm_mat4 model;
    dm_vec4 diffuse_color;
} light_src_instance;

typedef struct light_src_scene_uni_t
{
    dm_mat4 view_proj;
    float   fcoef_inv;
} light_src_scene_uni;

typedef struct light_src_handles_t
{
    dm_render_handle vb, instb, ib;
    dm_render_handle pass;
    
    dm_render_handle meshes[LIGHT_SRC_MAX_MESHES];
    uint32_t num_meshes;
} light_src_handles;

static light_src_handles  light_handles = { 0 };
static light_src_instance light_instances[LIGHT_SRC_MAX_MESHES][DM_MAX_INSTS] = { 0 };
static uint32_t           light_insts_count[LIGHT_SRC_MAX_MESHES] = { 0 };

int light_src_sort(const void* entity_a, const void* entity_b)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    dm_entity a = *(dm_entity*)entity_a;
    dm_entity b = *(dm_entity*)entity_b;
    
    dm_vec3 p1 = dm_vec3_set(pos_x[a], pos_y[a], pos_z[a]);
    dm_vec3 p2 = dm_vec3_set(pos_x[b], pos_y[b], pos_z[b]);
    
    return (dm_vec3_len2(p1) > dm_vec3_len2(p2));
}

bool light_src_pass(dm_entity* entities, uint32_t entity_count)
{
    dm_memzero(light_insts_count, sizeof(uint32_t) * LIGHT_SRC_MAX_MESHES);
    
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
    
    // update instance buffer
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        dm_vec3 pos   = dm_vec3_set(pos_x[entity], pos_y[entity], pos_z[entity]);
        
        dm_vec3 scale = dm_vec3_set(scale_x[entity], scale_y[entity], scale_z[entity]);
        dm_quat rot   = dm_quat_set(rot_i[entity], rot_j[entity], rot_k[entity], rot_r[entity]);
        dm_vec4 diffuse = diffuses[entity];
        dm_render_handle mesh_handle = mesh_handles[entity];
        
        light_src_instance* inst = &light_instances[mesh_handle][light_insts_count[mesh_handle]++];
        
        inst->model = dm_mat_scale(dm_mat4_identity(), scale);
        inst->model = dm_mat4_mul_mat4(inst->model, dm_mat4_rotate_from_quat(rot));
        inst->model = dm_mat_translate(inst->model, pos);
        
#ifdef DM_DIRECTX
        inst->model = dm_mat4_transpose(inst->model);
#endif
        
        inst->diffuse_color = diffuse;
    }
    
    light_src_scene_uni uni = { 0 };
#ifdef DM_DIRECTX
    uni.view_proj = dm_mat4_transpose(app_data.camera.view_proj);
#else
    uni.view_proj = app_data.camera.view_proj;
#endif
    uni.fcoef_inv = 1.0f / dm_log2f(app_data.camera.far_plane + 1);
    
    // submit all render commands
    dm_render_command_begin_renderpass(light_handles.pass);
    dm_render_command_bind_uniform(0, 0, 2, light_handles.pass);
    dm_render_command_update_uniform(0, &uni, sizeof(uni), light_handles.pass);
    
#ifdef DM_DEBUG
    static bool wireframe = false;
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE)) wireframe = !wireframe;
    dm_render_command_toggle_wireframe(wireframe);
#endif
    
    dm_render_command_bind_buffer(light_handles.ib, 0);
    dm_render_command_bind_buffer(light_handles.vb, 0);
    dm_render_command_bind_buffer(light_handles.instb, 1);
    
    for(uint32_t i=0; i<light_handles.num_meshes; i++)
    {
        dm_mesh mesh = dm_renderer_get_mesh(light_handles.meshes[i]);
        uint32_t num = light_insts_count[i];
        if(num==0) continue;
        
        dm_render_command_update_buffer(light_handles.instb, &light_instances[i], sizeof(light_src_instance) * num, 0);
        dm_render_command_draw_instanced(mesh.index_count, num, mesh.index_offset, 0, 0);
    }
    
    dm_render_command_end_renderpass(light_handles.pass);
    
    return true;
}


return_code __light_src_pass_init(float* positions, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes)
{
    light_src_vertex* vertices = dm_alloc(sizeof(light_src_vertex) * num_vertices);
    for(uint32_t i=0; i<num_vertices; i++)
    {
        uint32_t offset_v3 = i * 3;
        uint32_t offset_v2 = i * 2;
        light_src_vertex vertex = {
            { positions[offset_v3], positions[offset_v3+1], positions[offset_v3+2] },
            { tex_coords[offset_v2], tex_coords[offset_v2+1] },
        };
        vertices[i] = vertex;
    }
    
    dm_vertex_attrib_desc attrib_descs[] = { 
        DM_MAKE_VERTEX_ATTRIB("POSITION", light_src_vertex, pos, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
        DM_MAKE_VERTEX_ATTRIB("TEXCOORD", light_src_vertex, tex_coords, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 2, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_MODEL", light_src_instance, model, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_MATRIX_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_DIFFUSE", light_src_instance, diffuse_color, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
    };
    
    dm_uniform unis[] = {
        { .data_size=sizeof(light_src_scene_uni), .stage=DM_UNIFORM_STAGE_BOTH, .name="scene_uni" }
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
    if(!DM_CREATE_STATIC_INDEX_BUFFER(indices, uint32_t, num_indices, light_handles.ib)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_STATIC_VERTEX_BUFFER(vertices, light_src_vertex, num_vertices, light_handles.vb)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_DYNAMIC_VERTEX_BUFFER(NULL, light_src_instance, DM_MAX_INSTS, light_handles.instb)) return RESOURCE_CREATION_FAIL;
    
    dm_free(vertices);
    
#ifdef DM_OPENGL
    dm_render_handle vb_buffers[] = { light_handles.vb, light_handles.instb };
    if(!DM_RENDERER_CREATE_RENDERPASS("assets/shaders/light_src_vertex.glsl", "assets/shaders/light_src_pixel.glsl", vb_buffers, unis, attrib_descs, pipeline_desc, light_handles.pass)) return RESOURCE_CREATION_FAIL;
#else
#ifdef DM_DIRECTX
    const char* vertex_src = "assets/shaders/light_src_vertex.fxc";
    const char* pixel_src = "assets/shaders/light_src_pixel.fxc";
#elif defined(DM_METAL)
    const char* vertex_src = "assets/shaders/light_src.metallib";
    const char* pixel_src = "assets/shaders/light_src.metallib";
#endif
    
    if(!DM_RENDERER_CREATE_RENDERPASS(vertex_src, pixel_src, unis, attrib_descs, pipeline_desc, light_handles.pass)) return RESOURCE_CREATION_FAIL;
#endif
    
    // mesh handles
    dm_memcpy(light_handles.meshes, mesh_handles, sizeof(dm_render_handle) * num_meshes);
    light_handles.num_meshes = num_meshes;
    
    // register render system
    dm_ecs_id light_src_system_component_ids[] = { DM_COMPONENT_TRANSFORM, DM_COMPONENT_MESH, DM_COMPONENT_MATERIAL, get_light_id() };
    dm_ecs_id light_src_system_exclude_ids[] = { get_blackbody_id() };
    dm_ecs_id render_system;
    DM_ECS_REGISTER_SYSTEM_EXCLUDES(DM_ECS_SYSTEM_TIMING_RENDER, light_src_system_component_ids, light_src_system_exclude_ids, light_src_pass, render_system);
    
    return SUCCESS;
}

#define LIGHT_SRC_PASS_INIT(POSITIONS, TEX_COORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES) __light_src_pass_init(POSITIONS, TEX_COORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES, DM_ARRAY_LEN(MESHES))

#endif //LIGHT_SRC_PASS_H
