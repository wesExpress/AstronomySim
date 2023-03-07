#ifndef __BLACKBODY_PASS_H__
#define __BLACKBODY_PASS_H__

#define BLACKBODY_MAX_MESHES 100

typedef struct blackbody_vertex_t
{
    dm_vec3 pos;
    dm_vec2 tex_coords;
} blackbody_vertex;

typedef struct blackbody_instance_t
{
    dm_mat4 model;
    float   color[4];
    float   brightness;
} blackbody_instance;

typedef struct airy_disc_vertex_t
{
    float pos[3];
    float color[4];
    float brightness;
} airy_disc_vertex;

typedef struct blackbody_scene_uni_t
{
    dm_mat4 view_proj;
    float   fcoef_inv;
} blackbody_scene_uni;

typedef struct airy_blur_uni_t
{
    float offset;
} airy_blur_uni;

typedef struct blackbody_handles_t
{
    dm_render_handle vb, instb, ib;
    
    // normal pass
    dm_render_handle pass;
    
    // airy disc pass
    dm_render_handle point_vb;
    dm_render_handle point_fb;
    dm_render_handle point_pass;
    
    dm_render_handle blur_vb;
    dm_render_handle blur_pass;
    
    dm_render_handle meshes[BLACKBODY_MAX_MESHES];
    uint32_t         num_meshes;
} blackbody_handles;

typedef struct blackbody_instances_t
{
    blackbody_instance resolved_instances[BLACKBODY_MAX_MESHES][DM_MAX_INSTS];
    uint32_t           resolved_count[BLACKBODY_MAX_MESHES];
    
    airy_disc_vertex   airy_disc_instances[DM_MAX_INSTS];
    uint32_t           airy_disc_count;
} blackbody_instances;

static blackbody_handles   bb_handles = { 0 };
static blackbody_instances bb_instances = { 0 };

bool resolved_pass(dm_entity* entities, uint32_t entity_count)
{
    dm_memzero(bb_instances.resolved_count, sizeof(uint32_t) * BLACKBODY_MAX_MESHES);
    
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
    
    float* brightness = get_blackbody_member(BLACKBODY_MEM_BRIGHTNESS);
    float* color_r = get_blackbody_member(BLACKBODY_MEM_COLOR_R);
    float* color_g = get_blackbody_member(BLACKBODY_MEM_COLOR_G);
    float* color_b = get_blackbody_member(BLACKBODY_MEM_COLOR_B);
    float* color_a = get_blackbody_member(BLACKBODY_MEM_COLOR_A);
    
    dm_render_handle* mesh_handles = dm_ecs_get_component_member(DM_COMPONENT_MESH, DM_MESH_MEM_HANDLE);
    
    // update instance buffer
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        // early out if we are way too far
        dm_vec3 pos = dm_vec3_set(pos_x[entity], pos_y[entity], pos_z[entity]);
        if(dm_vec3_len2(pos) > RESOLUTION_DISTANCE) continue;
        
        dm_vec3 scale = dm_vec3_set(scale_x[entity], scale_y[entity], scale_z[entity]);
        dm_quat rot   = dm_quat_set(rot_i[entity], rot_j[entity], rot_k[entity], rot_r[entity]);
        dm_render_handle mesh_handle = mesh_handles[entity];
        
        blackbody_instance* inst = &bb_instances.resolved_instances[mesh_handle][bb_instances.resolved_count[mesh_handle]++];
        
        inst->model = dm_mat_scale(dm_mat4_identity(), scale);
        inst->model = dm_mat4_mul_mat4(inst->model, dm_mat4_rotate_from_quat(rot));
        inst->model = dm_mat_translate(inst->model, pos);
        
#ifdef DM_DIRECTX
        inst->model = dm_mat4_transpose(inst->model);
#endif
        
        inst->color[0] = color_r[entity]; 
        inst->color[1] = color_g[entity]; 
        inst->color[2] = color_b[entity]; 
        inst->color[3] = color_a[entity];
        
        inst->brightness = brightness[entity];
    }
    
    blackbody_scene_uni uni = { 0 };
#ifdef DM_DIRECTX
    uni.view_proj = dm_mat4_transpose(app_data.camera.view_proj);
#else
    uni.view_proj = app_data.camera.view_proj;
#endif
    uni.fcoef_inv = 1.0f / dm_log2f(app_data.camera.far_plane + 1);
    
    // submit all render commands
    dm_render_command_begin_renderpass(bb_handles.pass);
    dm_render_command_bind_uniform(0, 0, 2, bb_handles.pass);
    dm_render_command_update_uniform(0, &uni, sizeof(uni), bb_handles.pass);
    
#ifdef DM_DEBUG
    static bool wireframe = false;
    
    if(dm_input_key_just_pressed(DM_KEY_SPACE)) wireframe = !wireframe;
    dm_render_command_toggle_wireframe(wireframe);
#endif
    
    dm_render_command_bind_buffer(bb_handles.ib, 0);
    dm_render_command_bind_buffer(bb_handles.vb, 0);
    dm_render_command_bind_buffer(bb_handles.instb, 1);
    
    for(uint32_t i=0; i<bb_handles.num_meshes; i++)
    {
        dm_mesh mesh = dm_renderer_get_mesh(bb_handles.meshes[i]);
        uint32_t num = bb_instances.resolved_count[i];
        if(num==0) continue;
        
        dm_render_command_update_buffer(bb_handles.instb, &bb_instances.resolved_instances[i], sizeof(blackbody_instance) * num, 0);
        dm_render_command_draw_instanced(mesh.index_count, num, mesh.index_offset, 0, 0);
    }
    
    dm_render_command_end_renderpass(bb_handles.pass);
    
    return true;
}

bool airy_disc_pass(dm_entity* entities, uint32_t entity_count)
{
    bb_instances.airy_disc_count = 0;
    
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    float* brightness = get_blackbody_member(BLACKBODY_MEM_BRIGHTNESS);
    float* color_r = get_blackbody_member(BLACKBODY_MEM_COLOR_R);
    float* color_g = get_blackbody_member(BLACKBODY_MEM_COLOR_G);
    float* color_b = get_blackbody_member(BLACKBODY_MEM_COLOR_B);
    float* color_a = get_blackbody_member(BLACKBODY_MEM_COLOR_A);
    
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        airy_disc_vertex* vertex = &bb_instances.airy_disc_instances[bb_instances.airy_disc_count++];
        
        vertex->pos[0] = pos_x[entity];
        vertex->pos[1] = pos_y[entity];
        vertex->pos[2] = pos_z[entity];
        
        vertex->color[0] = color_r[entity];
        vertex->color[1] = color_g[entity];
        vertex->color[2] = color_b[entity];
        vertex->color[3] = color_a[entity];
        
        vertex->brightness = brightness[entity];
    }
    
    blackbody_scene_uni uni = { 0 };
#ifdef DM_DIRECTX
    uni.view_proj = dm_mat4_transpose(app_data.camera.view_proj);
#else
    uni.view_proj = app_data.camera.view_proj;
#endif
    uni.fcoef_inv = 1.0f / dm_log2f(app_data.camera.far_plane + 1);
    
    // render commands
    // first render stars as points to texture
    //dm_render_command_bind_framebuffer(bb_handles.point_fb);
    
    dm_render_command_begin_renderpass(bb_handles.point_pass);
    dm_render_command_bind_uniform(0, 0, 2, bb_handles.point_pass);
    dm_render_command_update_uniform(0, &uni, sizeof(uni), bb_handles.point_pass);
    
    dm_render_command_bind_buffer(bb_handles.point_vb, 0);
    dm_render_command_update_buffer(bb_handles.point_vb, &bb_instances.airy_disc_instances, sizeof(airy_disc_vertex) * bb_instances.airy_disc_count, 0);
    
    dm_render_command_set_primitive_topology(DM_TOPOLOGY_POINT_LIST);
    dm_render_command_draw_arrays(0, bb_instances.airy_disc_count);
    
    dm_render_command_end_renderpass(bb_handles.point_pass);
    
    //dm_render_command_set_primitive_topology(DM_TOPOLOGY_TRIANGLE_LIST);
    
    /*
    // now use texture to blur each single pixel star out with airy disc, then render to a quad that fills screen
    dm_render_command_bind_default_framebuffer();
    
    airy_blur_uni blur_uni = { 0 };
    blur_uni.offset = 1.0f / 600.0f;
    
    dm_render_command_begin_renderpass(bb_handles.blur_pass);
    dm_render_command_bind_uniform(0, 0, bb_handles.blur_pass);
    dm_render_command_update_uniform(0, &blur_uni, sizeof(blur_uni), bb_handles.blur_pass);
    
    dm_render_command_bind_buffer(bb_handles.blur_vb, 0);
    dm_render_command_bind_framebuffer_texture(bb_handles.point_fb, 0);
    
    dm_render_command_draw_arrays(0, 6);
    
    dm_render_command_end_renderpass(bb_handles.blur_pass);
    */
    return true;
}

return_code __blackbody_pass_init(float* positions, float* tex_coords, uint32_t num_vertices, uint32_t* indices, uint32_t num_indices, dm_render_handle* mesh_handles, uint32_t num_meshes)
{
    blackbody_vertex* vertices = dm_alloc(sizeof(blackbody_vertex) * num_vertices);
    for(uint32_t i=0; i<num_vertices; i++)
    {
        uint32_t offset_v3 = i * 3;
        uint32_t offset_v2 = i * 2;
        blackbody_vertex vertex = {
            { positions[offset_v3], positions[offset_v3 + 1], positions[offset_v3 + 2] },
            { tex_coords[offset_v2], tex_coords[offset_v2 + 1] },
        };
        vertices[i] = vertex;
    }
    
    dm_vertex_attrib_desc attrib_descs[] = {
        DM_MAKE_VERTEX_ATTRIB("POSITION", blackbody_vertex, pos, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
        DM_MAKE_VERTEX_ATTRIB("TEXCOORD", blackbody_vertex, tex_coords, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 2, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_MODEL", blackbody_instance, model, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_MATRIX_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_DIFFUSE", blackbody_instance, color, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("BRIGHTNESS", blackbody_instance, brightness, DM_VERTEX_ATTRIB_CLASS_INSTANCE, DM_VERTEX_DATA_T_FLOAT, 1, 0, false),
    };
    
    dm_uniform unis[] = {
        { .data_size=sizeof(blackbody_scene_uni), .stage=DM_UNIFORM_STAGE_BOTH, .name="scene_uni" }
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
    if(!DM_CREATE_STATIC_INDEX_BUFFER(indices, uint32_t, num_indices, bb_handles.ib)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_STATIC_VERTEX_BUFFER(vertices, blackbody_vertex, num_vertices, bb_handles.vb)) return RESOURCE_CREATION_FAIL;
    if(!DM_CREATE_DYNAMIC_VERTEX_BUFFER(NULL, blackbody_instance, DM_MAX_INSTS, bb_handles.instb)) return RESOURCE_CREATION_FAIL;
    
#if 0
    if(!dm_renderer_create_framebuffer(true,true,true, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &bb_handles.point_fb)) return RESOURCE_CREATION_FAIL;
#endif
    
    dm_free(vertices);
    
#ifdef DM_OPENGL
    dm_render_handle vb_buffers[] = { bb_handles.vb, bb_handles.instb };
    if(!DM_RENDERER_CREATE_RENDERPASS("assets/shaders/blackbody_vertex.glsl", "assets/shaders/blackbody_pixel.glsl", vb_buffers, unis, attrib_descs, pipeline_desc, bb_handles.pass)) return RESOURCE_CREATION_FAIL;
#else
#ifdef DM_DIRECTX
    const char* vertex_src = "assets/shaders/blackbody_vertex.fxc";
    const char* pixel_src = "assets/shaders/blackbody_pixel.fxc";
#elif defined(DM_METAL)
    const char* vertex_src = "assets/shaders/blackbody.metallib";
    const char* pixel_src = "assets/shaders/blackbody.metallib";
#endif
    
    if(!DM_RENDERER_CREATE_RENDERPASS(vertex_src, pixel_src, unis, attrib_descs, pipeline_desc, bb_handles.pass)) return RESOURCE_CREATION_FAIL;
#endif
    
    // mesh handles
    dm_memcpy(bb_handles.meshes, mesh_handles, sizeof(dm_render_handle) * num_meshes);
    bb_handles.num_meshes = num_meshes;
    
    // point pass
    if(!DM_CREATE_DYNAMIC_VERTEX_BUFFER(NULL, airy_disc_vertex, DM_MAX_INSTS, bb_handles.point_vb)) return RESOURCE_CREATION_FAIL;
    
    dm_vertex_attrib_desc airy_point_attribs[] = {
        DM_MAKE_VERTEX_ATTRIB("POSITION", airy_disc_vertex, pos, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 3, 0, false),
        DM_MAKE_VERTEX_ATTRIB("OBJ_COLOR", airy_disc_vertex, color, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 4, 0, false),
        DM_MAKE_VERTEX_ATTRIB("BRIGHTNESS", airy_disc_vertex, brightness, DM_VERTEX_ATTRIB_CLASS_VERTEX, DM_VERTEX_DATA_T_FLOAT, 1, 0, false),
    };
    
#ifdef DM_DIRECTX
    const char* airy_vertex = "assets/shaders/airy_point_vertex.fxc";
    const char* airy_pixel = "assets/shaders/airy_point_pixel.fxc";
#elif defined(DM_METAL)
    const char* airy_vertex = "assets/shaders/airy_point.metallib";
    const char* airy_pixel = "assets/shaders/airy_point.metallib";
#endif
    
    if(!DM_RENDERER_CREATE_RENDERPASS(airy_vertex, airy_pixel, unis, airy_point_attribs, pipeline_desc, bb_handles.point_pass)) return RESOURCE_CREATION_FAIL;
    
    // blur pass
    float blur_vertices[] = {
        -0.5f, -0.5f, 0, 0,
        0.5f,  -0.5f, 1, 0,
        0.5f,   0.5f, 1, 1,
        
        0.5f,   0.5f, 1, 1,
        -0.5f,  0.5f, 0, 1,
        -0.5f, -0.5f, 0, 0
    };
    if(!__dm_renderer_create_static_vertex_buffer(blur_vertices, sizeof(blur_vertices), sizeof(float) * 4, &bb_handles.blur_vb)) return RESOURCE_CREATION_FAIL;
    
#ifndef DM_METAL
    dm_uniform blur_uni[] = {
        { .data_size=sizeof(airy_blur_uni), .name="scene_uni" }
    };
    
    dm_vertex_attrib_desc airy_blur_attribs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(float)*4, .offset=0, .count=2, .index=0, .normalized=false },
        { .name="TEXCOORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(float)*4, .offset=2 * sizeof(float), .count=2, .index=0, .normalized=false },
    };
    
    if(!DM_RENDERER_CREATE_RENDERPASS("assets/shaders/airy_blur_vertex.fxc", "assets/shaders/airy_blur_pixel.fxc", blur_uni, airy_blur_attribs, pipeline_desc, bb_handles.blur_pass)) return RESOURCE_CREATION_FAIL;
#endif
    // register render system
    dm_ecs_id blackbody_system_component_ids[] = { DM_COMPONENT_TRANSFORM, DM_COMPONENT_MESH, get_blackbody_id() };
    dm_ecs_id blackbody_system_exclude_ids[] = { get_light_id() };
    dm_ecs_id render_system;
    
    DM_ECS_REGISTER_SYSTEM_EXCLUDES(DM_ECS_SYSTEM_TIMING_BEGIN, blackbody_system_component_ids, blackbody_system_exclude_ids, update_blackbodies, render_system);
    
    DM_ECS_REGISTER_SYSTEM_EXCLUDES(DM_ECS_SYSTEM_TIMING_RENDER, blackbody_system_component_ids, blackbody_system_exclude_ids, resolved_pass, render_system);
    DM_ECS_REGISTER_SYSTEM_EXCLUDES(DM_ECS_SYSTEM_TIMING_RENDER, blackbody_system_component_ids, blackbody_system_exclude_ids, airy_disc_pass, render_system);
    
    return SUCCESS;
}

#define BLACKBODY_PASS_INIT(POSITIONS, TEXCOORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES) __blackbody_pass_init(POSITIONS, TEXCOORDS, NUM_VERTICES, INDICES, NUM_INDICES, MESHES, DM_ARRAY_LEN(MESHES))

#endif //BLACKBODY_PASS_H
