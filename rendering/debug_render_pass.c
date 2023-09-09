#include "debug_render_pass.h"

#include "../app/app.h"
#include "dm.h"

typedef struct debug_render_vertex_t
{
    float pos[N3];
} debug_render_vertex;

typedef struct debug_render_instance_t
{
    float model[M4];
    float color[N4];
} debug_render_instance;

typedef struct debug_render_uniform_t
{
    float view_proj[M4];
} debug_render_uniform;

#define MAX_INSTS_PER_FRAME 2048
typedef struct debug_render_pass_data_t
{
    dm_render_handle vb, instb, ib, shader, pipe, uni;
    
    uint32_t line_count, line_vertex_count, line_vertex_offset, line_index_count, line_index_offset;
    uint32_t cube_count, cube_vertex_count,cube_vertex_offset, cube_index_count, cube_index_offset;
    uint32_t bilboard_count, bilboard_vertex_count, bilboard_vertex_offset, bilboard_index_count, bilboard_index_offset;
    
    debug_render_instance line_insts[MAX_INSTS_PER_FRAME];
    debug_render_instance cube_insts[MAX_INSTS_PER_FRAME];
    debug_render_instance bilboard_insts[MAX_INSTS_PER_FRAME];
} debug_render_pass_data;

bool debug_render_pass_init(dm_context* context)
{
    application_data* app_data = context->app_data;
    app_data->debug_render_pass_data = dm_alloc(sizeof(debug_render_pass_data));
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    
    ///
    dm_vertex_attrib_desc attrib_descs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(debug_render_vertex), .offset=offsetof(debug_render_vertex, pos), .count=3, .index=0, .normalized=false },
        { .name="OBJ_MODEL", .data_t=DM_VERTEX_DATA_T_MATRIX_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(debug_render_instance), .offset=offsetof(debug_render_instance, model), .count=4, .index=0, .normalized=false},
        { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_INSTANCE, .stride=sizeof(debug_render_instance), .offset=offsetof(debug_render_instance, color), .count=4, .index=0, .normalized=false }
    };
    
    // pipeline desc
    dm_pipeline_desc pipe_desc = { 0 };
    pipe_desc.cull_mode          = DM_CULL_BACK;
    pipe_desc.winding_order      = DM_WINDING_COUNTER_CLOCK;
    pipe_desc.primitive_topology = DM_TOPOLOGY_LINE_LIST;
    pipe_desc.wireframe          = true;
    
    pipe_desc.depth              = true;
    pipe_desc.depth_comp         = DM_COMPARISON_LESS;
    
    pipe_desc.blend              = true;
    pipe_desc.blend_eq           = DM_BLEND_EQUATION_ADD;
    pipe_desc.blend_src_f        = DM_BLEND_FUNC_SRC_ALPHA;
    pipe_desc.blend_dest_f       = DM_BLEND_FUNC_ONE_MINUS_SRC_ALPHA;
    
    // vertex data
    // debug render uses: line (2 vertices), cubes (8 vertices), bilboard (quads, so 4 vertices)
    debug_render_vertex vertices[2 + 8 + 4] = { 0 };
    uint32_t            indices[2 + 24 + 6] = { 0 };
    uint32_t            vertex_count=0, index_count=0;
    uint32_t            index_offset;
    
    // line (line list)
    pass_data->line_vertex_offset = vertex_count;
    pass_data->line_index_offset  = index_count;
    
    vertices[vertex_count++] = (debug_render_vertex){ 0 };
    vertices[vertex_count++] = (debug_render_vertex){ DM_MATH_INV_SQRT3,DM_MATH_INV_SQRT3,DM_MATH_INV_SQRT3 };
    
    indices[index_count++]   = 0;
    indices[index_count++]   = 1;
    
    pass_data->line_vertex_count = 2;
    pass_data->line_index_count  = 2;
    
    index_offset = vertex_count;
    
    // cube (line list)
    pass_data->cube_vertex_offset = vertex_count;
    pass_data->cube_index_offset  = index_count;
    
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f,-0.5f,-0.5f };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f,-0.5f,-0.5f };
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f, 0.5f,-0.5f };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f, 0.5f,-0.5f };
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f,-0.5f, 0.5f };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f,-0.5f, 0.5f };
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f, 0.5f, 0.5f };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f, 0.5f, 0.5f };
    
    indices[index_count++]   = index_offset + 0;
    indices[index_count++]   = index_offset + 1;
    indices[index_count++]   = index_offset + 0;
    indices[index_count++]   = index_offset + 2;
    indices[index_count++]   = index_offset + 0;
    indices[index_count++]   = index_offset + 4;
    
    indices[index_count++]   = index_offset + 1;
    indices[index_count++]   = index_offset + 3;
    indices[index_count++]   = index_offset + 1;
    indices[index_count++]   = index_offset + 5;
    
    indices[index_count++]   = index_offset + 2;
    indices[index_count++]   = index_offset + 3;
    indices[index_count++]   = index_offset + 2;
    indices[index_count++]   = index_offset + 6;
    
    indices[index_count++]   = index_offset + 3;
    indices[index_count++]   = index_offset + 7;
    
    indices[index_count++]   = index_offset + 4;
    indices[index_count++]   = index_offset + 5;
    indices[index_count++]   = index_offset + 4;
    indices[index_count++]   = index_offset + 6;
    
    indices[index_count++]   = index_offset + 5;
    indices[index_count++]   = index_offset + 7;
    
    indices[index_count++]   = index_offset + 6;
    indices[index_count++]   = index_offset + 7;
    
    pass_data->cube_vertex_count = 8;
    pass_data->cube_index_count  = 24;
    
    index_offset = vertex_count;
    
    // bilboard (triangle list)
    pass_data->bilboard_vertex_offset = vertex_count;
    pass_data->bilboard_index_offset  = index_count;
    
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f,-0.5f, 0 };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f,-0.5f, 0 };
    vertices[vertex_count++] = (debug_render_vertex){  0.5f, 0.5f, 0 };
    vertices[vertex_count++] = (debug_render_vertex){ -0.5f, 0.5f, 0 };
    
    indices[index_count++]   = index_offset + 0;
    indices[index_count++]   = index_offset + 1;
    indices[index_count++]   = index_offset + 2;
    
    indices[index_count++]   = index_offset + 2;
    indices[index_count++]   = index_offset + 3;
    indices[index_count++]   = index_offset + 0;
    
    pass_data->bilboard_vertex_count = 4;
    pass_data->bilboard_index_count  = 6;
    
    index_offset = vertex_count;
    
    // buffers
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(debug_render_vertex), &pass_data->vb, context)) return false;
    if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(debug_render_instance) * MAX_INSTS_PER_FRAME, sizeof(debug_render_instance), &pass_data->instb, context)) return false;
    if(!dm_renderer_create_static_index_buffer(indices, sizeof(indices), &pass_data->ib, context)) return false;
    if(!dm_renderer_create_uniform(sizeof(debug_render_uniform), DM_UNIFORM_STAGE_VERTEX, &pass_data->uni, context)) return false;
    
    dm_shader_desc shader_desc = { 0 };
#ifdef DM_VULKAN
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.spv");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.spv");
#elif defined(DM_OPENGL)
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.glsl");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.glsl");
    
    shader_desc.vb_count = 2;
    shader_desc.vb[0] = pass_data->vb;
    shader_desc.vb[1] = pass_data->instb;
#elif defined(DM_DIRECTX)
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.fxc");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.fxc");
#else
    strcpy(shader_desc.vertex, "vertex_main");
    strcpy(shader_desc.pixel, "fragment_main");
    strcpy(shader_desc.master, "assets/shaders/debug.metallib");
#endif
    
    if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipe_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &pass_data->shader, &pass_data->pipe, context)) return false;
    
    return true;
}

void debug_render_pass_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    dm_free(app_data->debug_render_pass_data);
}

bool debug_render_lines(dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    
    dm_render_command_update_buffer(pass_data->instb, pass_data->line_insts, sizeof(pass_data->line_insts), 0, context);
    dm_render_command_draw_instanced(pass_data->line_index_count, pass_data->line_count, pass_data->line_index_offset, 0, 0, context);
    
    pass_data->line_count = 0;
    
    return true;
}

bool debug_render_cubes(dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    
    dm_render_command_update_buffer(pass_data->instb, pass_data->cube_insts, sizeof(pass_data->cube_insts), 0, context);
    dm_render_command_draw_instanced(pass_data->cube_index_count, pass_data->cube_count, pass_data->cube_index_offset, 0, 0, context);
    
    pass_data->cube_count = 0;
    
    return true;
}

bool debug_render_bilboards(dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    
    dm_render_command_set_primitive_topology(DM_TOPOLOGY_TRIANGLE_LIST, context);
    dm_render_command_toggle_wireframe(false, context);
    dm_render_command_update_buffer(pass_data->instb, pass_data->bilboard_insts, sizeof(pass_data->bilboard_insts), 0, context);
    dm_render_command_draw_instanced(pass_data->bilboard_index_count, pass_data->bilboard_count, pass_data->bilboard_index_offset, 0, 0, context);
    
    pass_data->bilboard_count = 0;
    
    return true;
}

bool debug_render_pass_render(dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    
    if(!pass_data->line_count && !pass_data->cube_count && !pass_data->bilboard_count)   return true;
    
    debug_render_uniform uni = { 0 };
#ifdef DM_DIRECTX
    dm_mat4_transpose(app_data->camera.view_proj, uni.view_proj);
#else
    dm_memcpy(uni.view_proj, app_data->camera.view_proj, sizeof(uni.view_proj));
#endif
    
    dm_render_command_bind_shader(pass_data->shader, context);
    dm_render_command_bind_pipeline(pass_data->pipe, context);
    dm_render_command_bind_buffer(pass_data->ib,    0, context);
    dm_render_command_bind_buffer(pass_data->vb,    0, context);
    dm_render_command_bind_buffer(pass_data->instb, 1, context);
    
    dm_render_command_bind_uniform(pass_data->uni, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(pass_data->uni, &uni, sizeof(uni), context);
    
    if(pass_data->line_count     && !debug_render_lines(context))     return false;
    if(pass_data->cube_count     && !debug_render_cubes(context))     return false;
    if(pass_data->bilboard_count && !debug_render_bilboards(context)) return false;
    
#ifdef DM_METAL
    dm_render_command_end_shader_encoding(pass_data->shader, context);
#endif

    return true;
}

// rendering funcs
void debug_render_line(const float pos_0[3], const float pos_1[3], const float color[4], dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    if(pass_data->bilboard_count >= MAX_INSTS_PER_FRAME) return;
    
    float line[3];
    dm_vec3_sub_vec3(pos_1, pos_0, line);
    float ref_vec[] = {
        DM_MATH_INV_SQRT3,
        DM_MATH_INV_SQRT3,
        DM_MATH_INV_SQRT3,
    };
    
    float len = dm_vec3_mag(line);
    const float len2 = dm_vec3_mag(ref_vec);
    const float s = len / len2;
    
    float scale[3] = { s,s,s };
    
    float rot[4];
    float rotate[M4];
    
    dm_quat_from_vectors(ref_vec, line, rot);
    dm_mat4_rotate_from_quat(rot, rotate);
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    dm_mat_scale_make(scale, inst.model);
    dm_mat4_mul_mat4(inst.model, rotate, inst.model);
    dm_mat_translate(inst.model, pos_0, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(pass_data->line_insts + pass_data->line_count++, &inst, sizeof(inst));
}

void debug_render_arrow(const float pos_0[3], const float pos_1[3], const float color[4], dm_context* context)
{
    debug_render_line(pos_0, pos_1, color, context);
    debug_render_bilboard(pos_1, 0.05f,0.05f, color, context); 
}

void debug_render_bilboard(const float pos[3], const float width, const float height, const float color[4], dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    if(pass_data->bilboard_count >= MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    float scale[3] = { width, height, 1 };
    
    dm_mat_scale(app_data->camera.inv_view, scale, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(pass_data->bilboard_insts + pass_data->bilboard_count++, &inst, sizeof(inst));
}

void debug_render_aabb(const float pos[3], const float dim[3], const float color[4], dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    if(pass_data->cube_count >= MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    dm_mat4_identity(inst.model);
    dm_mat_scale(inst.model, dim, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(pass_data->cube_insts + pass_data->cube_count++, &inst, sizeof(inst));
}

void debug_render_cube(const float pos[3], const float dim[3], const float orientation[4], const float color[4], dm_context* context)
{
    application_data*       app_data = context->app_data;
    debug_render_pass_data* pass_data = app_data->debug_render_pass_data;
    if(pass_data->cube_count >= MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    float rot[M4] = { 0 };
    dm_mat4_rotate_from_quat(orientation, rot);
    
    dm_mat4_identity(inst.model);
    dm_mat_scale(inst.model, dim, inst.model);
    dm_mat4_mul_mat4(inst.model, rot, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(pass_data->cube_insts + pass_data->cube_count++, &inst, sizeof(inst));
}