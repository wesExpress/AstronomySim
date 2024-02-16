#include "debug_render.h"

typedef enum debug_render_shapes_t
{
    DEBUG_RENDER_SHAPE_LINE,
    DEBUG_RENDER_SHAPE_CUBE,
    DEBUG_RENDER_SHAPE_BILBOARD,
    DEBUG_RENDER_SHAPE_UNKNOWN
} debug_render_shapes;

typedef struct debug_render_vertex_t
{
    dm_vec3 pos;
} debug_render_vertex;

typedef struct debug_render_instance_t
{
    dm_mat4 model;
    dm_vec4 color;
} debug_render_instance;

typedef struct debug_render_uniform_t
{
    dm_mat4 view_proj;
} debug_render_uniform;

#define DEBUG_MAX_INSTS_PER_FRAME 40000
typedef struct debug_render_data_t
{
    dm_render_handle vb, instb[DEBUG_RENDER_SHAPE_UNKNOWN], ib, shader, pipe, uni;
    
    uint32_t line_count, line_vertex_count, line_vertex_offset, line_index_count, line_index_offset;
    uint32_t cube_count, cube_vertex_count,cube_vertex_offset, cube_index_count, cube_index_offset;
    uint32_t bilboard_count, bilboard_vertex_count, bilboard_vertex_offset, bilboard_index_count, bilboard_index_offset;
    
    debug_render_instance line_insts[DEBUG_MAX_INSTS_PER_FRAME];
    debug_render_instance cube_insts[DEBUG_MAX_INSTS_PER_FRAME];
    debug_render_instance bilboard_insts[DEBUG_MAX_INSTS_PER_FRAME];
} debug_render_data;

bool debug_render_pass_init(void** data, dm_context* context)
{
    *data = dm_alloc(sizeof(debug_render_data));
    debug_render_data* render_data = *data;
    
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
    render_data->line_vertex_offset = vertex_count;
    render_data->line_index_offset  = index_count;
    
    vertices[vertex_count++] = (debug_render_vertex){ 0 };
    vertices[vertex_count++] = (debug_render_vertex){ DM_MATH_INV_SQRT3,DM_MATH_INV_SQRT3,DM_MATH_INV_SQRT3 };
    
    indices[index_count++]   = 0;
    indices[index_count++]   = 1;
    
    render_data->line_vertex_count = 2;
    render_data->line_index_count  = 2;
    
    index_offset = vertex_count;
    
    // cube (line list)
    render_data->cube_vertex_offset = vertex_count;
    render_data->cube_index_offset  = index_count;
    
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
    
    render_data->cube_vertex_count = 8;
    render_data->cube_index_count  = 24;
    
    index_offset = vertex_count;
    
    // bilboard (triangle list)
    render_data->bilboard_vertex_offset = vertex_count;
    render_data->bilboard_index_offset  = index_count;
    
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
    
    render_data->bilboard_vertex_count = 4;
    render_data->bilboard_index_count  = 6;
    
    // buffers
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(debug_render_vertex), &render_data->vb, context)) return false;
    for(uint32_t i=0; i<DEBUG_RENDER_SHAPE_UNKNOWN; i++)
    {
        if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(debug_render_instance) * DEBUG_MAX_INSTS_PER_FRAME, sizeof(debug_render_instance), &render_data->instb[i], context)) return false;
    }
    if(!dm_renderer_create_static_index_buffer(indices, sizeof(indices), sizeof(uint32_t), &render_data->ib, context)) return false;
    if(!dm_renderer_create_uniform(sizeof(debug_render_uniform), DM_UNIFORM_STAGE_VERTEX, &render_data->uni, context)) return false;
    
    dm_shader_desc shader_desc = { 0 };
#ifdef DM_VULKAN
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.spv");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.spv");
#elif defined(DM_OPENGL)
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.glsl");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.glsl");
    
    shader_desc.vb_count = 2;
    shader_desc.vb[0] = pass_data->vb;
    shader_desc.vb[1] = pass_data->instb[0];
#elif defined(DM_DIRECTX)
    strcpy(shader_desc.vertex, "assets/shaders/debug_vertex.fxc");
    strcpy(shader_desc.pixel, "assets/shaders/debug_pixel.fxc");
#else
    strcpy(shader_desc.vertex, "vertex_main");
    strcpy(shader_desc.pixel, "fragment_main");
    strcpy(shader_desc.master, "assets/shaders/debug.metallib");
#endif
    
    if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipe_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &render_data->shader, &render_data->pipe, context)) return false;
    
    return true;
}

void debug_render_pass_shutdown(void** data, dm_context* context)
{
    dm_free(*data);
}

DM_INLINE
bool debug_render_lines(void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    
    dm_render_command_bind_buffer(render_data->instb[DEBUG_RENDER_SHAPE_LINE], 1, context);
    dm_render_command_update_buffer(render_data->instb[DEBUG_RENDER_SHAPE_LINE], render_data->line_insts, sizeof(render_data->line_insts), 0, context);
    dm_render_command_draw_instanced(render_data->line_index_count, render_data->line_count, render_data->line_index_offset, 0, 0, context);
    
    render_data->line_count = 0;
    
    return true;
}

DM_INLINE
bool debug_render_cubes(void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    
    dm_render_command_bind_buffer(render_data->instb[DEBUG_RENDER_SHAPE_CUBE], 1, context);
    dm_render_command_update_buffer(render_data->instb[DEBUG_RENDER_SHAPE_CUBE], render_data->cube_insts, sizeof(render_data->cube_insts), 0, context);
    dm_render_command_draw_instanced(render_data->cube_index_count, render_data->cube_count, render_data->cube_index_offset, 0, 0, context);
    
    render_data->cube_count = 0;
    
    return true;
}

DM_INLINE
bool debug_render_bilboards(void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    
    dm_render_command_set_primitive_topology(DM_TOPOLOGY_TRIANGLE_LIST, context);
    dm_render_command_toggle_wireframe(false, context);
    dm_render_command_bind_buffer(render_data->instb[DEBUG_RENDER_SHAPE_BILBOARD], 1, context);
    dm_render_command_update_buffer(render_data->instb[DEBUG_RENDER_SHAPE_BILBOARD], render_data->bilboard_insts, sizeof(render_data->bilboard_insts), 0, context);
    dm_render_command_draw_instanced(render_data->bilboard_index_count, render_data->bilboard_count, render_data->bilboard_index_offset, 0, 0, context);
    
    render_data->bilboard_count = 0;
    
    return true;
}

bool debug_render_pass_render(void** data, const dm_mat4 view_proj, dm_context* context)
{
    debug_render_data* render_data = *data;
    
    if(!render_data->line_count && !render_data->cube_count && !render_data->bilboard_count)   return true;
    
    debug_render_uniform uni = { 0 };
#ifdef DM_DIRECTX
    dm_mat4_transpose(view_proj, uni.view_proj);
#else
    dm_memcpy(uni.view_proj, view_proj, sizeof(uni.view_proj));
#endif
    
    dm_render_command_bind_shader(render_data->shader, context);
    dm_render_command_bind_pipeline(render_data->pipe, context);
    dm_render_command_bind_buffer(render_data->ib,    0, context);
    dm_render_command_bind_buffer(render_data->vb,    0, context);
    
    dm_render_command_bind_uniform(render_data->uni, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(render_data->uni, &uni, sizeof(uni), context);
    
    if(render_data->line_count     && !debug_render_lines(data, context))     return false;
    if(render_data->cube_count     && !debug_render_cubes(data, context))     return false;
    if(render_data->bilboard_count && !debug_render_bilboards(data, context)) return false;
    
    return true;
}

// rendering funcs
void debug_render_line(const dm_vec3 pos_0, const dm_vec3 pos_1, const dm_vec4 color, void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    if(render_data->bilboard_count >= DEBUG_MAX_INSTS_PER_FRAME) return;
    
    dm_vec3 line;
    dm_vec3_sub_vec3(pos_1, pos_0, line);
    dm_vec3 ref_vec = {
        DM_MATH_INV_SQRT3,
        DM_MATH_INV_SQRT3,
        DM_MATH_INV_SQRT3,
    };
    
    float len = dm_vec3_mag(line);
    const float len2 = dm_vec3_mag(ref_vec);
    const float s = len / len2;
    
    dm_vec3 scale = { s,s,s };
    
    dm_quat rot;
    dm_mat4 rotate;
    
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
    
    dm_memcpy(render_data->line_insts + render_data->line_count++, &inst, sizeof(inst));
}

void debug_render_bilboard(const dm_vec3 pos, const float width, const float height, const dm_vec4 color, const dm_mat4 inv_view, void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    if(render_data->bilboard_count >= DEBUG_MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    float scale[3] = { width, height, 1 };
    
    dm_mat_scale(inv_view, scale, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(render_data->bilboard_insts + render_data->bilboard_count++, &inst, sizeof(inst));
}

void debug_render_arrow(const dm_vec3 pos_0, const dm_vec3 pos_1, const dm_vec4 color, const dm_mat4 inv_view, void** data, dm_context* context)
{
    debug_render_line(pos_0, pos_1, color, data, context);
    debug_render_bilboard(pos_1, 0.05f,0.05f, color, inv_view, data, context);
}

void debug_render_aabb(const dm_vec3 pos, const dm_vec3 dim, const dm_vec4 color, void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    if(render_data->cube_count >= DEBUG_MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    dm_mat_scale_make(dim, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(render_data->cube_insts + render_data->cube_count++, &inst, sizeof(inst));
}

void debug_render_cube(const dm_vec3 pos, const dm_vec3 dim, const dm_quat orientation, const dm_vec4 color, void** data, dm_context* context)
{
    debug_render_data* render_data = *data;
    if(render_data->cube_count >= DEBUG_MAX_INSTS_PER_FRAME) return;
    
    debug_render_instance inst = { 0 };
    dm_memcpy(inst.color, color, sizeof(inst.color));
    
    dm_mat4 rot = { 0 };
    dm_mat4_rotate_from_quat(orientation, rot);
    
    dm_mat4_identity(inst.model);
    dm_mat_scale(inst.model, dim, inst.model);
    dm_mat4_mul_mat4(inst.model, rot, inst.model);
    dm_mat_translate(inst.model, pos, inst.model);
#ifdef DM_DIRECTX
    dm_mat4_transpose(inst.model, inst.model);
#endif
    
    dm_memcpy(render_data->cube_insts + render_data->cube_count++, &inst, sizeof(inst));
}
