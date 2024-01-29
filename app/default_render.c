#include "default_render.h"

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

typedef struct default_render_data_t
{
    dm_render_handle vb, instb, ib, uniform;
    dm_render_handle shader, pipeline, star_texture;
} default_render_data;

bool default_render_init(const uint32_t array_length, void** data, dm_context* context)
{
    *data = dm_alloc(sizeof(default_render_data));
    default_render_data* render_data = *data;
    
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
    if(!dm_renderer_create_static_index_buffer(indices, sizeof(uint32_t) * 6, sizeof(uint32_t), &render_data->ib, context)) return false;
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(render_vertex) * 4, sizeof(render_vertex), &render_data->vb, context)) return false;
    if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(render_instance) * array_length, sizeof(render_instance), &render_data->instb, context)) return false;
    if(!dm_renderer_create_uniform(sizeof(uniform_data), DM_UNIFORM_STAGE_VERTEX, &render_data->uniform, context)) return false;
    
    dm_shader_desc shader_desc = { 0 };
#ifdef DM_METAL
    strcpy(shader_desc.master, "assets/shaders/persp.metallib");
    strcpy(shader_desc.vertex, "vertex_main");
    strcpy(shader_desc.pixel, "fragment_main");
    shader_desc.vb[0] = render_data->vb;
    shader_desc.vb[1] = render_data->instb;
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
    
    if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipe_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &render_data->shader, &render_data->pipeline, context)) return false;
    
    
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
    
    if(!dm_renderer_create_texture_from_data(tex_width, tex_height, 4, tex_data, "star_texture", &render_data->star_texture, context)) return false;
    
    return true;
}

void default_render_shutdown(void** data, dm_context* context)
{
    dm_free(*data);
}

bool default_render_render(const uint32_t array_length, const dm_mat4 view_proj, const dm_mat4 inv_view, const float* pos_x, const float* pos_y, const float* pos_z, void** data, dm_context* context)
{
    default_render_data* render_data = *data;
    
    render_instance* instances = dm_alloc(sizeof(render_instance) * array_length);
    render_instance* inst = NULL;
    
    dm_vec3 pos;
    dm_vec3 scale;
    for(uint32_t i=0; i<array_length; i++)
    {
        inst = &instances[i];
        
        pos[0] = pos_x[i];
        pos[1] = pos_y[i];
        pos[2] = pos_z[i];
        
        switch(i)
        {
            case 0:
            scale[0] = 0.5f;
            scale[1] = 0.5f;
            scale[2] = 0.5f;
            
            inst->color[0] = 1;
            inst->color[1] = 1;
            inst->color[2] = 0;
            break;
            
            default:
            scale[0] = 0.15f;
            scale[1] = 0.15f;
            scale[2] = 0.15f;
            
            inst->color[0] = 1;
            inst->color[1] = 1;
            inst->color[2] = 1;
            break;
        }
        inst->color[3] = 1;
        
        dm_mat_scale(inv_view, scale, inst->model);
        
        // copy over position
        inst->model[3][0] = pos[0];
        inst->model[3][1] = pos[1];
        inst->model[3][2] = pos[2];
        
#ifdef DM_DIRECTX
        dm_mat4_transpose(inst->model, inst->model);
#endif
    }
    
    uniform_data uniform = { 0 };
#ifdef DM_DIRECTX
    dm_mat4_transpose(view_proj, uniform.view_proj);
#else
    dm_memcpy(uniform.view_proj, view_proj, sizeof(dm_mat4));
#endif
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    dm_render_command_bind_shader(render_data->shader, context);
    dm_render_command_bind_pipeline(render_data->pipeline, context);
    
    dm_render_command_bind_texture(render_data->star_texture, 0, context);
    
    dm_render_command_bind_buffer(render_data->ib, 0, context);
    dm_render_command_bind_buffer(render_data->vb, 0, context);
    dm_render_command_bind_buffer(render_data->instb, 1, context);
    dm_render_command_update_buffer(render_data->instb, instances, sizeof(render_instance) * array_length, 0, context);
    dm_render_command_bind_uniform(render_data->uniform, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(render_data->uniform, &uniform, sizeof(uniform), context);
    
    dm_render_command_draw_instanced(6, array_length, 0, 0, 0, context);
    
    dm_free(instances);
    
    return true;
}
