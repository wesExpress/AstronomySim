#include "dm.h"

typedef struct application_data_t
{
    dm_render_handle vb, ib;
    dm_render_handle pipeline;
} application_data;

typedef struct vertex_t
{
    dm_vec3 pos;
    dm_vec4 color;
} vertex;

void dm_application_setup(dm_context_init_packet* init_packet)
{
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    vertex vertices[] = {
        { { -0.5f,-0.5f,0 }, { 1,0,0,1} },
        { { 0.5f,-0.5f,0 }, { 0,1,0,1 } },
        { { 0,0.5f,0}, { 0,0,1,1 } }
    };
    
    dm_vertex_attrib_desc attribs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, pos), .count=3, .index=0, .normalized=false },
        { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, color), .count=4, .index=0, .normalized=false },
    };
    
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(vertex), &app_data->vb, context)) return false;
    
    dm_pipeline_desc pipe_desc = dm_renderer_default_pipeline();
    strcpy(pipe_desc.vertex_shader, "assets/shaders/dx12_vertex.fxc");
    strcpy(pipe_desc.pixel_shader, "assets/shaders/dx12_pixel.fxc");
    
    if(!dm_renderer_create_pipeline(pipe_desc, attribs, DM_ARRAY_LEN(attribs), &app_data->pipeline, context)) return false; 
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    dm_free(&context->app_data);
}

bool dm_application_update(dm_context* context)
{
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_render_command_bind_pipeline(app_data->pipeline, context);
    dm_render_command_bind_buffer(app_data->vb, 0, context);
    dm_render_command_draw_arrays(0,3, context);
    
    return true;
}
