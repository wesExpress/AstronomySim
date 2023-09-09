#include "imgui_render_pass.h"

#include "../app/app.h"

#include "stb_truetype/stb_truetype.h"

#include <stdarg.h>

typedef struct imgui_pass_vertex_t
{
    float pos[3];
    float tex_coords[2];
    float color[4];
} imgui_pass_vertex;

typedef struct imgui_pass_uniform_t
{
    float proj[M4];
} imgui_pass_uniform;

#define IMGUI_PASS_MAX_TEXT_COUNT 500
#define IMGUI_PASS_MAX_INST_COUNT IMGUI_PASS_MAX_TEXT_COUNT * 4
typedef struct imgui_pass_text_data_packet_t
{
    float x,y;
    float r,g,b,a;
    char* text;
} imgui_pass_text_data_packet;

typedef struct imgui_pass_data_t
{
    dm_render_handle instb, uni;
    dm_render_handle shader, pipe;
    dm_render_handle default_font, active_font;
    
    imgui_pass_text_data_packet text_packets[IMGUI_PASS_MAX_TEXT_COUNT];
    uint32_t text_count;
    
    imgui_pass_vertex text_vertices[IMGUI_PASS_MAX_INST_COUNT];
    uint32_t text_vertex_count;
} imgui_pass_data;

/************
SYSTEM FUNCS
**************/
void imgui_draw_text_internal(imgui_pass_text_data_packet text_packet, dm_context* context);

bool imgui_render_pass_init(dm_context* context)
{
    application_data* app_data = context->app_data;
    app_data->imgui_pass_data = dm_alloc(sizeof(imgui_pass_data));
    imgui_pass_data* pass_data = app_data->imgui_pass_data;
    
    // resources
    
    dm_vertex_attrib_desc attrib_descs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(imgui_pass_vertex), .offset=offsetof(imgui_pass_vertex, pos), .count=3, .index=0, .normalized=false },
        { .name="TEXCOORD", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(imgui_pass_vertex), .offset=offsetof(imgui_pass_vertex, tex_coords), .count=2, .index=0, .normalized=false },
        { .name="COLOR", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(imgui_pass_vertex), .offset=offsetof(imgui_pass_vertex, color), .count=4, .index=0, .normalized=false }
    };
    
    // pipeline desc
    dm_pipeline_desc pipeline_desc = { 0 };
    pipeline_desc.cull_mode = DM_CULL_BACK;
    pipeline_desc.winding_order = DM_WINDING_COUNTER_CLOCK;
    pipeline_desc.primitive_topology = DM_TOPOLOGY_TRIANGLE_LIST;
    
    pipeline_desc.blend = true;
    pipeline_desc.blend_eq = DM_BLEND_EQUATION_ADD;
    pipeline_desc.blend_src_f = DM_BLEND_FUNC_SRC_ALPHA;
    pipeline_desc.blend_dest_f = DM_BLEND_FUNC_ONE_MINUS_SRC_ALPHA;
    
    pipeline_desc.sampler_comp = DM_COMPARISON_ALWAYS;
    pipeline_desc.sampler_filter = DM_FILTER_LINEAR;
    pipeline_desc.u_mode = pipeline_desc.v_mode = pipeline_desc.w_mode = DM_TEXTURE_MODE_WRAP;
    
    if(!dm_renderer_create_dynamic_vertex_buffer(NULL, sizeof(imgui_pass_vertex) * IMGUI_PASS_MAX_INST_COUNT, sizeof(imgui_pass_vertex), &pass_data->instb, context)) return false;
    if(!dm_renderer_create_uniform(sizeof(imgui_pass_uniform), DM_UNIFORM_STAGE_VERTEX, &pass_data->uni, context)) return false;
    
    dm_shader_desc shader_desc = { 0 };
#ifdef DM_VULKAN
    strcpy(shader_desc.vertex, "assets/shaders/imgui_vertex.spv");
    strcpy(shader_desc.pixel, "assets/shaders/imgui_pixel.spv");
#elif defined(DM_OPENGL)
    strcpy(shader_desc.vertex, "assets/shaders/imgui_vertex.glsl");
    strcpy(shader_desc.pixel, "assets/shaders/imgui_pixel.glsl");
    
    shader_desc.vb_count = 1;
    shader_desc.vb[0] = pass_data->instb;
#elif defined(DM_DIRECTX)
    strcpy(shader_desc.vertex, "assets/shaders/imgui_vertex.fxc");
    strcpy(shader_desc.pixel, "assets/shaders/imgui_pixel.fxc");
#else
    strcpy(shader_desc.vertex, "vertex_main");
    strcpy(shader_desc.pixel, "fragment_main");
    strcpy(shader_desc.master, "assets/shaders/imgui.metallib");
#endif
    
    if(!dm_renderer_create_shader_and_pipeline(shader_desc, pipeline_desc, attrib_descs, DM_ARRAY_LEN(attrib_descs), &pass_data->shader, &pass_data->pipe, context)) return false;
    
    // load in font
    if(!dm_renderer_load_font("assets/fonts/Chicago.ttf", &pass_data->default_font, context)) return false;
    
    pass_data->active_font = pass_data->default_font;
    pass_data->text_count = 0;
    
    return true;
}

void imgui_render_pass_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    imgui_pass_data*  pass_data = app_data->imgui_pass_data;
    
    for(uint32_t i=0; i<IMGUI_PASS_MAX_TEXT_COUNT; i++)
    {
        if(pass_data->text_packets[i].text) dm_free(pass_data->text_packets[i].text);
    }
    
    dm_free(app_data->imgui_pass_data);
}

// loop through all of our command types and submit
bool imgui_render_pass_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    imgui_pass_data* pass_data = app_data->imgui_pass_data;
    
    if(pass_data->text_count == 0) return true;
    
    dm_render_command_bind_shader(pass_data->shader, context);
    dm_render_command_bind_pipeline(pass_data->pipe, context);
    dm_render_command_bind_buffer(pass_data->instb, 0, context);
    
    imgui_pass_uniform uni = { 0 };
    dm_mat_ortho(0.0f, (float)context->renderer.width, (float)context->renderer.height, 0, -1.0f, 1.0f, uni.proj);
#ifdef DM_DIRECTX
    dm_mat4_transpose(uni.proj, uni.proj);
#endif
    dm_render_command_bind_uniform(pass_data->uni, 0, DM_UNIFORM_STAGE_VERTEX, 0, context);
    dm_render_command_update_uniform(pass_data->uni, &uni, sizeof(uni), context);
    
    // text
    dm_font font = context->renderer.fonts[pass_data->active_font];
    dm_render_command_bind_texture(font.texture_index, 0, context);
    
    for(uint32_t i=0; i<pass_data->text_count; i++)
    {
        imgui_draw_text_internal(pass_data->text_packets[i], context);
    }
    
#ifdef DM_METAL
    dm_render_command_end_shader_encoding(pass_data->shader, context);
#endif

    pass_data->text_count = 0;
    
    return true;
}

/****************
RENDER FUNCTIONS
******************/
void imgui_set_font(dm_render_handle font, dm_context* context)
{
    application_data* app_data = context->app_data;
    imgui_pass_data* pass_data = app_data->imgui_pass_data;
    
    pass_data->active_font = font;
}

void imgui_draw_text(uint32_t x, uint32_t y, float r, float g, float b, float a, const char* text, dm_context* context)
{
    application_data* app_data = context->app_data;
    imgui_pass_data* pass_data = app_data->imgui_pass_data;
    
    imgui_pass_text_data_packet* packet = &pass_data->text_packets[pass_data->text_count];
    
    packet->x = (float)x;
    packet->y = (float)y;
    packet->r = r;
    packet->g = g;
    packet->b = b;
    packet->a = a;
    
    size_t str_len = strlen(text) + 1;
    
    if(!packet->text) packet->text = dm_alloc(str_len * sizeof(char));
    else              packet->text = dm_realloc(packet->text, str_len * sizeof(char));
    
    strcpy(packet->text, text);
    
    pass_data->text_count++;
}

void imgui_draw_text_fmt(uint32_t x, uint32_t y, float r, float g, float b, float a, dm_context* context, const char* text, ...)
{
#define IMGUI_PASS_BUFFER_LEN 5000
    char buffer[IMGUI_PASS_BUFFER_LEN];
    dm_memzero(buffer, sizeof(buffer));
    va_list ar_ptr;
    va_start(ar_ptr, text);
    vsnprintf(buffer, IMGUI_PASS_BUFFER_LEN, text, ar_ptr);
    va_end(ar_ptr);
    
    imgui_draw_text(x,y,r,g,b,a,buffer,context);
}

void imgui_draw_text_internal(imgui_pass_text_data_packet text_packet, dm_context* context)
{
    application_data* app_data = context->app_data;
    imgui_pass_data* pass_data = app_data->imgui_pass_data;
    
    dm_font font = context->renderer.fonts[pass_data->active_font];
    
    float text_len = 0.0f;
    float max_h    = 0.0f;
    
    // get text length and max height
    uint32_t num_glyphs = 0;
    float xf = (float)text_packet.x;
    float yf = (float)text_packet.y;
    
    const char* runner = text_packet.text;
    
    stbtt_aligned_quad q;
    while(*runner)
    {
        if(*runner >= 32 && *runner <= 127)
        {
            stbtt_GetBakedQuad((stbtt_bakedchar*)font.glyphs, 512, 512, *runner-32, &xf, &yf, &q, 1);
            
            text_len += (q.x1 - q.x0);
            max_h = DM_MAX(max_h, (q.y1 - q.y0));
            num_glyphs++;
        }
        runner++;
    }
    num_glyphs *= 6;
    runner = text_packet.text;
    
    while(*runner)
    {
        if(*runner >= 32 && *runner <= 127)
        {
            stbtt_GetBakedQuad((stbtt_bakedchar*)font.glyphs, 512, 512, *runner-32, &text_packet.x, &text_packet.y, &q, 1);
            
            // first triangle
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t0;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
            
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s1;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t1;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
            
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s1;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t0;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
            
            // second triangle
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s1;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t1;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
            
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t0;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
            
            pass_data->text_vertices[pass_data->text_vertex_count].pos[0]        = q.x0;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[1]        = q.y1;
            pass_data->text_vertices[pass_data->text_vertex_count].pos[2]        = 0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[0] = q.s0;
            pass_data->text_vertices[pass_data->text_vertex_count].tex_coords[1] = q.t1;
            pass_data->text_vertices[pass_data->text_vertex_count].color[0]      = text_packet.r;
            pass_data->text_vertices[pass_data->text_vertex_count].color[1]      = text_packet.g;
            pass_data->text_vertices[pass_data->text_vertex_count].color[2]      = text_packet.b;
            pass_data->text_vertices[pass_data->text_vertex_count].color[3]      = text_packet.a;
            pass_data->text_vertex_count++;
        }
        
        runner++;
    }
    
    dm_render_command_update_buffer(pass_data->instb, pass_data->text_vertices, num_glyphs * sizeof(imgui_pass_vertex), 0, context);
    dm_render_command_draw_arrays(0, num_glyphs, context);
    
    pass_data->text_vertex_count = 0;
}
