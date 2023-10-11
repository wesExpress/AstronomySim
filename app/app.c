#include "app.h"

#include "dm.h"

#include "camera.h"

#include "components.h"
#include "../systems/physics_system.h"
#include "../systems/gravity_system.h"

#include "../rendering/render_pass.h"
#include "../rendering/debug_render_pass.h"
#include "../rendering/imgui_render_pass.h"

#include "stress_test.h"
#include "physics_test.h"

#define PHYSICS_TEST

#define TIME_LIM 1.0f
void draw_path(application_data* app_data, dm_context* context)
{
    const dm_ecs_id t_id = app_data->components.transform;
    const component_transform* transform = dm_ecs_get_component_block(t_id, context);
    const dm_entity entity = app_data->entities[1];
    const uint32_t index   = dm_ecs_entity_get_component_index(entity, t_id, context);
    if(index==DM_ECS_INVALID_ENTITY) return;
    
    timer_draw_data* draw_data = &app_data->draw_data;
    
    if(dm_timer_elapsed(&draw_data->draw_timer, context) >= TIME_LIM)
    {
        for(uint32_t i=DRAW_LEN-1; i>0; i--)
        {
            draw_data->draw_pos_x[i] = draw_data->draw_pos_x[i-1];
            draw_data->draw_pos_y[i] = draw_data->draw_pos_y[i-1];
            draw_data->draw_pos_z[i] = draw_data->draw_pos_z[i-1];
        }
        
        draw_data->draw_pos_x[0] = transform->pos_x[index];
        draw_data->draw_pos_y[0] = transform->pos_y[index];
        draw_data->draw_pos_z[0] = transform->pos_z[index];
        
        dm_timer_start(&draw_data->draw_timer, context);
    }
    
    for(uint32_t i=0; i<DRAW_LEN; i++)
    {
        if(draw_data->draw_pos_x[0]==0 && draw_data->draw_pos_y[0]==0 && draw_data->draw_pos_z[0]==0) break;
        
        float r = (float)(DRAW_LEN - i) / (float)DRAW_LEN;
        
        debug_render_bilboard((float[]){ draw_data->draw_pos_x[i],draw_data->draw_pos_y[i],draw_data->draw_pos_z[i] }, 0.1f,0.1f, (float[]){ r,0,0,r}, context);
    }
}

void dm_application_setup(dm_context_init_packet* init_packet)
{
    //init_packet->window_width = 1920;
    //init_packet->window_height = 1080;
}

bool dm_application_init(dm_context* context)
{
    // app stuff
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    app_data->entity_count = 0;
    
    // rendering
    if(!render_pass_init(context))       return false;
    if(!debug_render_pass_init(context)) return false;
    //if(!imgui_render_pass_init(context)) return false;
    
    // components
    if(!register_transform(&app_data->components.transform, context))   return false;
    if(!register_physics(&app_data->components.physics, context))       return false;
    if(!register_collision(&app_data->components.collision, context))   return false;
    if(!register_rigid_body(&app_data->components.rigid_body, context)) return false;
    if(!register_mesh(&app_data->components.mesh, context))             return false;
    
    // systems
    app_data->physics_system_timing = DM_ECS_SYSTEM_TIMING_UPDATE_BEGIN;
    
    if(!physics_system_init(app_data->components.transform, app_data->components.collision, app_data->components.physics, app_data->components.rigid_body, app_data->physics_system_timing, &app_data->physics_system, context)) return false;
    //if(!gravity_system_init(app_data->components.transform, app_data->components.physics, context)) { DM_LOG_FATAL("Could not initialize gravity system"); return false; }
    
    // camera
    const float cam_pos[] = { 0,4,10.0f };
    float cam_forward[] = { 0,0,-1 };
    dm_vec3_norm(cam_forward, cam_forward);
    
    camera_init(cam_pos, cam_forward, 0.01f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), 5.0f, 1.0f, &app_data->camera); 
    
    // entities
#ifdef PHYSICS_TEST
    physics_test_init_entities(app_data, context);
#else
    stress_test_init_entities(app_data, context);
#endif
    
    dm_timer_start(&app_data->draw_data.draw_timer, context);
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    render_pass_shutdown(context);
    debug_render_pass_shutdown(context);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    camera_update(&app_data->camera, context);
    
#ifdef PHYSICS_TEST
    physics_test_update_entities(app_data, context);
#endif
    
    // submit entities
    for(uint32_t i=0; i<app_data->entity_count; i++)
    {
        render_pass_submit_entity(app_data->entities[i], context);
    }
    
    // imgui
    dm_ecs_system_timing timing = app_data->physics_system_timing;
    dm_ecs_id sys_id = app_data->physics_system;
    dm_imgui_nuklear_context* imgui_nk_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_nk_ctx->ctx;
    
    if(nk_begin(ctx, "Physics Timings", nk_rect(100,100, 250,250),  NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_SCALABLE|
                NK_WINDOW_MINIMIZABLE|NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_float(ctx, "Broadphase average (ms)", physics_system_get_broadphase_average(timing, sys_id, context));
        
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_float(ctx, "Narrowphase average (ms)", physics_system_get_narrowphase_average(timing, sys_id, context));
        
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_float(ctx, "Constraints average (ms)", physics_system_get_constraints_average(timing, sys_id, context));
        
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_float(ctx, "Update average (ms)", physics_system_get_update_average(timing, sys_id, context));
        
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_float(ctx, "Total time (ms)", physics_system_get_total_time(timing, sys_id, context));
        
        nk_layout_row_dynamic(ctx, 22, 1);
        nk_value_uint(ctx, "Num iterations", physics_system_get_num_iterations(timing, sys_id, context));
    }
    nk_end(ctx);
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0.1f,0.3f,0.5f,1,context);
    
    if(!render_pass_render(context))       return false;
    if(!debug_render_pass_render(context)) return false;
    
    return true;
}
