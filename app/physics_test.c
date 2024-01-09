#include "physics_test.h"
#include "components.h"
#include "dm.h"
#include <float.h>

#define N 5

#define STACK_WIDTH  N
#define STACK_HEIGHT N
#define STACK_DEPTH  N

void physics_test_init(application_data* app_data, dm_context* context)
{
    dm_entity entity;
    
    // ground
    entity = dm_ecs_entity_create(context);
    
    const float ground_mass = FLT_MAX;
    
    const float ground_width  = 50;
    const float ground_height = 0.25f;
    const float ground_depth  = 50;
    
    const float ground_half_width = ground_width * 0.5f;
    const float ground_half_height = ground_height * 0.5f;
    const float ground_half_depth = ground_depth * 0.5f;
    
    entity_add_transform(entity, app_data->components.transform, 0,0,0, ground_width,ground_height,ground_depth, 0,0,0,1, context);
    entity_add_statics(entity, app_data->components.physics, ground_mass, context);
    entity_add_collider_box(entity, app_data->components.collision, 0,0,0, ground_width,ground_height,ground_depth, context);
    entity_add_rigid_body_box(entity, app_data->components.rigid_body, ground_mass, -ground_half_width,-ground_half_height,-ground_half_depth, ground_half_width,ground_half_height,ground_half_depth, context);
    
    app_data->entities[app_data->entity_count++] = entity;
    
    // box stack
    const float box_height_start = 10.0f;
    
    const float box_width  = 1.0f;
    const float box_height = 1.0f;
    const float box_depth  = 1.0f;
    
    const float half_width  = box_width * 0.5f;
    const float half_height = box_height * 0.5f;
    const float half_depth  = box_depth * 0.5f;
    
    const float mass = 1.0f;
    
    float pos_x;
    float pos_y;
    float pos_z;
    
    for(uint32_t i=0; i<STACK_WIDTH; i++)
    {
        pos_x = -half_width + i * box_width - STACK_WIDTH * half_width;
        for(uint32_t j=0; j<STACK_HEIGHT; j++)
        {
            pos_y = -half_height + j * box_height + box_height_start;
            for(uint32_t k=0; k<STACK_DEPTH; k++)
            {
                pos_z = -half_depth + k * box_depth - STACK_DEPTH * half_depth;
                
                entity = dm_ecs_entity_create(context);
                
                entity_add_transform(entity, app_data->components.transform, pos_x,pos_y,pos_z, box_width,box_height,box_depth, 0,0,0,1, context);
                entity_add_kinematics(entity, app_data->components.physics, mass, 0,0,0, 0,0, context);
                entity_add_collider_box(entity, app_data->components.collision, 0,0,0, box_width, box_height, box_depth, context);
                entity_add_rigid_body_box(entity, app_data->components.rigid_body, mass, -half_width,-half_height,-half_depth, half_width,half_height,half_depth, context);
                
                app_data->entities[app_data->entity_count++] = entity;
            }
        }
    }
}

void physics_test_update(application_data* app_data, dm_context* context)
{
    const float box_mass = 1.0f;
    float force = 0.5f * -9.8f * box_mass;
    
    for(uint32_t i=0; i<app_data->entity_count; i++)
    {
        entity_apply_force(app_data->entities[i], app_data->components.physics, 0,force,0, context);
    }
    
    static int mesh_index = 1;
    
    // imgui
    dm_ecs_system_timing timing = app_data->physics_system_timing;
    dm_ecs_id sys_id = app_data->physics_system;
    dm_imgui_nuklear_context* imgui_nk_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_nk_ctx->ctx;
    
    if(nk_begin(ctx, "Timings", nk_rect(100,100, 250,250), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE | 
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        if(nk_tree_push(ctx, NK_TREE_TAB, "Physics", NK_MINIMIZED))
        {
            nk_value_float(ctx, "Broadphase average (ms)", physics_system_get_broadphase_average(timing, sys_id, context));
            nk_value_float(ctx, "Narrowphase average (ms)", physics_system_get_narrowphase_average(timing, sys_id, context));
            nk_value_float(ctx, "Constraints average (ms)", physics_system_get_constraints_average(timing, sys_id, context));
            nk_value_float(ctx, "Update average (ms)", physics_system_get_update_average(timing, sys_id, context));
            nk_value_float(ctx, "Total time (ms)", physics_system_get_total_time(timing, sys_id, context));
            nk_value_uint(ctx, "Num iterations", physics_system_get_num_iterations(timing, sys_id, context));
            
            nk_tree_pop(ctx);
        }
        
        if(nk_tree_push(ctx, NK_TREE_TAB, "Gravity", NK_MINIMIZED))
        {
            nk_value_float(ctx, "Naive (ms)", gravity_system_get_timing(app_data->gravity_system_timing, app_data->gravity_system, context));
            
            nk_tree_pop(ctx);
        }
    }
    nk_end(ctx);
    
    if(nk_begin(ctx, "Mesh Selector", nk_rect(100,400, 250,150), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE | 
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_int(ctx, "Mesh index", mesh_index);
        nk_layout_row_dynamic(ctx, 30, 2);
        if(nk_button_label(ctx, "Increment")) mesh_index++;
        else if(nk_button_label(ctx, "Decrement")) mesh_index--;
        mesh_index = DM_CLAMP(mesh_index, 1,NUM_PLANETS);
    }
    nk_end(ctx);
    
    // submit entities
    for(uint32_t i=0; i<app_data->entity_count; i++)
    {
        render_pass_submit_entity(app_data->entities[i], (uint32_t)mesh_index, context);
    }
}

bool physics_test_render(application_data* app_data, dm_context* context)
{
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1,context);
    
    if(!render_pass_render(context))       return false;
    if(!debug_render_pass_render(context)) return false;
    
    return true;
}