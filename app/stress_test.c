#include "stress_test.h"

#include "dm.h"

#include "components.h"

#define WORLD_SIZE 150

dm_entity create_entity(application_data* app_data, dm_context* context)
{
    dm_entity entity = dm_ecs_entity_create(context);
    
    static const float h = (float)WORLD_SIZE * 0.5f;
    
    float pos_x = (dm_random_float(context) * 2 - 1) * h;
    float pos_y = (dm_random_float(context) * 2 - 1) * h;
    float pos_z = (dm_random_float(context) * 2 - 1) * h;
    
    //float scale_x = dm_random_float_range(0.5,3,context);
    //float scale_y = dm_random_float_range(0.5,3,context);
    //float scale_z = dm_random_float_range(0.5,3,context);
    
    float scale_x = 1;
    float scale_y = 1;
    float scale_z = 1;
    
    float rot_i = dm_random_float(context);
    float rot_j = dm_random_float(context);
    float rot_k = dm_random_float(context);
    float rot_r = dm_random_float(context);
    float mag   = dm_sqrtf(rot_i*rot_i + rot_j*rot_j + rot_k*rot_k + rot_r*rot_r);
    rot_i /= mag;
    rot_j /= mag;
    rot_k /= mag;
    rot_r /= mag;
    
    float mass = DM_MIN(scale_x, DM_MIN(scale_y, scale_z)) * 1e2f;
    entity_add_kinematics(entity, app_data->components.physics, mass, 0,0,0, 0,0.1f, context);
    
    if(dm_random_float(context) > 0)
    {
        scale_y = scale_z = scale_x;
        
        float radius = 20.0f * 0.5f;
        
        entity_add_collider_sphere(entity, app_data->components.collision, 0,0,0, radius, context);
        entity_add_rigid_body_sphere(entity, app_data->components.rigid_body, mass, radius, context);
    }
    else
    {
        float x_dim = scale_x * 0.5f;
        float y_dim = scale_y * 0.5f;
        float z_dim = scale_z * 0.5f;
        
        entity_add_collider_box(entity, app_data->components.collision, 0,0,0, scale_x,scale_y,scale_z, context);
        entity_add_rigid_body_box(entity, app_data->components.rigid_body, mass, -x_dim,-y_dim,-z_dim,x_dim,y_dim,z_dim, context);
    }
    
    entity_add_transform(entity, app_data->components.transform, pos_x,pos_y,pos_z, scale_x,scale_y,scale_z, rot_i,rot_j,rot_k,rot_r, context);
    
    return entity;
}

void stress_test_init(application_data* app_data, dm_context* context)
{
    for(uint32_t i=0; i<DM_ECS_MAX_ENTITIES; i++)
    {
        app_data->entities[app_data->entity_count++] = create_entity(app_data, context);
    }
}

void stress_test_update(application_data* app_data, dm_context* context)
{
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
}