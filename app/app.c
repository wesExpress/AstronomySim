#include "app.h"

#define DM_IMPLEMENTATION
#include "dm.h"

#include "camera.h"

#include "components.h"
#include "systems/physics_system.h"
#include "systems/gravity_system.h"

#include "rendering/render_pass.h"
#include "rendering/debug_render_pass.h"
#include "rendering/imgui_render_pass.h"

#define WORLD_SIZE 300
#define TIME_LIM 1.0f
void draw_path(application_data* app_data, dm_context* context)
{
    const dm_ecs_id t_id = app_data->components.transform;
    const component_transform* transform = dm_ecs_get_component_block(t_id, context);
    const dm_entity entity = app_data->entities[0];
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

dm_entity create_entity(application_data* app_data, dm_context* context)
{
    dm_entity entity = dm_ecs_create_entity(context);
    
    static const float h = (float)WORLD_SIZE * 0.5f;
    
    float pos_x = (dm_random_float(context) * 2 - 1) * h;
    float pos_y = (dm_random_float(context) * 2 - 1) * h;
    float pos_z = (dm_random_float(context) * 2 - 1) * h;
    
    float scale_x = dm_random_float_range(0.5,3,context);
    float scale_y = dm_random_float_range(0.5,3,context);
    float scale_z = dm_random_float_range(0.5,3,context);
    
    float rot_i = dm_random_float(context);
    float rot_j = dm_random_float(context);
    float rot_k = dm_random_float(context);
    float rot_r = dm_random_float(context);
    float mag   = dm_sqrtf(rot_i*rot_i + rot_j*rot_j + rot_k*rot_k + rot_r*rot_r);
    rot_i /= mag;
    rot_j /= mag;
    rot_k /= mag;
    rot_r /= mag;
    
    float vel_x = dm_random_float(context) * 2 - 1;
    float vel_y = dm_random_float(context) * 2 - 1;
    float vel_z = dm_random_float(context) * 2 - 1;
    
    float mass = dm_random_float(context) * 1e10;
    entity_add_kinematics(entity, app_data->components.physics, mass, vel_x,vel_y,vel_z, 0,0.1f, context);
    
    if(dm_random_float(context) > 1)
    {
        scale_y = scale_z = scale_x;
        
        float radius = scale_x * 0.5f;
        
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
    
    //entity_add_angular_velocity(entity, app_data->components.physics, dm_random_float(context),dm_random_float(context),dm_random_float(context), context);
    
    return entity;
}

void dm_application_setup(dm_context_init_packet* init_packet)
{
    //strcpy(init_packet->window_title, "TEST");
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
    if(!imgui_render_pass_init(context)) return false;
    
    // components
    if(!register_transform(&app_data->components.transform, context)) return false;
    if(!register_physics(&app_data->components.physics, context)) return false;
    if(!register_collision(&app_data->components.collision, context)) return false;
    if(!register_rigid_body(&app_data->components.rigid_body, context)) return false;
    
    // systems
    if(!physics_system_init(app_data->components.transform, app_data->components.collision, app_data->components.physics, app_data->components.rigid_body, context)) { DM_LOG_FATAL("Could not initialize physics system"); return false; }
    if(!gravity_system_init(app_data->components.transform, app_data->components.physics, context)) { DM_LOG_FATAL("Could not initialize gravity system"); return false; }
    
    // camera
    const float cam_pos[] = { -5,0,-5 };
    float cam_forward[] = { 1,0,1 };
    dm_vec3_norm(cam_forward, cam_forward);
    
    camera_init(cam_pos, cam_forward, 0.01f, 1000.0f, 75.0f, DM_SCREEN_WIDTH(context), DM_SCREEN_HEIGHT(context), 10.0f, 1.0f, &app_data->camera); 
    
    // entities
#if 1
    for(uint32_t i=0; i<MAX_ENTITIES; i++)
    {
        app_data->entities[app_data->entity_count++] = create_entity(app_data, context);
    }
#else
    dm_entity entity = dm_ecs_create_entity(context);
    
    float mass = 1e10f;
    entity_add_transform(entity, app_data->components.transform, 0,0,0, 1,1,1, 0,0,0,1, context);
    entity_add_box_collider(entity, app_data->components.collision, 0,0,0, 1,1,1, context);
    entity_add_kinematics_box_rigid_body(entity, app_data->components.physics, mass, 0,0,0, 0,0.1f, -0.5f,-0.5f,-0.5f,0.5f,0.5f,0.5f, context);
    app_data->entities[app_data->entity_count++] = entity;
    
    // orbiter
    entity = dm_ecs_create_entity(context);
    float radius = 3;
    entity_add_transform(entity, app_data->components.transform, radius,0,0, 0.1f,0.1f,0.1f, 0,0,0,1, context);
    entity_add_box_collider(entity, app_data->components.collision, 0,0,0, 1,1,1, context);
    entity_add_kinematics_box_rigid_body(entity, app_data->components.physics, 1, 0,0,0, 0,0.1f, -0.05f,-0.05f,-0.05f,0.05f,0.05f,0.05f, context);
    
    float vc = 6.67e-11f * mass / radius;
    vc = dm_sqrtf(vc);
    
    entity_add_velocity(entity, app_data->components.physics, 0,0,vc, context);
    
    app_data->entities[app_data->entity_count++] = entity;
#endif
    
    dm_timer_start(&app_data->draw_data.draw_timer, context);
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    render_pass_shutdown(context);
    debug_render_pass_shutdown(context);
    imgui_render_pass_shutdown(context);
    
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    camera_update(&app_data->camera, context);
    
    // submit entities
    for(uint32_t i=0; i<app_data->entity_count; i++)
    {
        render_pass_submit_entity(app_data->entities[i], context);
    }
    
    draw_path(app_data, context);
    
    imgui_draw_text_fmt(DM_SCREEN_WIDTH(context)-100,20, 0,1,0,1, context, "FPS: %0.2f", 1.0f / context->delta);
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    if(!render_pass_render(context))       return false;
    if(!debug_render_pass_render(context)) return false;
    if(!imgui_render_pass_render(context)) return false;
    
    return true;
}