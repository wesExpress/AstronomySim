#include "app.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

#define BILBOARD_SIZE (0.05f)
#define M_SCALE       (1e9)    // kg
#define V_SCALE       (1.0f)   // m/s

#ifdef DM_DEBUG
#define D_SCALE (30.0f)
#define NUM_OBJECTS 512
//#define NUM_OBJECTS 1024
#else
#define D_SCALE (75.0f)
#define NUM_OBJECTS 1024
//#define NUM_OBJECTS 4096
#endif

dm_entity make_box()
{
    dm_entity entity = dm_ecs_create_entity();
    
    float hd = D_SCALE * 0.5f;
    float r = 2.0f;
    float hr = r * 0.5f;
    float hv = V_SCALE * 0.5f;
    
    dm_vec3 position = dm_vec3_set(dm_random_float() * D_SCALE - hd, dm_random_float() * D_SCALE - hd, dm_random_float() * D_SCALE - hd);
    dm_quat rot = dm_quat_set(dm_random_float() * r - hr, dm_random_float() * r - hr, dm_random_float() * r - hr, dm_random_float() * r - hr);
    rot = dm_quat_norm(rot);
    dm_vec3 scale;
    
    // mesh
    float rand_mesh = dm_random_float();
    dm_render_handle mesh_handle = rand_mesh > 0.05f ? 2 : 0;
    dm_ecs_entity_add_mesh(entity, mesh_handle);
    
    // collision
    switch(mesh_handle)
    {
        // box
        case 0:
        {
            scale = dm_vec3_set(dm_random_float(), dm_random_float(), dm_random_float());
            dm_ecs_entity_add_transform_v(entity, position, scale, rot);
            dm_ecs_entity_add_collision(entity, DM_COLLISION_SHAPE_BOX);
        } break;
        
        // icosahedron
        // or icosphere
        case 1:
        case 2:
        {
            float s = dm_random_float();
            scale = (dm_vec3) { s,s,s };
            dm_ecs_entity_add_transform_v(entity, position, scale, rot);
            dm_ecs_entity_add_collision(entity, DM_COLLISION_SHAPE_SPHERE);
        } break;
    }
    
    // physics
    float mass = M_SCALE * (scale.x + scale.y + scale.z) / 3.0f;
    //float l = M_SCALE * 1e-2;
    //float hl = l * 0.5f;
    
    dm_vec3 velocity = dm_vec3_set(dm_random_float() * V_SCALE - hv, dm_random_float() * V_SCALE - hv, dm_random_float() * V_SCALE - hv);
    //dm_vec3 ang_mtm  = dm_vec3_set(dm_random_float() * l - hl, dm_random_float() * l - hl, dm_random_float() * l - hl);
    
    //dm_ecs_entity_add_physics(entity, velocity, ang_mtm, dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_physics(entity, velocity, dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    
    // material
    float m_r = mass / M_SCALE;
    float m_b = 1.0f - m_r;
    dm_vec4 color = dm_vec4_set(m_r, 0, m_b, 1);
    
    dm_ecs_entity_add_material(entity, color, color);
    
    return entity;
}

void app_debug_draw(dm_entity* entities, uint32_t num_entities, view_camera camera)
{
    static bool draw_aabb = false;
    static bool draw_physics = false;
    
    if(dm_input_key_just_pressed(DM_KEY_1)) draw_aabb = !draw_aabb;
    if(dm_input_key_just_pressed(DM_KEY_2)) draw_physics = !draw_physics;
    
    for(uint32_t i=0; i<num_entities; i++)
    {
        dm_entity entity = entities[i];
        
        bool* is_colliding = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_IS_COLLIDING);
        bool* possibly_colliding = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_POSSIBLY_COLLIDING);
        
        dm_debug_render_transform(entity);
        
        if(draw_aabb)
        {
            dm_aabb aabb = dm_ecs_entity_get_aabb(entity);
            dm_debug_render_billboard_v(aabb.min, BILBOARD_SIZE, BILBOARD_SIZE, dm_vec4_set(0,0.25f,0.75f,1));
            dm_debug_render_billboard_v(aabb.max, BILBOARD_SIZE, BILBOARD_SIZE, dm_vec4_set(0.75f,0.25f,0,1));
            
            if(is_colliding[entity]) dm_debug_render_aabb(entity, 1,0,0,1);
            else if(possibly_colliding[entity]) dm_debug_render_aabb(entity, 1,1,0,1);
            else dm_debug_render_aabb(entity, 1,1,1,1);
        }
        
        if(draw_physics) 
        {
            //dm_debug_render_force_vector(entity);
            dm_debug_render_velocity_vector(entity);
            dm_debug_render_angular_velocity_vector(entity);
        }
    }
    
    // world origin
    dm_debug_render_line(0,0,0,1,0,0,1,1,0,0,1);
    dm_debug_render_line(0,0,0,0,1,0,1,0,1,0,1);
    dm_debug_render_line(0,0,0,0,0,1,1,0,0,1,1);
}

return_code app_run()
{
    // init camera
    view_camera camera = { 0 };
    float d = 6;
    init_camera(dm_vec3_set(d,d,d), dm_vec3_set(-1,-1,-1), 0.1f, 100.0f, 45.0f, 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&camera.view_proj);
    dm_debug_render_set_inv_view(&camera.inv_view);
    
    // render passes
    float* positions = NULL;
    float* normals = NULL;
    float* tex_coords = NULL;
    uint32_t* indices = NULL;
    uint32_t num_vertices=0, num_indices=0;
    
    dm_render_handle meshes[3];
    uint32_t num_meshes=0;
    // box mesh
    dm_geometry_box(&positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // icosahedorn mesh
    dm_geometry_icosahedron(&positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // icosphere mesh
    dm_geometry_icosphere(4, &positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // submit data
    if(!default_pass_init(positions, normals, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), &camera)) return INIT_FAIL;
    
    dm_free(positions);
    dm_free(normals);
    dm_free(tex_coords);
    dm_free(indices);
    
    // our test boxes
#if 1
    dm_entity boxes[NUM_OBJECTS];
    for(uint32_t i=0; i<NUM_OBJECTS; i++)
    {
        boxes[i] = make_box();
    }
#else
#define NB (3*3*3 + 1)
    dm_entity boxes[NB];
    
    const float plane_size = 15.0f;
    const float wall_height = 0.5f;
    
    // floor and walls
    boxes[0] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(boxes[0], dm_vec3_set(0,0,0), dm_vec3_set(plane_size,0.25f,plane_size), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_mesh(boxes[0], 0);
    dm_ecs_entity_add_material(boxes[0], dm_vec4_set(0.5f,0.5f,0.5f,1), dm_vec4_set(0.5f,0.5f,0.5f,1));
    dm_ecs_entity_add_collision(boxes[0], DM_COLLISION_SHAPE_BOX);
    dm_ecs_entity_add_physics_at_rest(boxes[0], 100.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_STATIC);
    
    uint32_t box = 1;
    const static float size = 0.99999f;
    const static float height = 5.0f;
    for(uint32_t z=0; z<3; z++)
    {
        for(uint32_t y=0; y<3; y++)
        {
            for(uint32_t x=0; x<3; x++)
            {
                dm_vec3 pos = dm_vec3_set((float)(x%3), (float)(y%3) + height, (float)(z%3));
                boxes[box] = dm_ecs_create_entity();
                dm_ecs_entity_add_transform_v(boxes[box], pos, dm_vec3_set(size,size,size), dm_quat_set(0,0,0,1));
                dm_ecs_entity_add_mesh(boxes[box], 0);
                dm_ecs_entity_add_material(boxes[box], dm_vec4_set(0,0.5f,0.5f,1), dm_vec4_set(0,0.5f,0.5f,1));
                dm_ecs_entity_add_collision(boxes[box], DM_COLLISION_SHAPE_BOX);
                dm_ecs_entity_add_physics_at_rest(boxes[box], dm_random_float(), DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
                box++;
            }
        }
    }
#endif
    
    // main loop
    double render_time     = 0;
    uint32_t frame_counter = 0;
    uint32_t fps           = 0;
    bool debug_draw        = false;
    
    dm_timer frame_timer = { 0 };
    dm_timer_start(&frame_timer);
    
    // gravity system
    gravity_system_init();
    
    while(true)
    {
        if(!dm_begin_update()) break;
        
        if(dm_timer_elapsed(&frame_timer) > 1)
        {
            fps = frame_counter;
            frame_counter = 0;
            dm_timer_start(&frame_timer);
        }
        
        if(dm_input_key_just_pressed(DM_KEY_P)) dm_physics_toggle_pause();
        
        if(dm_input_key_just_pressed(DM_KEY_TAB)) debug_draw = !debug_draw;
        
        uint32_t width = DM_SCREEN_WIDTH;
        uint32_t height = DM_SCREEN_HEIGHT;
        
        // update camera
        resize_camera(width, height, &camera);
        update_camera(dm_get_delta_time(), &camera);
        
        // render
        dm_timer render_timer = { 0 };
        dm_timer_start(&render_timer);
        
        if(!dm_renderer_begin_frame()) return RENDER_FAIL;
        
        // fps
        dm_imgui_text_fmt(10, 25, 1, 1, 1, 1, "FPS: %u", fps);
        
        // frame render time display
        dm_imgui_text_fmt(10, 50, 1, 1, 1, 1, "Render took: %0.2lf ms", render_time);
        
        // debug drawing
        if(debug_draw) app_debug_draw(boxes, DM_ARRAY_LEN(boxes), camera);
        
        // wrap up frame
        if(!dm_renderer_end_frame()) return RENDER_FAIL;
        
        render_time = dm_timer_elapsed_ms(&render_timer);
        
        if(!dm_end_update()) return RENDER_FAIL;
        
        // update objects
#if 0
        for(uint32_t i=1; i<NB; i++)
        {
            dm_physics_apply_earth_gravity(boxes[i]);
        }
#endif
        
        frame_counter++;
    }
    
    return SUCCESS;
}
