#include "space_sim.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

typedef struct space_sim_data_t
{
    view_camera camera;
    dm_entity entities[2];
} space_sim_data;

space_sim_data space_data = { 0 };

#define G          6.673e-11f
#define EARTH_G    9.8f
#define E_G_OVER_G 1.47e11f   // kg / m^2

return_code space_sim_init()
{
    const dm_vec4 white = dm_vec4_set(1,1,1,1);
    const float gray_scale = 0.5f;
    
    //const float r_moon = 1.7374e6f;   // m
    //const float m_moon = 7.3459e22f;  // kg
    float r_planet = 1e4f;
    const dm_vec4 c_moon = dm_vec4_set(white.x * gray_scale, white.y * gray_scale, white.z * gray_scale, 1);
    
    // gravity system
    gravity_system_init();
    
    // init camera
    init_camera(dm_vec3_set(r_planet * 4,0,0), dm_vec3_set(-1,0,0), 0.1f, 1e13f, 45.0f, 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &space_data.camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&space_data.camera.view_proj);
    dm_debug_render_set_inv_view(&space_data.camera.inv_view);
    dm_debug_render_set_far_plane(&space_data.camera.far_plane);
    
    // render passes
    float* positions = NULL;
    float* normals = NULL;
    float* tex_coords = NULL;
    uint32_t* indices = NULL;
    uint32_t num_vertices=0, num_indices=0;
    
    dm_render_handle meshes[2];
    uint32_t num_meshes=0;
    // box mesh
    dm_geometry_box(&positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // icosphere mesh
    dm_geometry_icosphere(4, &positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // submit data
    if(!default_pass_init(positions, normals, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), &space_data.camera)) return INIT_FAIL;
    
    dm_free(positions);
    dm_free(normals);
    dm_free(tex_coords);
    dm_free(indices);
    
    // entities
    // big planet and box for flying around
    
    // planet
    float m_planet = r_planet * r_planet * E_G_OVER_G;
    
    dm_vec3 scale = { r_planet,r_planet,r_planet };
    dm_vec3 pos   = { 0 };
    dm_quat rot   = { 0,0,0,1 };
    
    space_data.entities[0] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(space_data.entities[0], pos, scale, rot);
    dm_ecs_entity_add_collision(space_data.entities[0], DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics_at_rest(space_data.entities[0], m_planet, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC); 
    dm_ecs_entity_add_mesh(space_data.entities[0], 1);
    dm_ecs_entity_add_material(space_data.entities[0], c_moon, c_moon);
    
    dm_physics_add_angular_momentum(space_data.entities[0], dm_vec3_set(0.0f,1e25f,0.0f));
    
    // box (space ship lmao)
    scale = dm_vec3_set(10,0.5f,1);
    pos = dm_vec3_set(r_planet + 100.0f,0,0);
    rot = dm_quat_set(dm_random_float() * 2.0f - 1.0f, dm_random_float() * 2.0f - 1.0f, dm_random_float() * 2.0f - 1.0f, dm_random_float() * 2.0f - 1.0f);
    
    space_data.entities[1] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(space_data.entities[1], pos, scale, rot);
    dm_ecs_entity_add_collision(space_data.entities[1], DM_COLLISION_SHAPE_BOX);
    dm_ecs_entity_add_physics_at_rest(space_data.entities[1], 100.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC); 
    dm_ecs_entity_add_mesh(space_data.entities[1], 0);
    dm_ecs_entity_add_material(space_data.entities[1], dm_vec4_set(1,0,0,1), dm_vec4_set(1,0,0,1));
    
    return SUCCESS;
}

return_code space_sim_update()
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    dm_entity rocket = space_data.entities[1];
    dm_vec3 pos = { pos_x[rocket],pos_y[rocket],pos_z[rocket] };
    
    uint32_t width = DM_SCREEN_WIDTH;
    uint32_t height = DM_SCREEN_HEIGHT;
    
    // update camera
    resize_camera(width, height, &space_data.camera);
    
    static float distance = 30.0f;
    
    if(dm_input_is_key_pressed(DM_KEY_LSHIFT))
    {
        if(dm_input_mouse_has_scrolled())
        {
            int t = dm_input_get_mouse_scroll();
            distance += (float)t;
            
            distance = DM_CLAMP(distance, 10.0f, 50.0f);
        }
    }
    
    track_camera(pos, distance, &space_data.camera);
    
    return SUCCESS;
}

return_code space_sim_render()
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
    float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
    float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
    float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
    
    float* vel_x = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_X);
    float* vel_y = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Y);
    float* vel_z = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Z);
    
    dm_entity rocket = space_data.entities[1];
    dm_vec3 pos = { pos_x[rocket],pos_y[rocket],pos_z[rocket] };
    dm_vec3 vel = { vel_x[rocket],vel_y[rocket],vel_z[rocket] };
    dm_quat rot = { rot_i[rocket],rot_j[rocket],rot_k[rocket],rot_r[rocket] };
    
    dm_imgui_text_fmt(10,475, 1,1,0,1, "Rocket pos: x:%0.2f, y:%0.2f, z:%0.2f", pos.x, pos.y, pos.z);
    dm_imgui_text_fmt(10,500, 1,1,0,1, "Rocket vel: x:%0.2f, y:%0.2f, z:%0.2f", vel.x, vel.y, vel.z);
    dm_imgui_text_fmt(10,525, 1,1,0,1, "Rocket rot: i:%0.2f, j:%0.2f, k:%0.2f, r:%0.2f", rot.i,rot.j,rot.k,rot.r);
    
    dm_entity planet = space_data.entities[0];
    pos = dm_vec3_set(pos_x[planet], pos_y[planet], pos_z[planet]);
    vel = dm_vec3_set(vel_x[planet], vel_y[planet], vel_z[planet]);
    rot = dm_quat_set(rot_i[planet], rot_k[planet], rot_k[planet], rot_r[planet]);
    
    dm_imgui_text_fmt(10,550, 1,0,1,1, "Planet pos: x:%0.2f, y:%0.2f, z:%0.2f", pos.x, pos.y, pos.z);
    dm_imgui_text_fmt(10,575, 1,0,1,1, "Planet vel: x:%0.2f, y:%0.2f, z:%0.2f", vel.x, vel.y, vel.z);
    dm_imgui_text_fmt(10,600, 1,1,0,1, "Planet rot: i:%0.2f, j:%0.2f, k:%0.2f, r:%0.2f", rot.i,rot.j,rot.k,rot.r);
    
    return SUCCESS;
}