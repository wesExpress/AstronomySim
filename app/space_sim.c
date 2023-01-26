#include "space_sim.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

// constants
#define G          6.673e-11f
#define EARTH_G    9.8f
#define E_G_OVER_G 1.47e11f   // kg / m^2

// data
#define NUM_OBJECTS 4

typedef struct space_sim_data_t
{
    view_camera camera;
    dm_entity entities[NUM_OBJECTS];
} space_sim_data;

space_sim_data space_data = { 0 };

// entity wrappers
#define STAR     space_data.entities[0]
#define PLANET_1 space_data.entities[1]
#define PLANET_2 space_data.entities[2]
#define ROCKET   space_data.entities[3]

return_code space_sim_init()
{
    const dm_vec4 white = dm_vec4_set(1,1,1,1);
    const float gray_scale = 0.5f;
    
    float r_planet = 100.0f; // m
    float r_star = r_planet * 1e3f;
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
    // star, planet, moon, and box for flying around
    
    // star
    dm_vec3 scale = { r_star,r_star,r_star };
    dm_vec3 pos   = { 0 };
    dm_quat rot   = { 0,0,0,1 };
    const float star_mass = 1e20f;
    
    STAR = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(STAR, pos, scale, rot);
    dm_ecs_entity_add_collision(STAR, DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics_at_rest(STAR, star_mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(STAR, 1);
    dm_ecs_entity_add_material(STAR, dm_vec4_set(1,1,0,1), dm_vec4_set(1,1,0,1));
    
    dm_physics_add_angular_momentum(STAR, dm_vec3_set(0,1e29f,0));
    
    // planet 1
    const float planet_orbit = r_star * 10.0f;
    float m_planet = r_planet * r_planet * E_G_OVER_G;
    float vc = dm_sqrtf(G * star_mass / planet_orbit);
    pos = dm_vec3_set(0,0,planet_orbit);
    scale = dm_vec3_set(r_planet,r_planet,r_planet);
    
    PLANET_1 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(PLANET_1, pos, scale, rot);
    dm_ecs_entity_add_collision(PLANET_1, DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics(PLANET_1, dm_vec3_set(vc,0,0), dm_vec3_set(0,5e18f,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), m_planet, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(PLANET_1, 1);
    dm_ecs_entity_add_material(PLANET_1, c_moon, c_moon);
    
    dm_physics_add_angular_momentum(PLANET_1, dm_vec3_set(0,1e18f,0));
    
    // sphere (space ship lmao)
    scale = dm_vec3_set(1,1,1);
    pos = dm_vec3_set(30.0f,0,planet_orbit + r_planet);
    
    ROCKET = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(ROCKET, pos, scale, rot);
    dm_ecs_entity_add_collision(ROCKET, DM_COLLISION_SHAPE_BOX);
    dm_ecs_entity_add_physics(ROCKET, dm_vec3_set(vc,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), 100.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(ROCKET, 0);
    dm_ecs_entity_add_material(ROCKET, dm_vec4_set(1,0,0,1), dm_vec4_set(1,0,0,1));
    
    //dm_physics_add_impulse(ROCKET, dm_vec3_set(0,0,-15));
    
    // moon
    const float moon_orbit = r_planet * 3.0f;
    scale = dm_vec3_set(r_planet,r_planet,r_planet);
    scale = dm_vec3_scale(scale, 0.1f);
    pos   = dm_vec3_set(0,0,planet_orbit + moon_orbit);
    vc += dm_sqrtf(G * m_planet / moon_orbit);
    
    PLANET_2 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(PLANET_2, pos, scale, rot);
    dm_ecs_entity_add_collision(PLANET_2, DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics(PLANET_2, dm_vec3_set(vc,0,0), dm_vec3_set(0,1e16f,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), m_planet * 0.01f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(PLANET_2, 1);
    dm_ecs_entity_add_material(PLANET_2, c_moon, c_moon);
    
    return SUCCESS;
}

void space_sim_update_positions(dm_vec3 p)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    for(uint32_t i=0; i<NUM_OBJECTS; i++)
    {
        pos_x[space_data.entities[i]] -= p.x;
        pos_y[space_data.entities[i]] -= p.y;
        pos_z[space_data.entities[i]] -= p.z;
    }
}

return_code space_sim_update()
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    float* force_x = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_X);
    float* force_y = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Y);
    float* force_z = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Z);
    
    dm_vec3 pos = { pos_x[ROCKET],pos_y[ROCKET],pos_z[ROCKET] };
    float   d = dm_vec3_len(pos);
    
    //if(d > 1000.0f) space_sim_update_positions(pos);
    
    // resize camera
    uint32_t width = DM_SCREEN_WIDTH;
    uint32_t height = DM_SCREEN_HEIGHT;
    
    resize_camera(width, height, &space_data.camera);
    
    // scrolling
    static float distance = 30.0f;
    
    if(dm_input_is_key_pressed(DM_KEY_LSHIFT))
    {
        if(dm_input_mouse_has_scrolled())
        {
            int t = dm_input_get_mouse_scroll();
            distance += (float)t;
            
            distance = DM_CLAMP(distance, 5.0f, 50.0f);
        }
    }
    
    // update camera
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
    float* w_x   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_X);
    float* w_y   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_Y);
    float* w_z   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_W_Z);
    
    dm_vec3 pos = { pos_x[ROCKET],pos_y[ROCKET],pos_z[ROCKET] };
    dm_vec3 vel = { vel_x[ROCKET],vel_y[ROCKET],vel_z[ROCKET] };
    dm_quat rot = { rot_i[ROCKET],rot_j[ROCKET],rot_k[ROCKET],rot_r[ROCKET] };
    
    dm_imgui_text_fmt(10,475, 1,1,0,1, "Rocket pos: x:%0.2f, y:%0.2f, z:%0.2f", pos.x, pos.y, pos.z);
    dm_imgui_text_fmt(10,500, 1,1,0,1, "Rocket vel: x:%0.2f, y:%0.2f, z:%0.2f", vel.x, vel.y, vel.z);
    dm_imgui_text_fmt(10,525, 1,1,0,1, "Rocket rot: i:%0.2f, j:%0.2f, k:%0.2f, r:%0.2f", rot.i,rot.j,rot.k,rot.r);
    
    pos = dm_vec3_set(pos_x[PLANET_1], pos_y[PLANET_1], pos_z[PLANET_1]);
    vel = dm_vec3_set(vel_x[PLANET_1], vel_y[PLANET_1], vel_z[PLANET_1]);
    rot = dm_quat_set(rot_i[PLANET_1], rot_k[PLANET_1], rot_k[PLANET_1], rot_r[PLANET_1]);
    dm_vec3 w = dm_vec3_set(w_x[PLANET_1], w_y[PLANET_1], w_z[PLANET_1]);
    
    dm_imgui_text_fmt(10,550, 1,0,1,1, "Planet pos: x:%0.2f, y:%0.2f, z:%0.2f", pos.x, pos.y, pos.z);
    dm_imgui_text_fmt(10,575, 1,0,1,1, "Planet vel: x:%0.2f, y:%0.2f, z:%0.2f", vel.x, vel.y, vel.z);
    dm_imgui_text_fmt(10,600, 1,1,0,1, "Planet rot: i:%0.2f, j:%0.2f, k:%0.2f, r:%0.2f", rot.i,rot.j,rot.k,rot.r);
    dm_imgui_text_fmt(10,625, 1,1,0,1, "Planet rot: x:%0.2f, y:%0.2f, z:%0.2f", w.x, w.y, w.z);
    
    pos = dm_vec3_set(pos_x[PLANET_2], pos_y[PLANET_2], pos_z[PLANET_2]);
    vel = dm_vec3_set(vel_x[PLANET_2], vel_y[PLANET_2], vel_z[PLANET_2]);
    rot = dm_quat_set(rot_i[PLANET_2], rot_k[PLANET_2], rot_k[PLANET_2], rot_r[PLANET_2]);
    w = dm_vec3_set(w_x[PLANET_2], w_y[PLANET_2], w_z[PLANET_2]);
    
    dm_imgui_text_fmt(10,650, 1,0,1,1, "Moon pos: x:%0.2f, y:%0.2f, z:%0.2f", pos.x, pos.y, pos.z);
    dm_imgui_text_fmt(10,675, 1,0,1,1, "Moon vel: x:%0.2f, y:%0.2f, z:%0.2f", vel.x, vel.y, vel.z);
    dm_imgui_text_fmt(10,700, 1,1,0,1, "Moon rot: i:%0.2f, j:%0.2f, k:%0.2f, r:%0.2f", rot.i,rot.j,rot.k,rot.r);
    dm_imgui_text_fmt(10,725, 1,1,0,1, "Moon rot: x:%0.2f, y:%0.2f, z:%0.2f", w.x, w.y, w.z);
    
    static bool debug_draw = false;
    
    if(dm_input_key_just_pressed(DM_KEY_TAB)) debug_draw = !debug_draw;
    
    if(debug_draw)
    {
        dm_debug_render_transform(PLANET_1);
        dm_debug_render_force_vector(PLANET_1);
        dm_debug_render_velocity_vector(PLANET_1);
        
        dm_debug_render_transform(PLANET_2);
        dm_debug_render_force_vector(PLANET_2);
        dm_debug_render_velocity_vector(PLANET_2);
        
        dm_debug_render_transform(ROCKET);
        dm_debug_render_force_vector(ROCKET);
        dm_debug_render_velocity_vector(ROCKET);
    }
    
    return SUCCESS;
}