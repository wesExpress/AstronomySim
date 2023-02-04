#include "components.h"

// constants
#define G          6.673e-11f
#define EARTH_G    9.8f
#define E_G_OVER_G 1.47e11f   // kg / m^2

// data
#define MAX_ENTITIES   500
#define MAX_STARS      50
#define MAX_SATELLITES 300

typedef struct space_sim_data_t
{
    dm_entity entities[MAX_ENTITIES];
    uint32_t  num_entities;
    
    dm_entity stars[MAX_STARS];
    uint32_t  num_stars;
    
    dm_entity satellites[MAX_SATELLITES];
    uint32_t  num_satellites;
    
    bool align_with_grav;
    dm_entity nearest_grav_body;
    dm_vec3   align_axis;
} space_sim_data;

space_sim_data space_data = { 0 };

// entities
dm_ecs_id STAR;
dm_ecs_id PLANET_1, PLANET_2;
dm_ecs_id MOON_1, MOON_2;
dm_ecs_id PLAYER;

#define WHITE    (dm_vec4){1,1,1,1}

dm_ecs_id create_star(dm_vec3 pos, float radius, dm_vec3 velocity, float mass, dm_vec4 color)
{
    dm_ecs_id star = dm_ecs_create_entity();
    
    dm_ecs_entity_add_transform_v(star, pos, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(star, radius);
    dm_ecs_entity_add_physics_at_rest(star, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(star, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(star, color, color);
    add_point_light_component(star, dm_vec4_set(0.01f,0.01f,0.01f,1), WHITE, WHITE, dm_vec3_set(0,0,0), 1, 1e-7f, 1e-14f, COMPONENT_LIGHT);
    
    dm_physics_add_impulse(star, velocity);
    
    default_pass_add_point_light(star);
    
    space_data.stars[space_data.num_stars++] = star;
    space_data.entities[space_data.num_entities++] = star;
    
    return star;
}

dm_ecs_id create_satellite(dm_entity host, float radius, float orbit, float mass, dm_vec4 color)
{
    float* pos_x  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* masses = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_MASS);
    float* vel_x  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_X);
    float* vel_y  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Y);
    float* vel_z  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Z);
    
    // set up for circular orbit
    float orbit_angle = dm_random_float() * DM_MATH_PI;
    float x = pos_x[host] + orbit * dm_cos(orbit_angle);
    float z = pos_z[host] + orbit * dm_sin(orbit_angle);
    
    dm_vec3 r = dm_vec3_set(x, 0, z);
    dm_vec3 v = dm_vec3_norm(dm_vec3_cross(r, dm_vec3_unit_y));
    dm_vec3 host_v = dm_vec3_set(vel_x[host], vel_y[host], vel_z[host]);
    
    float vc = dm_sqrtf(G * masses[host] / orbit);
    
    //
    dm_vec3 pos = dm_vec3_set(x, pos_y[host], z);
    
    dm_ecs_id satellite = dm_ecs_create_entity();
    
    dm_ecs_entity_add_transform_v(satellite, pos, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(satellite, radius);
    dm_ecs_entity_add_physics_at_rest(satellite, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(satellite, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(satellite, color, color);
    
    dm_physics_add_impulse(satellite, dm_vec3_add_vec3(host_v, dm_vec3_scale(v, vc)));
    dm_physics_add_angular_velocity(satellite, dm_vec3_set(0,0.05f,0));
    
    space_data.satellites[space_data.num_satellites++] = satellite;
    space_data.entities[space_data.num_entities++] = satellite;
    
    return satellite;
}

dm_ecs_id create_player(dm_entity host, float mass, dm_vec4 color)
{
    float* pos_x  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* radii  = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_UNION_0);
    float* vel_x  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_X);
    float* vel_y  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Y);
    float* vel_z  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Z);
    
    dm_vec3 pos = dm_vec3_set(pos_x[host] + radii[host] + 0.5f, pos_y[host], pos_z[host]);
    dm_vec3 host_v = dm_vec3_set(vel_x[host], vel_y[host], vel_z[host]);
    
    dm_ecs_id player = dm_ecs_create_entity();
    
    dm_ecs_entity_add_transform_v(player, pos, dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(player, 1);
    dm_ecs_entity_add_physics_at_rest(player, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(player, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(player, color, color);
    
    dm_physics_add_impulse(player, host_v);
    
    space_data.entities[space_data.num_entities++] = player;
    
    return player;
}

return_code app_init()
{
    // gravity system
    gravity_system_init();
    
    // entities
#if 0
    float r_planet = 500.0f; // m
    float r_star   = 5e3f;
    
    // star
    STAR = create_star(dm_vec3_set(0,0,0), r_star, dm_vec3_set(0,0,0), 1e22f, dm_vec4_set(1,1,0,1));
    
    // planet 1
    PLANET_1 = create_satellite(STAR, r_planet, 5e5f, 5e16f, dm_vec4_set(0.5f,0.5f,0.5f,1));
    dm_physics_add_angular_velocity(PLANET_1, dm_vec3_set(0,0.025f,0));
    
    // player
    PLAYER = create_player(PLANET_1, 10.0f, dm_vec4_set(1,0,0,1));
    
    // moon 1
    MOON_1 = create_satellite(PLANET_1, 50.0f, 2e3f, 1e13f, dm_vec4_set(0.5f,0.5f,0.5f,1));
    dm_physics_add_angular_velocity(MOON_1, dm_vec3_set(0,0.05f,0));
    
    // planet 2
    PLANET_2 = create_satellite(STAR, 500.0f, 5e4f, 5e16f, dm_vec4_set(0,1,0,1));
    dm_physics_add_angular_velocity(PLANET_2, dm_vec3_set(0,0.025f,0));
    
    // moon 2
    MOON_2 = create_satellite(PLANET_2, 50.0f, 5e3f, 1e13f, dm_vec4_set(0,0,1,1));
    dm_physics_add_angular_velocity(MOON_2, dm_vec3_set(0,0.05f,0));
#else
    const float star_mass      = 1e20f;
    const float star_radius    = 1e5f;
    const float planet_mass    = 1e19f;
    const float planet_radius  = 500.0f;
    const float moon_mass      = 1e13f;
    const float moon_radius    = 50.0f;
    const dm_vec4 star_color   = { 1,1,0,1 };
    const dm_vec4 planet_color = { 0.5f,0.5f,0.5f,1 };
    const dm_vec4 moon_color   = { 0.75f,0.75f,0.75f,1 };
    
    for(uint32_t i=0; i<MAX_STARS; i++)
    {
        dm_vec3 star_pos = dm_vec3_set(dm_random_float(), dm_random_float(), dm_random_float());
        star_pos = dm_vec3_scale(star_pos, 1e9f);
        star_pos = dm_vec3_sub_scalar(star_pos, 5e8f);
        
        dm_vec3 star_vel = dm_vec3_set(dm_random_float(), dm_random_float(), dm_random_float());
        star_vel = dm_vec3_scale(star_vel, 100.0f);
        star_vel = dm_vec3_sub_scalar(star_vel, 50.0f);
        
        if(i==0) create_star(star_pos, star_radius, star_vel, star_mass, dm_vec4_set(0,1,0,1));
        else create_star(star_pos, star_radius, star_vel, star_mass, star_color);
        
        for(uint32_t j=0; j<3; j++)
        {
            float orbit = dm_random_float() * 5e7f + 1e6f;
            create_satellite(space_data.stars[i], planet_radius, orbit, planet_mass, planet_color);
        }
    }
    
    //PLANET_1 = create_satellite(space_data.stars[0], planet_radius, 5e5f, planet_mass, planet_color);
    //dm_physics_add_angular_velocity(PLANET_1, dm_vec3_set(0,0.025f,0));
    PLAYER = create_player(space_data.satellites[0], 10.0f, dm_vec4_set(1,0,0,1));
#endif
    
    return SUCCESS;
}

void space_sim_update_positions(dm_vec3 p)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    for(uint32_t i=0; i<space_data.num_entities; i++)
    {
        pos_x[space_data.entities[i]] -= p.x;
        pos_y[space_data.entities[i]] -= p.y;
        pos_z[space_data.entities[i]] -= p.z;
    }
}

return_code app_update(view_camera* camera)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
    float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
    float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
    float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
    
    dm_vec3 pos = { pos_x[PLAYER], pos_y[PLAYER], pos_z[PLAYER] };
    dm_quat rot = dm_quat_set(rot_i[PLAYER], rot_j[PLAYER], rot_k[PLAYER], rot_r[PLAYER]);
    float   d   = dm_vec3_len(pos);
    
    if(d >= 1e4f) space_sim_update_positions(pos);
    
    dm_vec3 player_up = dm_ecs_entity_get_transform_up(PLAYER);
    
    // align with nearest gravitation object
    {
        float closest_d = FLT_MAX;
        for(uint32_t i=0; i<PLAYER; i++)
        {
            if(space_data.entities[i] == PLAYER) continue;
            
            dm_vec3 pos_g = dm_vec3_set(pos_x[space_data.entities[i]], pos_y[space_data.entities[i]], pos_z[space_data.entities[i]]);
            dm_vec3 sep = dm_vec3_sub_vec3(pos, pos_g);
            float d2 = dm_vec3_len2(sep);
            
            if(d2 < closest_d)
            {
                space_data.nearest_grav_body = space_data.entities[i];
                space_data.align_axis = dm_vec3_norm(sep);
                closest_d = d2;
            }
        }
        
        dm_quat new_rot = dm_quat_from_to_direction(player_up, space_data.align_axis);
        new_rot = dm_quat_mul_quat(new_rot, rot);
        new_rot = dm_quat_norm(new_rot);
        
        rot_i[PLAYER] = new_rot.i;
        rot_j[PLAYER] = new_rot.j;
        rot_k[PLAYER] = new_rot.k;
        rot_r[PLAYER] = new_rot.r;
    }
    
    // update camera
    pos = dm_vec3_set(pos_x[PLAYER], pos_y[PLAYER], pos_z[PLAYER]);
    float speed = 2.0f * dm_get_delta_time();
    
    dm_vec3 impulse = { 0 };
    if(dm_input_is_key_pressed(DM_KEY_W)) dm_vec3_add_vec3_inpl(impulse, camera->forward, &impulse);
    else if(dm_input_is_key_pressed(DM_KEY_S)) dm_vec3_sub_vec3_inpl(impulse, camera->forward, &impulse);
    
    if(dm_input_is_key_pressed(DM_KEY_A)) dm_vec3_sub_vec3_inpl(impulse, camera->right, &impulse);
    else if(dm_input_is_key_pressed(DM_KEY_D)) dm_vec3_add_vec3_inpl(impulse, camera->right, &impulse);
    
    dm_vec3_norm_inpl(&impulse);
    dm_vec3_scale_inpl(speed, &impulse);
    
    dm_physics_add_impulse(PLAYER, impulse);
    
    return SUCCESS;
}

return_code app_render(view_camera* camera)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    dm_vec3 pos = { pos_x[PLAYER], pos_y[PLAYER], pos_z[PLAYER] };
    
#if 1
    static bool debug_draw = false;
    
    if(dm_input_key_just_pressed(DM_KEY_TAB)) debug_draw = !debug_draw;
    
    if(debug_draw)
    {
        dm_debug_render_transform(space_data.satellites[0]);
        dm_debug_render_force_vector(space_data.satellites[0]);
        dm_debug_render_velocity_vector(space_data.satellites[0]);
        
        dm_debug_render_transform(PLAYER);
        dm_debug_render_force_vector(PLAYER);
        dm_debug_render_relative_velocity_vector(PLAYER, space_data.satellites[0]);
    }
#endif
    
    // camera is 1.7m off ground
    dm_vec3 player_up = dm_ecs_entity_get_transform_up(PLAYER);
    dm_vec3 camera_pos = dm_vec3_add_vec3(pos, dm_vec3_scale(player_up, 1.7f));
    fps_camera(dm_get_delta_time(), camera_pos, player_up, camera);
    
    return SUCCESS;
}