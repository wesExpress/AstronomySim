#include "components.h"

#define USE_GRAVITY

// constants
#define G            6.673e-11f
#define EARTH_G      9.8f
#define E_G_OVER_G   1.47e11f   // kg / m^2

// units are fractions of their real values, otherwise everything breaks
#define STELLAR_MASS   1.65e23f  // g
#define STELLAR_RADIUS 6.957e4f  // m

// data
#define MAX_ENTITIES   8192

#ifdef USE_GRAVITY
#ifdef DM_DEBUG
#define MAX_STARS      100
#else
#define MAX_STARS      500
#endif
#define MAX_SATELLITES 3 * MAX_STARS
#else
#define MAX_STARS 500
#define MAX_SATELLITES 3 * MAX_STARS
#endif

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
    
#ifdef USE_GRAVITY
    dm_physics_add_impulse(satellite, dm_vec3_add_vec3(host_v, dm_vec3_scale(v, vc)));
#endif
    dm_physics_add_angular_velocity(satellite, dm_vec3_set(0,0.01f,0));
    
    space_data.satellites[space_data.num_satellites++] = satellite;
    space_data.entities[space_data.num_entities++] = satellite;
    
    return satellite;
}

// generate a random star: first calculates a random temp, then derives mass and radius from that
dm_ecs_id create_star(dm_vec3 pos, dm_vec3 velocity)
{
    dm_ecs_id star = dm_ecs_create_entity();
    
    float temperature, mass, radius;
    
    // determine color
    float type = dm_random_float();
    
    // giants (8000K - 10000K), (2, 4) solar radii, 
    if(type > 0.9f) 
    {
        temperature = dm_random_float_range(8000,10000);
        radius      = dm_random_float_range(2,4) * STELLAR_RADIUS;
        mass        = dm_random_float_range(2,10) * STELLAR_MASS;
    }
    // main sequence stars (5000 - 8000K), 1 solar radii
    else if(type > 0.4f) 
    {
        temperature = dm_random_float_range(5000,7999);
        radius = dm_random_float_range(1,2) * STELLAR_RADIUS;
        mass = dm_random_float_range(1,2) * STELLAR_MASS;
    }
    // dwarfs (3000-5000), 0.5 solar radii
    else 
    {
        temperature = dm_random_float_range(3000,4999);
        radius = dm_random_float_range(0.25f,1) * STELLAR_RADIUS;
        mass = dm_random_float_range(0.25f,1) * STELLAR_MASS;
    }
    
    dm_ecs_entity_add_transform_v(star, pos, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(star, radius);
    dm_ecs_entity_add_physics_at_rest(star, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(star, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(star, dm_vec4_set(0,0,0,1), dm_vec4_set(0,0,0,1));
    add_blackbody_component(star, temperature);
    
#ifdef USE_GRAVITY
    dm_physics_add_impulse(star, velocity);
#endif
    
    space_data.stars[space_data.num_stars++] = star;
    space_data.entities[space_data.num_entities++] = star;
    
    // make between 1-3 planets
    float satellite_num = dm_random_float();
    uint32_t num = 0;
    if(satellite_num > 0.66f) num = 3;
    else if(satellite_num > 0.33f) num = 2;
    else num = 1;
    
    for(uint32_t i=0; i<3; i++)
    {
        float orbit = dm_random_float_range(radius * 100.0f, radius * 250.0f);
        create_satellite(star, 1e4f, orbit, 1e21f, dm_vec4_set(0.25f,0.75f,0.25f,1));
    }
    
    return star;
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
    
    dm_vec3 pos = dm_vec3_set(pos_x[host] + radii[host] + 10.0f, pos_y[host], pos_z[host]);
    dm_vec3 host_v = dm_vec3_set(vel_x[host], vel_y[host], vel_z[host]);
    
    dm_ecs_id player = dm_ecs_create_entity();
    
#if 1
    dm_ecs_entity_add_transform_v(player, pos, dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(player, 1);
    dm_ecs_entity_add_mesh(player, ICOSPHERE_MESH);
#else
    dm_ecs_entity_add_transform_v(player, pos, dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_box(player, dm_vec3_set(-0.5f,-0.5f,-0.5f), dm_vec3_set(0.5f,0.5f,0.5f));
    dm_ecs_entity_add_mesh(player, BOX_MESH);
#endif
    dm_ecs_entity_add_physics_at_rest(player, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(player, color, color);
    
#ifdef USE_GRAVITY
    //dm_physics_add_impulse(player, host_v);
#endif
    
    space_data.entities[space_data.num_entities++] = player;
    
    return player;
}

return_code app_init()
{
    // gravity system
#ifdef USE_GRAVITY
    gravity_system_init();
#endif
    
    //dm_physics_toggle_pause();
    
    // entities
    const float star_pos_range = 5e8f;
    const float star_vel_range = 1e2f;
    const float planet_mass    = 1e16f;
    const float planet_radius  = 1e3f;
    const float moon_mass      = 1e13f;
    const float moon_radius    = 1e2f;
    const dm_vec4 planet_color = { 0.25f,0.75f,0.25f,1 };
    const dm_vec4 moon_color   = { 0.75f,0.75f,0.75f,1 };
    
    for(uint32_t i=0; i<MAX_STARS; i++)
    {
        dm_vec3 star_pos = dm_vec3_set(dm_random_float_range(-star_pos_range,star_pos_range), dm_random_float_range(-star_pos_range,star_pos_range), dm_random_float_range(-star_pos_range,star_pos_range));
        
        dm_vec3 star_vel = dm_vec3_set(dm_random_float_range(-star_vel_range,star_vel_range), dm_random_float_range(-star_vel_range,star_vel_range), dm_random_float_range(-star_vel_range,star_vel_range));
        
        create_star(star_pos, star_vel);
    }
    
    PLAYER = create_player(space_data.satellites[0], 10.0f, dm_vec4_set(1,0,0,1));
    floating_origin_system_init(PLAYER);
    floating_origin_enable_rot(space_data.satellites[0]);
    
    return SUCCESS;
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
    float* vel_x = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_X);
    float* vel_y = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Y);
    float* vel_z = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Z);
    
    if(dm_input_key_just_pressed(DM_KEY_R)) floating_origin_enable_rot(space_data.satellites[0]);
    else if(dm_input_key_just_pressed(DM_KEY_T)) floating_origin_disable_rot();
    
    dm_vec3 pos = { pos_x[PLAYER], pos_y[PLAYER], pos_z[PLAYER] };
    dm_quat rot = dm_quat_set(rot_i[PLAYER], rot_j[PLAYER], rot_k[PLAYER], rot_r[PLAYER]);
    dm_vec3 vel = { vel_x[PLAYER], vel_y[PLAYER], vel_z[PLAYER] };
    float d = dm_vec3_len(pos);
    float v = dm_vec3_len(vel);  
    
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
    float speed = 200.0f * dm_get_delta_time();
    //speed *= 1e5f;
    
    dm_vec3 impulse = { 0 };
    if(dm_input_is_key_pressed(DM_KEY_W)) dm_vec3_add_vec3_inpl(impulse, camera->forward, &impulse);
    else if(dm_input_is_key_pressed(DM_KEY_S)) dm_vec3_sub_vec3_inpl(impulse, camera->forward, &impulse);
    
    if(dm_input_is_key_pressed(DM_KEY_A)) dm_vec3_sub_vec3_inpl(impulse, camera->right, &impulse);
    else if(dm_input_is_key_pressed(DM_KEY_D)) dm_vec3_add_vec3_inpl(impulse, camera->right, &impulse);
    
    dm_vec3_norm_inpl(&impulse);
    dm_vec3_scale_inpl(speed, &impulse);
    
    dm_physics_add_impulse(PLAYER, impulse);
    
    // camera is 1.7m off ground
    pos = dm_vec3_set(pos_x[PLAYER], pos_y[PLAYER], pos_z[PLAYER]);
    pos = dm_vec3_add_vec3(pos, dm_vec3_scale(space_data.align_axis, 1.7f));
    
    fps_camera(pos, space_data.align_axis, camera);
    //track_camera(pos, space_data.align_axis, 10.0f, camera);
    
    return SUCCESS;
}

return_code app_render(view_camera* camera)
{
    dm_debug_render_transform(PLAYER);
    
    return SUCCESS;
}