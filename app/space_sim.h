#include "components.h"

// constants
#define G          6.673e-11f
#define EARTH_G    9.8f
#define E_G_OVER_G 1.47e11f   // kg / m^2

// data
#define MAX_ENTITIES 10

typedef struct space_sim_data_t
{
    dm_entity entities[MAX_ENTITIES];
    uint32_t  num_entities;
    
    bool align_with_grav;
    dm_entity nearest_grav_body;
    dm_vec3   align_axis;
} space_sim_data;

space_sim_data space_data = { 0 };

// entities
dm_ecs_id STAR;
dm_ecs_id PLANET_1;
dm_ecs_id MOON_1;
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
    add_point_light_component(star, WHITE, WHITE, WHITE, dm_vec3_set(0,0,0), 1, 1e-5f, 1e-14f, COMPONENT_LIGHT);
    
    dm_physics_add_impulse(star, velocity);
    
    default_pass_add_point_light(star);
    
    space_data.entities[space_data.num_entities++] = star;
    
    return star;
}

dm_ecs_id create_satellite(dm_entity host, float radius, float orbit, float mass, dm_vec4 color)
{
    float* pos_x  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* radii  = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_UNION_0);
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
    
    dm_ecs_entity_add_transform_v(satellite, r, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(satellite, radius);
    dm_ecs_entity_add_physics_at_rest(satellite, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(satellite, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(satellite, color, color);
    
    dm_physics_add_impulse(satellite, dm_vec3_add_vec3(host_v, dm_vec3_scale(v, vc)));
    
    space_data.entities[space_data.num_entities++] = satellite;
    
    return satellite;
}

dm_ecs_id create_player(dm_entity host, float mass, dm_vec4 color)
{
    float* pos_x  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z  = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* radii  = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_UNION_0);
    float* masses = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_MASS);
    float* vel_x  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_X);
    float* vel_y  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Y);
    float* vel_z  = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_VEL_Z);
    
    dm_vec3 pos = dm_vec3_set(pos_x[host] + radii[host] + 100.0f, pos_y[host], pos_z[host]);
    dm_vec3 host_v = dm_vec3_set(vel_x[host], vel_y[host], vel_z[host]);
    
    dm_ecs_id player = dm_ecs_create_entity();
    
    dm_ecs_entity_add_transform_v(player, pos, dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    //dm_ecs_entity_add_collision_sphere(player, 1);
    dm_ecs_entity_add_collision_box(player, dm_vec3_set(-0.5f,-0.5f,-0.5f), dm_vec3_set(0.5f,0.5f,0.5f));
    dm_ecs_entity_add_physics_at_rest(player, mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(player, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(player, color, color);
    
    dm_physics_add_impulse(player, host_v);
    
    space_data.entities[space_data.num_entities++] = player;
    
    return player;
}

return_code space_sim_init()
{
    const dm_vec4 white = dm_vec4_set(1,1,1,1);
    const float gray_scale = 0.5f;
    
    float r_planet = 500.0f; // m
    float r_star   = 5e3f;
    const dm_vec4 c_moon = dm_vec4_set(white.x * gray_scale, white.y * gray_scale, white.z * gray_scale, 1);
    
    // gravity system
    gravity_system_init();
    
    // entities
    // star, planet, moon, and box for flying around
    
    STAR = create_star(dm_vec3_set(0,0,0), 5e3f, dm_vec3_set(0,0,0), 1e20f, dm_vec4_set(1,1,0,1));
    dm_physics_add_angular_momentum(STAR, dm_vec3_set(0,1e28f,0));
    
    // planet 1
    PLANET_1 = create_satellite(STAR, 500.0f, 5e5f, 1e17f, dm_vec4_set(0.5f,0.5f,0.5f,1));
    
    // player
    PLAYER = create_player(PLANET_1, 10.0f, dm_vec4_set(1,0,0,1));
    
    // moon
    MOON_1 = create_satellite(PLANET_1, 100.0f, 5e3f, 1e13f, dm_vec4_set(0.25f,0.25f,0.25f,1));
    
    return SUCCESS;
}

void space_sim_update_positions(dm_vec3 p)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    for(uint32_t i=0; i<space_data.num_entities; i++)
    {
        dm_vec3 pos = dm_vec3_set(pos_x[space_data.entities[i]], pos_y[space_data.entities[i]], pos_z[space_data.entities[i]]);
        
        pos_x[space_data.entities[i]] -= p.x;
        pos_y[space_data.entities[i]] -= p.y;
        pos_z[space_data.entities[i]] -= p.z;
        
        pos = dm_vec3_set(pos_x[space_data.entities[i]], pos_y[space_data.entities[i]], pos_z[space_data.entities[i]]);
    }
}

return_code space_sim_update(view_camera* camera)
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
    
    //if(d > 10000.0f) space_sim_update_positions(pos);
    space_sim_update_positions(pos);
    
    dm_vec3 rocket_right   = dm_ecs_entity_get_transform_right(PLAYER);
    dm_vec3 rocket_up      = dm_ecs_entity_get_transform_up(PLAYER);
    dm_vec3 rocket_forward = dm_ecs_entity_get_transform_forward(PLAYER);
    
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
        
        dm_quat new_rot = dm_quat_from_to_direction(rocket_up, space_data.align_axis);
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
    
    // camera is 1.7m off ground
    dm_vec3 camera_pos = dm_vec3_add_vec3(pos, dm_vec3_scale(rocket_up, 1.7f));
    fps_camera(dm_get_delta_time(), camera_pos, rocket_up, camera);
    
    return SUCCESS;
}

return_code space_sim_render()
{
#if 1 
    static bool debug_draw = false;
    
    if(dm_input_key_just_pressed(DM_KEY_TAB)) debug_draw = !debug_draw;
    
    if(debug_draw)
    {
        float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
        float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
        float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
        
        dm_debug_render_transform(PLANET_1);
        dm_debug_render_force_vector(PLANET_1);
        dm_debug_render_velocity_vector(PLANET_1);
        
        dm_debug_render_transform(MOON_1);
        dm_debug_render_force_vector(MOON_1);
        dm_debug_render_velocity_vector(MOON_1);
        
        dm_debug_render_transform(PLAYER);
        dm_debug_render_force_vector(PLAYER);
        dm_debug_render_relative_velocity_vector(PLAYER, PLANET_1);
        
        /*
        dm_vec3 p = dm_vec3_set(pos_x[ROCKET], pos_y[ROCKET], pos_z[ROCKET]);
        dm_debug_render_arrow_v(p, dm_vec3_add_vec3(p, space_data.align_axis), 1.0f, dm_vec4_set(1,1,0,1));
*/
    }
#endif
    
    return SUCCESS;
}