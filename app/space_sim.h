// constants
#define G          6.673e-11f
#define EARTH_G    9.8f
#define E_G_OVER_G 1.47e11f   // kg / m^2

// data
#define NUM_OBJECTS 4

typedef struct space_sim_data_t
{
    dm_entity entities[NUM_OBJECTS];
    bool align_with_grav;
    dm_entity nearest_grav_body;
    dm_vec3   align_axis;
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
    float r_star   = 1e3f;
    const dm_vec4 c_moon = dm_vec4_set(white.x * gray_scale, white.y * gray_scale, white.z * gray_scale, 1);
    
    // gravity system
    gravity_system_init();
    
    // entities
    // star, planet, moon, and box for flying around
    
    // star
    dm_vec3 scale = { r_star,r_star,r_star };
    dm_vec3 pos   = { 0 };
    dm_quat rot   = { 0,0,0,1 };
    const float star_mass = 1e20f;
    
    STAR = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(STAR, pos, scale, rot);
    dm_ecs_entity_add_collision_sphere(STAR, r_star);
    dm_ecs_entity_add_physics_at_rest(STAR, star_mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(STAR, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(STAR, dm_vec4_set(1,1,0,1), dm_vec4_set(1,1,0,1));
    
    dm_physics_add_angular_momentum(STAR, dm_vec3_set(0,1e29f,0));
    
    // planet 1
    const float planet_orbit = r_star * 100.0f;
    float m_planet = r_planet * r_planet * E_G_OVER_G;
    float vc = dm_sqrtf(G * star_mass / planet_orbit);
    pos = dm_vec3_set(0,0,planet_orbit);
    scale = dm_vec3_set(r_planet,r_planet,r_planet);
    
    PLANET_1 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(PLANET_1, pos, scale, rot);
    dm_ecs_entity_add_collision_sphere(PLANET_1, r_planet);
    dm_ecs_entity_add_physics(PLANET_1, dm_vec3_set(vc,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), m_planet, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(PLANET_1, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(PLANET_1, c_moon, c_moon);
    
    // space ship 
    scale = dm_vec3_set(1,1,1);
    pos = dm_vec3_set(0,0,planet_orbit + r_planet + 100.0f);
    
    ROCKET = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(ROCKET, pos, scale, rot);
#if 1
    dm_ecs_entity_add_collision_sphere(ROCKET, 1);
    dm_ecs_entity_add_mesh(ROCKET, ICOSPHERE_MESH);
#else
    dm_ecs_entity_add_collision_capsule(ROCKET, 0.5f, 2);
    dm_ecs_entity_add_mesh(ROCKET, BOX_MESH);
#endif
    dm_ecs_entity_add_physics(ROCKET, dm_vec3_set(vc,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), 10.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(ROCKET, dm_vec4_set(1,0,0,1), dm_vec4_set(1,0,0,1));
    
    dm_physics_add_impulse(ROCKET, dm_vec3_set(0,0,0));
    
    // moon
    const float moon_orbit = r_planet * 3.0f;
    const float moon_s = 0.1f;
    scale = dm_vec3_set(r_planet,r_planet,r_planet);
    scale = dm_vec3_scale(scale, moon_s);
    pos   = dm_vec3_set(0,0,planet_orbit + moon_orbit);
    vc += dm_sqrtf(G * m_planet / moon_orbit);
    
    PLANET_2 = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(PLANET_2, pos, scale, rot);
    dm_ecs_entity_add_collision_sphere(PLANET_2, r_planet * moon_s);
    dm_ecs_entity_add_physics(PLANET_2, dm_vec3_set(vc,0,0), dm_vec3_set(0,1e16f,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), m_planet * 0.1f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(PLANET_2, ICOSPHERE_MESH);
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

return_code space_sim_update(view_camera* camera)
{
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
    float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
    float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
    float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
    
    dm_vec3 pos = { pos_x[ROCKET],pos_y[ROCKET],pos_z[ROCKET] };
    dm_quat rot = dm_quat_set(rot_i[ROCKET], rot_j[ROCKET], rot_k[ROCKET], rot_r[ROCKET]);
    float   d   = dm_vec3_len(pos);
    
    if(d > 10000.0f) space_sim_update_positions(pos);
    
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
    
    dm_vec3 rocket_right   = dm_ecs_entity_get_transform_right(ROCKET);
    dm_vec3 rocket_up      = dm_ecs_entity_get_transform_up(ROCKET);
    dm_vec3 rocket_forward = dm_ecs_entity_get_transform_forward(ROCKET);
    
    // align with nearest gravitation object
    if(dm_input_is_key_pressed(DM_KEY_E))
    {
        float closest_d = FLT_MAX;
        for(uint32_t i=0; i<NUM_OBJECTS; i++)
        {
            if(space_data.entities[i] == ROCKET) continue;
            
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
        
        dm_quat new_rot = dm_quat_from_to_direction(dm_ecs_entity_get_transform_up(ROCKET), space_data.align_axis);
        new_rot = dm_quat_add_quat(new_rot, rot);
        new_rot = dm_quat_norm(new_rot);
        //new_rot = dm_quat_norm(dm_quat_nlerp(rot, new_rot, 0.3f));
        rot_i[ROCKET] = new_rot.i;
        rot_j[ROCKET] = new_rot.j;
        rot_k[ROCKET] = new_rot.k;
        rot_r[ROCKET] = new_rot.r;
    }
    
#if 0
    if(dm_input_is_key_pressed(DM_KEY_G))
    {
        float mag = 10000.0f;
        dm_vec3 force = dm_vec3_rotate(dm_vec3_unit_y, rot);
        dm_physics_apply_force(ROCKET, dm_vec3_scale(force, mag));
    }
#endif
    if(dm_input_is_key_pressed(DM_KEY_W))
    {
        dm_physics_apply_force(ROCKET, dm_vec3_scale(dm_ecs_entity_get_transform_forward(ROCKET), 1e4));
    }
    
    // update camera
    pos = dm_vec3_set(pos_x[ROCKET], pos_y[ROCKET], pos_z[ROCKET]);
    track_camera(pos, distance, camera);
    
    // update light pos
    pos = dm_vec3_set(pos_x[STAR], pos_y[STAR], pos_z[STAR]);
    default_pass_set_light_pos(pos);
    
    return SUCCESS;
}

return_code space_sim_render()
{
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
        dm_debug_render_aabb(PLANET_1);
        
        dm_debug_render_transform(PLANET_2);
        dm_debug_render_force_vector(PLANET_2);
        dm_debug_render_velocity_vector(PLANET_2);
        
        dm_debug_render_transform(ROCKET);
        dm_debug_render_force_vector(ROCKET);
        dm_debug_render_relative_velocity_vector(ROCKET, PLANET_1);
        dm_debug_render_aabb(ROCKET);
        
        dm_vec3 p = dm_vec3_set(pos_x[ROCKET], pos_y[ROCKET], pos_z[ROCKET]);
        dm_debug_render_arrow_v(p, dm_vec3_add_vec3(p, space_data.align_axis), 1.0f, dm_vec4_set(1,1,0,1));
    }
    
    return SUCCESS;
}