#define NUM_OBJECTS 5

typedef struct physics_test_data_t
{
    dm_entity entities[NUM_OBJECTS];
    bool debug_draw;
} physics_test_data;

static physics_test_data physics_data = { 0 };

return_code physics_test_init()
{
    // gravity system
    gravity_system_init();
    
    dm_physics_toggle_pause();
    
    const dm_vec4 gray = dm_vec4_set(0.25f,0.25f,0.25f,1.0f);
    
    physics_data.entities[0] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(physics_data.entities[0], dm_vec3_set(-1,0,-1), dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision(physics_data.entities[0], DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics_at_rest(physics_data.entities[0], 1e12f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(physics_data.entities[0], ICOSPHERE_MESH);
    dm_ecs_entity_add_material(physics_data.entities[0], gray, gray);
    
    //dm_physics_add_angular_momentum(physics_data.entities[0], dm_vec3_set(0,1e12f,0));
    
    physics_data.entities[1] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(physics_data.entities[1], dm_vec3_set(-10,0,-10), dm_vec3_set(0.25f,0.25f,0.25f), dm_quat_set(0,0,0,1));
#if 0
    dm_ecs_entity_add_collision(physics_data.entities[1], DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_mesh(physics_data.entities[1], ICOSPHERE_MESH);
#else
    dm_ecs_entity_add_collision(physics_data.entities[1], DM_COLLISION_SHAPE_BOX);
    dm_ecs_entity_add_mesh(physics_data.entities[1], BOX_MESH);
#endif
    dm_ecs_entity_add_physics_at_rest(physics_data.entities[1], 1.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(physics_data.entities[1], gray, gray);
    
    dm_physics_add_impulse(physics_data.entities[1], dm_vec3_set(5.5f,0,5.0f));
    
    return SUCCESS;
}

return_code physics_test_update(view_camera* camera)
{
    update_camera(dm_get_delta_time(), camera);
    
    return SUCCESS;
}

return_code physics_test_render()
{
    if(dm_input_key_just_pressed(DM_KEY_TAB)) physics_data.debug_draw = !physics_data.debug_draw;
    
    if(physics_data.debug_draw)
    {
        for(uint32_t i=0; i<NUM_OBJECTS; i++)
        {
            dm_debug_render_transform(physics_data.entities[i]);
        }
    }
    
    return SUCCESS;
}

