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
    dm_ecs_entity_add_collision_sphere(physics_data.entities[0], 1);
    dm_ecs_entity_add_physics_at_rest(physics_data.entities[0], 1e10f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(physics_data.entities[0], ICOSPHERE_MESH);
    dm_ecs_entity_add_material(physics_data.entities[0], gray, gray);
    
    physics_data.entities[1] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(physics_data.entities[1], dm_vec3_set(-4,0,-4), dm_vec3_set(0.25f,0.5f,0.25f), dm_quat_set(0.5f,0,1,1));
#if 1
    dm_ecs_entity_add_collision_capsule(physics_data.entities[1], 0.5f, 1);
    dm_ecs_entity_add_mesh(physics_data.entities[1], BOX_MESH);
#else
    dm_ecs_entity_add_collision_box(physics_data.entities[1], dm_vec3_set(-0.25f,-0.25f,-0.25f), dm_vec3_set(0.25f,0.25f,0.25f));
    dm_ecs_entity_add_mesh(physics_data.entities[1], BOX_MESH);
#endif
    dm_ecs_entity_add_physics_at_rest(physics_data.entities[1], 1.0f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_material(physics_data.entities[1], gray, gray);
    
    //dm_physics_add_impulse(physics_data.entities[1], dm_vec3_set(1,0,1));
    
    return SUCCESS;
}

return_code physics_test_update(view_camera* camera)
{
    update_camera(dm_get_delta_time(), camera);
    
    return SUCCESS;
}

return_code physics_test_render()
{
    dm_debug_render_transform(physics_data.entities[0]);
    dm_debug_render_aabb(physics_data.entities[0]);
    
    dm_debug_render_transform(physics_data.entities[1]);
    dm_debug_render_aabb(physics_data.entities[1]);
    
    return SUCCESS;
}

