#define NUM_OBJECTS 100

typedef struct physics_test_data_t
{
    dm_entity entities[NUM_OBJECTS];
    uint32_t  num_entities;
    bool debug_draw;
} physics_test_data;

static physics_test_data physics_data = { 0 };
static dm_vec4 gray = { 0.25f,0.25f,0.25f,1.0f };

dm_entity make_sub_ball(view_camera* camera)
{
    dm_entity ball = dm_ecs_create_entity();
    
    static float radius = 0.25f;
    static float v      = 1.0f;
    
    dm_ecs_entity_add_transform_v(ball, camera->pos, dm_vec3_set(radius,radius,radius), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(ball, radius);
    dm_ecs_entity_add_physics_at_rest(ball, 1e3f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(ball, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(ball, gray, gray);
    
    dm_physics_add_impulse(ball, dm_vec3_scale(dm_vec3_norm(camera->forward), v));
    
    return ball;
}

return_code physics_test_init()
{
    // gravity system
    gravity_system_init();
    
    // light
    dm_entity entity = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(entity, dm_vec3_set(0,4,0), dm_vec3_set(0,0,0), dm_quat_set(0,0,0,1));
    add_point_light_component(entity, dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec3_set(0,0,0), 1, 0.09f, 0.032f, COMPONENT_LIGHT);
    
    default_pass_add_point_light(entity);
    
    physics_data.entities[physics_data.num_entities++] = entity;
    
    // massive object
    entity = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(entity, dm_vec3_set(0,0,0), dm_vec3_set(1,1,1), dm_quat_set(0,0,0,1));
    dm_ecs_entity_add_collision_sphere(entity, 1);
    dm_ecs_entity_add_physics_at_rest(entity, 1e12f, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_mesh(entity, ICOSPHERE_MESH);
    dm_ecs_entity_add_material(entity, gray, gray);
    
    //dm_physics_add_angular_momentum(entity, dm_vec3_set(0,1e13f,0));
    dm_physics_add_angular_velocity(entity, dm_vec3_set(0,1,0));
    
    physics_data.entities[physics_data.num_entities++] = entity;
    
    
    return SUCCESS;
}

return_code physics_test_update(view_camera* camera)
{
    update_camera(dm_get_delta_time(), camera);
    
    if(dm_input_mousebutton_just_pressed(DM_MOUSEBUTTON_L))
    {
        if(physics_data.num_entities < NUM_OBJECTS) physics_data.entities[physics_data.num_entities++] = make_sub_ball(camera);
        else DM_LOG_ERROR("Trying to create entity beyond max: %u", NUM_OBJECTS);
    }
    
    return SUCCESS;
}

return_code physics_test_render()
{
    for(uint32_t i=1; i<physics_data.num_entities; i++)
    {
        dm_debug_render_transform(physics_data.entities[i]);
        dm_debug_render_aabb(physics_data.entities[i]);
    }
    
    return SUCCESS;
}

