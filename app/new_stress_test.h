#define D_SCALE 20.0f
#define NUM_OBJECTS 256

#define M_SCALE  2e9f    // kg
#define V_SCALE  0.5f   // m/s
#define HD       D_SCALE * 0.5f
#define R        2.0f
#define HR       R * 0.5f
#define HV       V_SCALE * 0.5f

dm_entity objects[NUM_OBJECTS+1];

dm_entity make_object()
{
    dm_entity entity = dm_ecs_create_entity();
    
    float rand_mesh = dm_random_float();
    dm_render_handle mesh_handle = rand_mesh > 1.0 ? ICOSPHERE_MESH : BOX_MESH;
    dm_ecs_entity_add_mesh(entity, mesh_handle);
    
    float scale_x, scale_y, scale_z;
    float pos_x, pos_y, pos_z;
    float rot_i, rot_j, rot_k, rot_r;
    
    pos_x = dm_random_float() * D_SCALE - HD;
    pos_y = dm_random_float() * D_SCALE - HD;
    pos_z = dm_random_float() * D_SCALE - HD;
    
    scale_x = dm_random_float();
    scale_y = dm_random_float();
    scale_z = dm_random_float();
    
    rot_i = dm_random_float() * R - HR;
    rot_j = dm_random_float() * R - HR;
    rot_k = dm_random_float() * R - HR;
    rot_r = dm_random_float() * R - HR;
    
    switch(mesh_handle)
    {
        case BOX_MESH:
        scale_x = dm_random_float();
        scale_y = dm_random_float();
        scale_z = dm_random_float();
        
        dm_ecs_entity_add_transform(entity, pos_x,pos_y,pos_z, scale_x,scale_y,scale_z, rot_i,rot_j,rot_k,rot_r);
        dm_ecs_entity_add_collision_box(entity, dm_vec3_set(-scale_x * 0.5f,-scale_y * 0.5f,-scale_z * 0.5f), dm_vec3_set(scale_x * 0.5f,scale_y * 0.5f,scale_z * 0.5f));
        
        break;
        
        case ICOSPHERE_MESH:
        scale_y = scale_x;
        scale_z = scale_x;
        
        dm_ecs_entity_add_transform(entity, pos_x,pos_y,pos_z, scale_x * 0.5f,scale_y * 0.5f,scale_z * 0.5f, rot_i,rot_j,rot_k,rot_r);
        dm_ecs_entity_add_collision_sphere(entity, scale_x * 0.5f);
        
        break;
        
        default: break;
    }
    
    float mass = M_SCALE * (scale_x + scale_y + scale_z) / 3.0f;
    
    float vel_x = dm_random_float() * V_SCALE - HV;
    float vel_y = dm_random_float() * V_SCALE - HV;
    float vel_z = dm_random_float() * V_SCALE - HV;
    
    dm_ecs_entity_add_physics(entity, dm_vec3_set(vel_x,vel_y,vel_z), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    
    float m_r = mass / M_SCALE;
    float m_b = 1.0f - m_r;
    dm_vec4 color = { m_r,0,m_b,1 };
    
    dm_ecs_entity_add_material(entity, color, color);
    
    return entity;
}

return_code app_init()
{
    gravity_system_init();
    
    objects[0] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform(objects[0], 0,0,0, 1,1,1, 0,0,0,1);
    add_point_light_component(objects[0], dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec4_set(1,1,1,1), dm_vec3_set(0,0,0), 1,0.025f,0.009f);
    
    for(uint32_t i=1; i<NUM_OBJECTS + 1; i++)
    {
        objects[i] = make_object();
    }
    
    return SUCCESS;
}

return_code app_update(view_camera* camera)
{
    update_camera(camera);
    
    return SUCCESS;
}

return_code app_render()
{
    for(uint32_t i=0; i<NUM_OBJECTS; i++)
    {
        dm_debug_render_collider(objects[i+1]);
    }
    
    return SUCCESS;
}