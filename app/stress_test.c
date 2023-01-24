#include "stress_test.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

#define BILBOARD_SIZE (0.05f)
#define M_SCALE       (1e9)    // kg
#define V_SCALE       (1.0f)   // m/s

#ifdef DM_DEBUG
#define D_SCALE (30.0f)
#define NUM_OBJECTS 512
//#define NUM_OBJECTS 1024
#else
#define D_SCALE (75.0f)
#define NUM_OBJECTS 1024
//#define NUM_OBJECTS 4096
#endif

typedef struct stress_test_data_t
{
    dm_entity objects[NUM_OBJECTS];
    bool debug_draw;
    view_camera camera;
} stress_test_data;

static stress_test_data stress_data = { 0 };

dm_entity stress_test_make_object()
{
    dm_entity entity = dm_ecs_create_entity();
    
    float hd = D_SCALE * 0.5f;
    float r = 2.0f;
    float hr = r * 0.5f;
    float hv = V_SCALE * 0.5f;
    
    dm_vec3 position = dm_vec3_set(dm_random_float() * D_SCALE - hd, dm_random_float() * D_SCALE - hd, dm_random_float() * D_SCALE - hd);
    dm_quat rot = dm_quat_set(dm_random_float() * r - hr, dm_random_float() * r - hr, dm_random_float() * r - hr, dm_random_float() * r - hr);
    rot = dm_quat_norm(rot);
    dm_vec3 scale;
    
    // mesh
    float rand_mesh = dm_random_float();
    dm_render_handle mesh_handle = rand_mesh > 0.05f ? 2 : 0;
    dm_ecs_entity_add_mesh(entity, mesh_handle);
    
    // collision
    switch(mesh_handle)
    {
        // box
        case 0:
        {
            scale = dm_vec3_set(dm_random_float(), dm_random_float(), dm_random_float());
            dm_ecs_entity_add_transform_v(entity, position, scale, rot);
            dm_ecs_entity_add_collision(entity, DM_COLLISION_SHAPE_BOX);
        } break;
        
        // icosphere
        case 2:
        {
            float s = dm_random_float();
            scale = (dm_vec3) { s,s,s };
            dm_ecs_entity_add_transform_v(entity, position, scale, rot);
            dm_ecs_entity_add_collision(entity, DM_COLLISION_SHAPE_SPHERE);
        } break;
    }
    
    // physics
    float mass = M_SCALE * (scale.x + scale.y + scale.z) / 3.0f;
    //float l = M_SCALE * 1e-2;
    //float hl = l * 0.5f;
    
    dm_vec3 velocity = dm_vec3_set(dm_random_float() * V_SCALE - hv, dm_random_float() * V_SCALE - hv, dm_random_float() * V_SCALE - hv);
    //dm_vec3 ang_mtm  = dm_vec3_set(dm_random_float() * l - hl, dm_random_float() * l - hl, dm_random_float() * l - hl);
    
    //dm_ecs_entity_add_physics(entity, velocity, ang_mtm, dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    dm_ecs_entity_add_physics(entity, velocity, dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), dm_vec3_set(0,0,0), mass, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC);
    
    // material
    float m_r = mass / M_SCALE;
    float m_b = 1.0f - m_r;
    dm_vec4 color = dm_vec4_set(m_r, 0, m_b, 1);
    
    dm_ecs_entity_add_material(entity, color, color);
    
    return entity;
}

void stress_test_debug_draw()
{
    static bool draw_aabb = false;
    static bool draw_physics = false;
    
    if(dm_input_key_just_pressed(DM_KEY_1)) draw_aabb = !draw_aabb;
    if(dm_input_key_just_pressed(DM_KEY_2)) draw_physics = !draw_physics;
    
    for(uint32_t i=0; i<NUM_OBJECTS; i++)
    {
        dm_entity entity = stress_data.objects[i];
        
        bool* is_colliding = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_IS_COLLIDING);
        bool* possibly_colliding = dm_ecs_get_component_member(DM_COMPONENT_COLLISION, DM_COLLISION_MEM_POSSIBLY_COLLIDING);
        
        dm_debug_render_transform(entity);
        
        if(draw_aabb)
        {
            dm_aabb aabb = dm_ecs_entity_get_aabb(entity);
            dm_debug_render_billboard_v(aabb.min, BILBOARD_SIZE, BILBOARD_SIZE, dm_vec4_set(0,0.25f,0.75f,1));
            dm_debug_render_billboard_v(aabb.max, BILBOARD_SIZE, BILBOARD_SIZE, dm_vec4_set(0.75f,0.25f,0,1));
            
            if(is_colliding[entity]) dm_debug_render_aabb(entity, 1,0,0,1);
            else if(possibly_colliding[entity]) dm_debug_render_aabb(entity, 1,1,0,1);
            else dm_debug_render_aabb(entity, 1,1,1,1);
        }
        
        if(draw_physics) 
        {
            //dm_debug_render_force_vector(entity);
            dm_debug_render_velocity_vector(entity);
            dm_debug_render_angular_velocity_vector(entity);
        }
    }
    
    // world origin
    dm_debug_render_line(0,0,0,1,0,0,1,1,0,0,1);
    dm_debug_render_line(0,0,0,0,1,0,1,0,1,0,1);
    dm_debug_render_line(0,0,0,0,0,1,1,0,0,1,1);
}

return_code stress_test_init()
{
    // gravity system
    gravity_system_init();
    
    // init camera
    float d = 6;
    init_camera(dm_vec3_set(d,d,d), dm_vec3_set(-1,-1,-1), 0.1f, 100.0f, 45.0f, 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &stress_data.camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&stress_data.camera.view_proj);
    dm_debug_render_set_inv_view(&stress_data.camera.inv_view);
    
    // render passes
    float* positions = NULL;
    float* normals = NULL;
    float* tex_coords = NULL;
    uint32_t* indices = NULL;
    uint32_t num_vertices=0, num_indices=0;
    
    dm_render_handle meshes[3];
    uint32_t num_meshes=0;
    // box mesh
    dm_geometry_box(&positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // icosahedorn mesh
    dm_geometry_icosahedron(&positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // icosphere mesh
    dm_geometry_icosphere(4, &positions, &normals, &tex_coords, &indices, num_vertices, &num_vertices, &num_indices, &meshes[num_meshes++]);
    
    // submit data
    if(!default_pass_init(positions, normals, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), &stress_data.camera)) return INIT_FAIL;
    
    dm_free(positions);
    dm_free(normals);
    dm_free(tex_coords);
    dm_free(indices);
    
    for(uint32_t i=0; i<NUM_OBJECTS; i++)
    {
        stress_data.objects[i] = stress_test_make_object();
    }
    
    return SUCCESS;
}

return_code stress_test_update()
{
    uint32_t width = DM_SCREEN_WIDTH;
    uint32_t height = DM_SCREEN_HEIGHT;
    
    // update camera
    resize_camera(width, height, &stress_data.camera);
    update_camera(dm_get_delta_time(), &stress_data.camera);
    
    return SUCCESS;
}

return_code stress_test_render()
{
    if(dm_input_key_just_pressed(DM_KEY_TAB)) stress_data.debug_draw = !stress_data.debug_draw;
    
    // debug drawing
    if(stress_data.debug_draw) stress_test_debug_draw();
    
    return SUCCESS;
}
