#include "space_sim.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

typedef struct space_sim_data_t
{
    view_camera camera;
    dm_entity entities[2];
} space_sim_data;

space_sim_data app_data = { 0 };

return_code space_sim_init()
{
    const dm_vec4 white = dm_vec4_set(1,1,1,1);
    const float gray_scale = 0.5f;
    
    const float r_moon = 1.7374e6f;   // m
    const float m_moon = 7.3459e22f;  // kg
    const dm_vec4 c_moon = dm_vec4_set(white.x * gray_scale, white.y * gray_scale, white.z * gray_scale, 1);
    
    // gravity system
    gravity_system_init();
    
    // init camera
    init_camera(dm_vec3_set(r_moon * 4,0,0), dm_vec3_set(-1,0,0), 0.1f, 1e13f, 45.0f, 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &app_data.camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&app_data.camera.view_proj);
    dm_debug_render_set_inv_view(&app_data.camera.inv_view);
    dm_debug_render_set_far_plane(&app_data.camera.far_plane);
    
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
    if(!default_pass_init(positions, normals, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), &app_data.camera)) return INIT_FAIL;
    
    dm_free(positions);
    dm_free(normals);
    dm_free(tex_coords);
    dm_free(indices);
    
    // entities
    // big planet and box for flying around
    
    // planet (using moon data)
    dm_vec3 scale = { r_moon,r_moon,r_moon };
    dm_vec3 pos   = { 0 };
    dm_quat rot   = { 0,0,0,1 };
    
    app_data.entities[0] = dm_ecs_create_entity();
    dm_ecs_entity_add_transform_v(app_data.entities[0], pos, scale, rot);
    dm_ecs_entity_add_collision(app_data.entities[0], DM_COLLISION_SHAPE_SPHERE);
    dm_ecs_entity_add_physics_at_rest(app_data.entities[0], m_moon, DM_PHYSICS_BODY_TYPE_RIGID, DM_PHYSICS_MOVEMENT_KINEMATIC); 
    dm_ecs_entity_add_mesh(app_data.entities[0], 1);
    dm_ecs_entity_add_material(app_data.entities[0], c_moon, c_moon);
    
    return SUCCESS;
}

return_code space_sim_update()
{
    uint32_t width = DM_SCREEN_WIDTH;
    uint32_t height = DM_SCREEN_HEIGHT;
    
    // update camera
    resize_camera(width, height, &app_data.camera);
    update_camera(dm_get_delta_time(), &app_data.camera);
    
    return SUCCESS;
}

return_code space_sim_render()
{
    return SUCCESS;
}