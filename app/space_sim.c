#include "space_sim.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"

typedef struct space_sim_data_t
{
} space_sim_data;

space_sim_data app_data = { 0 };

return_code space_sim_init()
{
    // gravity system
    gravity_system_init();
    
    // init camera
    float d = 6;
    init_camera(dm_vec3_set(d,d,d), dm_vec3_set(-1,-1,-1), 0.1f, 100.0f, 45.0f, 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &app_data.camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&app_data.camera.view_proj);
    dm_debug_render_set_inv_view(&app_data.camera.inv_view);
    
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