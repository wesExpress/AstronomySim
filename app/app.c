#include "app.h"
#include "camera.h"
#include "../systems/default_pass.h"
#include "../systems/gravity.h"
#include "../systems/light_src_pass.h"

// wrappers
#define BOX_MESH 0
#define ICOSAHEDRON_MESH BOX_MESH + 1
#define ICOSPHERE_MESH ICOSAHEDRON_MESH + 1

dm_ecs_id COMPONENT_LIGHT;

typedef struct application_data_t
{
    view_camera camera;
} application_data;

static application_data app_data = { 0 };

//#define STRESS_TEST
#ifndef STRESS_TEST
//#define PHYSICS_TEST
#endif

#ifdef STRESS_TEST
#include "stress_test.h"
#elif defined(PHYSICS_TEST)
#include "physics_test.h"
#else
#include "space_sim.h"
#endif

#define APP_FUNC_CHECK(FUNC) {\
return_code app_return = FUNC;\
if(app_return != SUCCESS) return app_return;\
}\

return_code app_run()
{
    // init camera
    float d = 6.0f;
    init_camera(dm_vec3_set(d,d,d), dm_vec3_set(-1,-1,-1), 0.01f, 1e13f, dm_deg_to_rad(75.0f), 0.3f, 5.0f, DM_SCREEN_WIDTH, DM_SCREEN_HEIGHT, &app_data.camera);
    
    // link camera view_proj to debug draw pass
    dm_debug_render_set_view_proj(&app_data.camera.view_proj);
    dm_debug_render_set_inv_view(&app_data.camera.inv_view);
    dm_debug_render_set_far_plane(&app_data.camera.far_plane);
    
    // components
    register_light_component(&COMPONENT_LIGHT);
    
    // mesh data
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
    if(!default_pass_init(positions, normals, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), COMPONENT_LIGHT, &app_data.camera)) return INIT_FAIL;
    if(!light_src_pass_init(positions, tex_coords, num_vertices, indices, num_indices, meshes, DM_ARRAY_LEN(meshes), COMPONENT_LIGHT, &app_data.camera)) return INIT_FAIL;
    
    dm_free(positions);
    dm_free(normals);
    dm_free(tex_coords);
    dm_free(indices);
    
    ////////////////////////////////////
#ifdef STRESS_TEST
    APP_FUNC_CHECK(stress_test_init());
#elif defined(PHYSICS_TEST)
    APP_FUNC_CHECK(physics_test_init());
#else
    APP_FUNC_CHECK(space_sim_init());
#endif
    ////////////////////////////////////
    
    // main loop
    double render_time     = 0;
    uint32_t frame_counter = 0;
    uint32_t fps           = 0;
    
    dm_timer frame_timer = { 0 };
    dm_timer_start(&frame_timer);
    
    while(true)
    {
        if(dm_timer_elapsed(&frame_timer) > 1)
        {
            fps = frame_counter;
            frame_counter = 0;
            dm_timer_start(&frame_timer);
        }
        
        // update
        if(!dm_begin_update()) break;
        
        //////////////////////////////////////
#ifdef STRESS_TEST
        APP_FUNC_CHECK(stress_test_update(&app_data.camera));
#elif defined(PHYSICS_TEST)
        APP_FUNC_CHECK(physics_test_update(&app_data.camera));
#else
        APP_FUNC_CHECK(space_sim_update(&app_data.camera));
#endif
        //////////////////////////////////////
        
        if(dm_input_key_just_pressed(DM_KEY_P)) dm_physics_toggle_pause();
        
        // resize camera
        uint32_t width = DM_SCREEN_WIDTH;
        uint32_t height = DM_SCREEN_HEIGHT;
        
        resize_camera(width, height, &app_data.camera);
        
        // render
        dm_timer render_timer = { 0 };
        dm_timer_start(&render_timer);
        
        if(!dm_renderer_begin_frame()) return RENDER_FAIL;
        
        //////////////////////////////////////
#ifdef STRESS_TEST
        APP_FUNC_CHECK(stress_test_render());
#elif defined(PHYSICS_TEST)
        APP_FUNC_CHECK(physics_test_render());
#else
        APP_FUNC_CHECK(space_sim_render());
#endif
        //////////////////////////////////////
        
        render_time = dm_timer_elapsed_ms(&render_timer);
        
        // fps
        dm_imgui_text_fmt(10, 25, 1, 1, 1, 1, "FPS: %u", fps);
        // frame render time display
        dm_imgui_text_fmt(10, 50, 1, 1, 1, 1, "Render took: %0.2lf ms", render_time);
        
        // wrap up frame
        if(!dm_renderer_end_frame()) return RENDER_FAIL;
        
        // DarkMatter end update
        if(!dm_end_update()) return RENDER_FAIL;
        
        frame_counter++;
    }
    
    return SUCCESS;
}
