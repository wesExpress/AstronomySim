#include "app.h"

//#define STRESS_TEST

#ifdef STRESS_TEST
#include "stress_test.h"
#else
#include "space_sim.h"
#endif

#define APP_FUNC_CHECK(FUNC) {\
return_code app_return = FUNC;\
if(app_return != SUCCESS) return app_return;\
}\

return_code app_run()
{
    ////////////////////////////////////
#ifdef STRESS_TEST
    APP_FUNC_CHECK(stress_test_init());
#else
    APP_FUNC_CHECK(space_sim_init());
#endif
    ////////////////////////////////////
    
    // main loop
    double render_time     = 0;
    uint32_t frame_counter = 0;
    uint32_t fps           = 0;
    bool debug_draw        = false;
    
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
        APP_FUNC_CHECK(stress_test_update());
#else
        APP_FUNC_CHECK(space_sim_update());
#endif
        //////////////////////////////////////
        
        if(dm_input_key_just_pressed(DM_KEY_P)) dm_physics_toggle_pause();
        
        // render
        dm_timer render_timer = { 0 };
        dm_timer_start(&render_timer);
        
        if(!dm_renderer_begin_frame()) return RENDER_FAIL;
        
        //////////////////////////////////////
#ifdef STRESS_TEST
        APP_FUNC_CHECK(stress_test_render());
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
        
        if(!dm_end_update()) return RENDER_FAIL;
        
        frame_counter++;
    }
    
    return SUCCESS;
}
