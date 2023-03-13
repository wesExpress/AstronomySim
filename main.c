#define DM_IMPL
#ifndef DM_DEBUG
#define DM_PHYSICS_SMALLER_DT
#endif
//#define DM_PHYSICS_NO_SLOP
#define DM_PHYSICS_NO_PERSISTENT_MANIFOLDS
#include "DarkMatter/dm.h"

#include "app/app.h"

#ifdef DM_PLATFORM_APPLE
#define DEFAULT_SCREEN_WIDTH  1280
#define DEFAULT_SCREEN_HEIGHT  720
#else
#define DEFAULT_SCREEN_WIDTH  1980
#define DEFAULT_SCREEN_HEIGHT 1080
#endif

/******************
SIMPLE ENTRY POINT
********************/
int main(int argc, char** argv)
{
    uint32_t return_code = SUCCESS;
    /////////////////////////////////////////////////////
    if(!dm_init(100,100,DEFAULT_SCREEN_WIDTH,DEFAULT_SCREEN_HEIGHT,"test",true)) return_code = INIT_FAIL;
    /////////////////////////////////////////////////////
    
    if(return_code == SUCCESS) return_code = app_run();
    
    // shutdown dark matter
    dm_shutdown();
    
    // TODO: remove eventually, this just lets me see all errors before closing
    getchar();
    
    return return_code;
}