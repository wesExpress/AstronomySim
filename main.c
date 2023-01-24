#define DM_IMPL
//#define DM_PHYSICS_MORE_MASS
#include "DarkMatter/dm.h"

#include "app/app.h"

#define DEFAULT_SCREEN_WIDTH  1980
#define DEFAULT_SCREEN_HEIGHT 1080

/******************
SIMPLE ENTRY POINT
********************/
int main(int argc, char** argv)
{
    uint32_t return_code;
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