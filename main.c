#define DM_IMPL
#define DM_SLOT_ARRAY_BIG_INDEX
#include "../dm.h"

#include "app.h"

#define DEFAULT_SCREEN_WIDTH  1980
#define DEFAULT_SCREEN_HEIGHT 1080

#define BILBOARD_SIZE (0.05f)

/******************
SIMPLE ENTRY POINT
********************/
int main(int argc, char** argv)
{
    /////////////////////////////////////////////////////
    if(!dm_init(100,100,DEFAULT_SCREEN_WIDTH,DEFAULT_SCREEN_HEIGHT,"test",true)) return INIT_FAIL;
    /////////////////////////////////////////////////////
    
    uint32_t return_code = app_run();
    
    // shutdown dark matter
    dm_shutdown();
    
    // TODO: remove eventually, this just lets me see all errors before closing
    getchar();
    
    return return_code;
}