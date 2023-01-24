#ifndef __APP_H__
#define __APP_H__

typedef enum return_code_t
{
    SUCCESS,
    INIT_FAIL,
    RESOURCE_CREATION_FAIL,
    UPDATE_FAIL,
    RENDER_FAIL,
    UNKNOWN_FAIL
} return_code;

return_code app_run();

#endif //APP_H
