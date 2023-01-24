#ifndef __STRESS_TEST_H__
#define __STRESS_TEST_H__

#include "../DarkMatter/dm.h"
#include "app.h"

return_code stress_test_init();
return_code stress_test_update();
return_code stress_test_render();

dm_entity stress_test_make_object();
void stress_test_debug_draw();

#endif //STRESS_TEST_H
