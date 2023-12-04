#ifndef PHYSICS_TEST_H
#define PHYSICS_TEST_H

#include "app.h"

void physics_test_init(application_data* app_data, dm_context* context);
void physics_test_update(application_data* app_data, dm_context* context);
bool physics_test_render(application_data* app_data, dm_context* context);

#endif //PHYSICS_TEST_H
