#ifndef IMGUI_RENDER_PASS_H
#define IMGUI_RENDER_PASS_H

#include "dm.h"

bool imgui_render_pass_init(dm_context* context);
void imgui_render_pass_shutdown(dm_context* context);
bool imgui_render_pass_render(dm_context* context);

void imgui_set_font(dm_render_handle font, dm_context* context);
void imgui_draw_text(uint32_t x, uint32_t y, float r, float g, float b, float a, const char* text, dm_context* context);
void imgui_draw_text_fmt(uint32_t x, uint32_t y, float r, float g, float b, float a, dm_context* context, const char* text, ...);

#endif //IMGUI_RENDER_PASS_H
