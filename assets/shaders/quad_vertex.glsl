#version 460 core

layout(location=0) in vec2 position;
layout(location=1) in vec2 tex_coords;

struct ps_input
{
    vec4 position;
    vec2 tex_coords;
};

out ps_input vs_output;

void main()
{
    vs_output.position = vec4(position, 0, 1);
    vs_output.tex_coords = tex_coords;

    gl_Position = vs_output.position;
}
