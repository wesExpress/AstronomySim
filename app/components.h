#ifndef __COMPONENTS_H__
#define __COMPONENTS_H__

#include "../DarkMatter/dm.h"

typedef enum light_type
{
    LIGHT_TYPE_DIRECTIONAL,
    LIGHT_TYPE_POINT,
    LIGHT_TYPE_SPOTLIGHT,
    LIGHT_TYPE_UNKNOWN
} light_type;

typedef struct component_light_caster_t
{
    float ambient_r, ambient_g, ambient_b;
    float diffuse_r, diffuse_g, diffuse_b;
    float specular_r, specular_g, specular_b;
    float pos_x, pos_y, pos_z;
    
    union
    {
        float direction_x;
        float constant;
    };
    union
    {
        float direction_y;
        float linear;
    };
    union
    {
        float direction_z;
        float quadratic;
    };
    union
    {
        float cutoff;
    };
    
    light_type type;
} component_light_caster;

typedef enum component_light_member_t
{
    LIGHT_MEM_AMBIENT_X, LIGHT_MEM_AMBIENT_Y, LIGHT_MEM_AMBIENT_Z,
    LIGHT_MEM_DIFFUSE_R, LIGHT_MEM_DIFFUSE_G, LIGHT_MEM_DIFFUSE_B,
    LIGHT_MEM_SPECULAR_R, LIGHT_MEM_SPECULAR_G, LIGHT_MEM_SPECULAR_B,
    LIGHT_MEM_POS_X, LIGHT_MEM_POS_Y, LIGHT_MEM_POS_Z,
    LIGHT_MEM_UNION_0, LIGHT_MEM_UNION_1, LIGHT_MEM_UNION_2, LIGHT_MEM_UNION_3,
    LIGHT_MEM_TYPE,
    LIGHT_MEM_UNKNOWN
} component_light_member;

void register_light_component(dm_ecs_id* id);
void add_direction_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, dm_ecs_id id);
void add_point_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, float constant, float linear, float quadratic, dm_ecs_id id);
void add_spotlight_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, float cutoff, dm_ecs_id id);

#endif //COMPONENTS_H
