#ifndef __COMPONENTS_H__
#define __COMPONENTS_H__

#include "../DarkMatter/dm.h"

// generic lighting
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
    LIGHT_MEM_AMBIENT_R, LIGHT_MEM_AMBIENT_G, LIGHT_MEM_AMBIENT_B,
    LIGHT_MEM_DIFFUSE_R, LIGHT_MEM_DIFFUSE_G, LIGHT_MEM_DIFFUSE_B,
    LIGHT_MEM_SPECULAR_R, LIGHT_MEM_SPECULAR_G, LIGHT_MEM_SPECULAR_B,
    LIGHT_MEM_POS_X, LIGHT_MEM_POS_Y, LIGHT_MEM_POS_Z,
    LIGHT_MEM_UNION_0, LIGHT_MEM_UNION_1, LIGHT_MEM_UNION_2, LIGHT_MEM_UNION_3,
    LIGHT_MEM_TYPE,
    LIGHT_MEM_UNKNOWN
} component_light_member;

void add_direction_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction);
void add_point_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, float constant, float linear, float quadratic);
void add_spotlight_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, float cutoff);
dm_ecs_id get_light_id();
void* get_light_member(component_light_member member);

// special blackbody lighting
typedef struct component_blackbody_t
{
    float temperature, luminosity, brightness;
    float color_r, color_g, color_b, color_a;
} component_blackbody;

typedef enum component_blackbody_member_t
{
    BLACKBODY_MEM_TEMPERATURE, BLACKBODY_MEM_LUMINOSITY, BLACKBODY_MEM_BRIGHTNESS,
    BLACKBODY_MEM_COLOR_R, BLACKBODY_MEM_COLOR_G, BLACKBODY_MEM_COLOR_B, BLACKBODY_MEM_COLOR_A,
    BLACKBODY_MEM_UNKNOWN
} component_blackbody_member;

void add_blackbody_component(dm_entity entity, float temperature);
dm_ecs_id get_blackbody_id();
void* get_blackbody_member(component_blackbody_member member);
bool update_blackbodies(dm_entity* entities, uint32_t entity_count);

#endif //COMPONENTS_H
