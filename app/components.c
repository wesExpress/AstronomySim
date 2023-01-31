#include "components.h"
#include "../DarkMatter/dm.h"

static dm_ecs_id COMPONENT_LIGHT;

dm_ecs_id register_light_component()
{
    size_t light_sizes[] = {
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float), sizeof(float),
        sizeof(light_type),
    };
    
    DM_ECS_REGISTER_COMPONENT(component_light_caster, light_sizes, COMPONENT_LIGHT);
    
    return COMPONENT_LIGHT;
}

void add_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec4 wild_card, light_type type)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .type=type
    };
    
    switch(type)
    {
        case LIGHT_TYPE_DIRECTIONAL:
        {
            l.direction_x = wild_card.x;
            l.direction_y = wild_card.y;
            l.direction_z = wild_card.z;
        } break;
        
        case LIGHT_TYPE_SPOTLIGHT:
        {
            l.direction_x = wild_card.x;
            l.direction_y = wild_card.y;
            l.direction_z = wild_card.z;
            l.cutoff = wild_card.w;
        } break;
        
        case LIGHT_TYPE_POINT:
        {
            l.constant = wild_card.x;
            l.linear = wild_card.y;
            l.linear = wild_card.z;
        } break;
    }
    
    dm_ecs_entity_add_component(entity, COMPONENT_LIGHT, &l);
}