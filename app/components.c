#include "components.h"
#include "../DarkMatter/dm.h"

void register_light_component(dm_ecs_id* id)
{
    size_t light_sizes[] = {
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float), sizeof(float),
        sizeof(light_type),
    };
    
    DM_ECS_REGISTER_COMPONENT(component_light_caster, light_sizes, *id);
}

void add_direction_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, dm_ecs_id id)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .direction_x=direction.x, .direction_y=direction.y, .direction_z=direction.z,
        .type=LIGHT_TYPE_DIRECTIONAL
    };
    
    dm_ecs_entity_add_component(entity, id, &l);
}

void add_point_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, float constant, float linear, float quadratic, dm_ecs_id id)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .constant=constant, .linear=linear, .quadratic=quadratic,
        .type=LIGHT_TYPE_POINT
    };
    
    dm_ecs_entity_add_component(entity, id, &l);
}

void add_spotlight_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, float cutoff, dm_ecs_id id)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .direction_x=direction.x, .direction_y=direction.y, .direction_z=direction.z,
        .cutoff=cutoff,
        .type=LIGHT_TYPE_SPOTLIGHT
    };
    
    dm_ecs_entity_add_component(entity, id, &l);
}
