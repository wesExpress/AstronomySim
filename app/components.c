#include "components.h"
#include "../systems/default_pass.h"
#include "../DarkMatter/dm.h"

// generic lighters
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
    
    default_pass_add_point_light(entity);
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

// blackbody
#define STEPHAN_BOLTZMAN 5.67e-8f // W m^-2 K^-4
void register_blackbody_component(dm_ecs_id* id)
{
    size_t blackbody_sizes[] = {
        sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float), sizeof(float)
    };
    
    DM_ECS_REGISTER_COMPONENT(component_blackbody, blackbody_sizes, *id);
}

void add_blackbody_component(dm_entity entity, float temperature, dm_ecs_id id)
{
    float* scale_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_X);
    float* scale_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Y);
    float* scale_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Z);
    
    float radius = DM_MAX(scale_x[entity], DM_MAX(scale_y[entity], scale_z[entity]));
    
    float luminosity = 4.0f * DM_MATH_PI * radius * radius * STEPHAN_BOLTZMAN * temperature * temperature * temperature * temperature;
    dm_vec4 color = compute_blackbody_color(temperature);
    
    component_blackbody b = {
        .temperature=temperature,
        .luminosity=luminosity,
        .color=color
    };
    
    dm_ecs_entity_add_component(entity, id, &b);
    
    // change material diffuse color
    dm_vec4* diffuses = dm_ecs_get_component_member(DM_COMPONENT_MATERIAL, DM_MATERIAL_MEM_DIFFUSE);
    diffuses[entity] = color;
    
    default_pass_add_blackbody(entity);
}

// https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
dm_vec4 compute_blackbody_color(float temperature)
{
    temperature /= 100.0f;
    float red = 0.0f, green = 0.0f, blue = 0.0f;
    
    if(temperature <= 66) 
    {
        red = 255.0f;
        
        green = temperature;
        green = 99.4708025861f * dm_logf(green) - 161.1195681661f;
    }
    else
    {
        red = temperature - 60.0f;
        green = temperature - 60.0f;
        
        red   = 329.698727446f * dm_powf(red, -0.1332047592f);
        green = 288.1221695283f * dm_powf(green, -0.0755148492f);
        
        blue = 255.0f;
    }
    
    if(temperature > 19)
    {
        blue = temperature - 10.0f;
        blue = 138.5177312231f * dm_logf(blue) - 305.0447927307f;
    }
    
    red   = DM_CLAMP(red, 0, 255);
    green = DM_CLAMP(green, 0, 255);
    blue  = DM_CLAMP(blue, 0, 255);
    
    dm_vec4 color = dm_vec4_set(red,green,blue,255);
    return dm_vec4_scale(color, 1.0f / 255.0f);
}