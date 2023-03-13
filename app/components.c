#include "components.h"
#include "../DarkMatter/dm.h"

extern void default_pass_add_point_light(dm_entity entity);
extern void default_pass_add_blackbody(dm_entity entity);

typedef struct component_manager_t
{
    dm_ecs_id COMPONENT_LIGHT;
    dm_ecs_id COMPONENT_BLACKBODY;
} component_manager;

static component_manager manager = { 0 };

// generic lighters
void register_light_component()
{
    size_t light_sizes[] = {
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float), sizeof(float),
        sizeof(light_type),
    };
    
    DM_ECS_REGISTER_COMPONENT(component_light_caster, light_sizes, manager.COMPONENT_LIGHT);
}

void add_direction_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .direction_x=direction.x, .direction_y=direction.y, .direction_z=direction.z,
        .type=LIGHT_TYPE_DIRECTIONAL
    };
    
    DM_ADD_COMPONENT(entity, manager.COMPONENT_LIGHT, &l);
}

void add_point_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, float constant, float linear, float quadratic)
{
    component_light_caster l = {
        .ambient_r=ambient.x, .ambient_g=ambient.y, .ambient_b=ambient.z,
        .diffuse_r=diffuse.x, .diffuse_g=diffuse.y, .diffuse_b=diffuse.z,
        .specular_r=specular.x, .specular_g=specular.y, .specular_b=specular.z,
        .pos_x=pos.x, .pos_y=pos.y, .pos_z=pos.z,
        .constant=constant, .linear=linear, .quadratic=quadratic,
        .type=LIGHT_TYPE_POINT
    };
    
    DM_ADD_COMPONENT(entity, manager.COMPONENT_LIGHT, &l);
    
    default_pass_add_point_light(entity);
}

void add_spotlight_light_component(dm_entity entity, dm_vec4 ambient, dm_vec4 diffuse, dm_vec4 specular, dm_vec3 pos, dm_vec3 direction, float cutoff)
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
    
    DM_ADD_COMPONENT(entity, manager.COMPONENT_LIGHT, &l);
}

dm_ecs_id get_light_id()
{
    return manager.COMPONENT_LIGHT;
}

void* get_light_member(component_light_member member)
{
    return DM_GET_COMPONENT_MEMBER(manager.COMPONENT_LIGHT, member);
}

// blackbody
#define STEPHAN_BOLTZMAN 5.67e-8f // W m^-2 K^-4

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

float compute_blackbody_luminosity(float temperature, float radius)
{
    return DM_MATH_4PI * radius * radius * STEPHAN_BOLTZMAN * temperature * temperature * temperature * temperature;
}

float compute_blackbody_brightness(float luminosity, float distance_sq)
{
    return luminosity * DM_MATH_INV_4PI / distance_sq;
}

bool update_blackbodies(dm_entity* entities, uint32_t entity_count)
{
    float* pos_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    float* scale_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_X);
    
    float* temperature = get_blackbody_member(BLACKBODY_MEM_TEMPERATURE);
    float* luminosity  = get_blackbody_member(BLACKBODY_MEM_LUMINOSITY);
    float* brightness  = get_blackbody_member(BLACKBODY_MEM_BRIGHTNESS);
    float* color_r     = get_blackbody_member(BLACKBODY_MEM_COLOR_R);
    float* color_g     = get_blackbody_member(BLACKBODY_MEM_COLOR_G);
    float* color_b     = get_blackbody_member(BLACKBODY_MEM_COLOR_B);
    float* color_a     = get_blackbody_member(BLACKBODY_MEM_COLOR_A);
    
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        float d_sq = dm_vec3_len2(dm_vec3_set(pos_x[entity], pos_y[entity], pos_z[entity]));
        dm_vec4 color = compute_blackbody_color(temperature[entity]);
        
        color_r[entity] = color.x;
        color_g[entity] = color.y;
        color_b[entity] = color.z;
        color_a[entity] = color.w;
        
        float l = compute_blackbody_luminosity(temperature[entity], scale_x[entity]);
        luminosity[entity] = l;
        
        float b = compute_blackbody_brightness(luminosity[entity], d_sq);
        brightness[entity] = b;
    }
    
    return true;
}

void register_blackbody_component()
{
    size_t blackbody_sizes[] = {
        sizeof(float), sizeof(float), sizeof(float),
        sizeof(float), sizeof(float), sizeof(float), sizeof(float)
    };
    
    DM_ECS_REGISTER_COMPONENT(component_blackbody, blackbody_sizes, manager.COMPONENT_BLACKBODY);
}

void add_blackbody_component(dm_entity entity, float temperature)
{
    float* scale_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_X);
    float* scale_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Y);
    float* scale_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_SCALE_Z);
    
    float radius = DM_MAX(scale_x[entity], DM_MAX(scale_y[entity], scale_z[entity]));
    
    float luminosity = compute_blackbody_luminosity(temperature, radius);
    dm_vec4 color = compute_blackbody_color(temperature);
    
    component_blackbody b = {
        .temperature=temperature,
        .luminosity=luminosity,
        .color_r=color.x, .color_g=color.y, .color_b=color.z, .color_a=color.w
    };
    
    DM_ADD_COMPONENT(entity, manager.COMPONENT_BLACKBODY, &b);
    
    // change material diffuse color
    dm_vec4* diffuses = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_MATERIAL, DM_MATERIAL_MEM_DIFFUSE);
    diffuses[entity] = color;
    
    default_pass_add_blackbody(entity);
}

void* get_blackbody_member(component_blackbody_member member)
{
    return DM_GET_COMPONENT_MEMBER(manager.COMPONENT_BLACKBODY, member);
}

dm_ecs_id get_blackbody_id()
{
    return manager.COMPONENT_BLACKBODY;
}