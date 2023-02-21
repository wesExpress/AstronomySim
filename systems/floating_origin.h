#ifndef __FLOATING_ORIGIN_H__
#define __FLOATING_ORIGIN_H__

typedef struct floating_origin_data_t
{
    dm_entity ref_entity;
    bool has_ref;
} floating_origin_data;

static floating_origin_data origin_data = { 0 };

bool floating_origin_func(dm_entity* entities, uint32_t entity_count)
{
    if(!origin_data.has_ref) { DM_LOG_FATAL("Floating origin system needs a reference entity"); return false; }
    
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    float ref_x = pos_x[origin_data.ref_entity];
    float ref_y = pos_y[origin_data.ref_entity];
    float ref_z = pos_z[origin_data.ref_entity];
    
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        pos_x[entity] -= ref_x;
        pos_y[entity] -= ref_y;
        pos_z[entity] -= ref_z;
    }
    
    return true;
}

dm_ecs_id floating_origin_system_init(dm_entity ref_entity)
{
    dm_ecs_id component_ids[] = { DM_COMPONENT_TRANSFORM };
    dm_ecs_id system;
    DM_ECS_REGISTER_SYSTEM(DM_ECS_SYSTEM_TIMING_BEGIN, component_ids, floating_origin_func, system);
    
    origin_data.ref_entity = ref_entity;
    origin_data.has_ref = true;
    
    return system;
}

#endif //FLOATING_ORIGIN_H
