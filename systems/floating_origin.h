#ifndef __FLOATING_ORIGIN_H__
#define __FLOATING_ORIGIN_H__

typedef struct floating_origin_data_t
{
    dm_entity pos_ref, rot_ref;
    bool has_pos_ref, has_rot_ref;
} floating_origin_data;

static floating_origin_data origin_data = { 0 };

bool floating_origin_func(dm_entity* entities, uint32_t entity_count)
{
    if(dm_physics_is_paused()) return true;
    if(!origin_data.has_pos_ref) { DM_LOG_FATAL("Floating origin system needs a reference entity"); return false; }
    
    float* pos_x = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    const float ref_x = pos_x[origin_data.pos_ref];
    const float ref_y = pos_y[origin_data.pos_ref];
    const float ref_z = pos_z[origin_data.pos_ref];
    
    // more involved 'rotation' floating origin
    if(origin_data.has_rot_ref)
    {
        float* rot_i = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
        float* rot_j = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
        float* rot_k = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
        float* rot_r = dm_ecs_get_component_member(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
        
        float* vel_x = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_X);
        float* vel_y = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Y);
        float* vel_z = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Z);
        float* w_x   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_X);
        float* w_y   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_Y);
        float* w_z   = dm_ecs_get_component_member(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_Z);
        
        const float rot_ref_x = pos_x[origin_data.rot_ref];
        const float rot_ref_y = pos_y[origin_data.rot_ref];
        const float rot_ref_z = pos_z[origin_data.rot_ref];
        
        dm_vec3 w = dm_vec3_set(w_x[origin_data.rot_ref], w_y[origin_data.rot_ref], w_z[origin_data.rot_ref]);
        w = dm_vec3_negate(w);
        
        dm_vec3 axis = dm_vec3_norm(w);
        float theta = dm_vec3_len(w) * dm_get_delta_time();
        
        dm_mat4 test = dm_mat_rotation_make(theta, axis);
        
        for(uint32_t i=0; i<entity_count; i++)
        {
            dm_entity entity = entities[i];
            
            if(entity == origin_data.pos_ref || entity == origin_data.rot_ref) continue;
            
            dm_vec3 p = dm_vec3_set(pos_x[entity] - rot_ref_x, pos_y[entity] - rot_ref_y, pos_z[entity] - rot_ref_z);
            //p = dm_mat3_mul_vec3(r, p);
            
            dm_vec4 p2 = dm_vec4_set_from_vec3(p);
            p2.w = 1.0f;
            p2 = dm_mat4_mul_vec4(test, p2);
            
            pos_x[entity] = p2.x + rot_ref_x - ref_x;
            pos_y[entity] = p2.y + rot_ref_y - ref_y;
            pos_z[entity] = p2.z + rot_ref_z - ref_z;
        }
        
        // rotation reference entity
        dm_quat rot = dm_quat_set(rot_i[origin_data.rot_ref], rot_j[origin_data.rot_ref], rot_k[origin_data.rot_ref], rot_r[origin_data.rot_ref]);
        dm_quat delta_rot = dm_vec3_mul_quat(w, rot);
        delta_rot = dm_quat_scale(delta_rot, 0.5f * dm_get_delta_time());
        dm_quat new_rot = dm_quat_add_quat(rot, delta_rot);
        new_rot = dm_quat_norm(new_rot);
        
        rot_i[origin_data.rot_ref] = new_rot.i;
        rot_j[origin_data.rot_ref] = new_rot.j;
        rot_k[origin_data.rot_ref] = new_rot.k;
        rot_r[origin_data.rot_ref] = new_rot.r;
        
        pos_x[origin_data.rot_ref] -= ref_x;
        pos_y[origin_data.rot_ref] -= ref_y;
        pos_z[origin_data.rot_ref] -= ref_z;
        
        // reference entity
        pos_x[origin_data.pos_ref] = 0.0f;
        pos_y[origin_data.pos_ref] = 0.0f;
        pos_z[origin_data.pos_ref] = 0.0f;
        
        return true;
    }
    
    // simple floating origin
    for(uint32_t i=0; i<entity_count; i++)
    {
        dm_entity entity = entities[i];
        
        if(entity == origin_data.pos_ref) continue;
        
        pos_x[entity] -= ref_x;
        pos_y[entity] -= ref_y;
        pos_z[entity] -= ref_z;
    }
    
    // reference entity
    pos_x[origin_data.pos_ref] = 0.0f;
    pos_y[origin_data.pos_ref] = 0.0f;
    pos_z[origin_data.pos_ref] = 0.0f;
    
    return true;
}

dm_ecs_id floating_origin_system_init(dm_entity ref_entity)
{
    dm_ecs_id component_ids[] = { DM_COMPONENT_TRANSFORM };
    dm_ecs_id system;
    DM_ECS_REGISTER_SYSTEM(DM_ECS_SYSTEM_TIMING_BEGIN, component_ids, floating_origin_func, system);
    
    origin_data.pos_ref = ref_entity;
    origin_data.has_pos_ref = true;
    
    return system;
}

void floating_origin_enable_rot(dm_entity entity)
{
    if(!dm_ecs_entity_has_component(entity, DM_COMPONENT_PHYSICS)) { DM_LOG_ERROR("Floating rotation only works with entities that have a physics component"); return; }
    
    origin_data.rot_ref = entity;
    origin_data.has_rot_ref = true;
}

void floating_origin_disable_rot()
{
    origin_data.has_rot_ref = false;
}

#endif //FLOATING_ORIGIN_H
