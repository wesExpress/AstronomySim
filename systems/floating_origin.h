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
    
    float* pos_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_X);
    float* pos_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Y);
    float* pos_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_POS_Z);
    
    const float ref_x = pos_x[origin_data.pos_ref];
    const float ref_y = pos_y[origin_data.pos_ref];
    const float ref_z = pos_z[origin_data.pos_ref];
    
    // more involved 'rotation' floating origin
    if(origin_data.has_rot_ref)
    {
        float* rot_i = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_I);
        float* rot_j = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_J);
        float* rot_k = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_K);
        float* rot_r = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_TRANSFORM, DM_TRANSFORM_MEM_ROT_R);
        
        float* vel_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_X);
        float* vel_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Y);
        float* vel_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_VEL_Z);
        float* w_x   = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_X);
        float* w_y   = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_Y);
        float* w_z   = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS,   DM_PHYSICS_MEM_W_Z);
        float* force_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_X);
        float* force_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Y);
        float* force_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_FORCE_Z);
        float* torque_x = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_TORQUE_X);
        float* torque_y = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_TORQUE_Y);
        float* torque_z = DM_GET_COMPONENT_MEMBER(DM_COMPONENT_PHYSICS, DM_PHYSICS_MEM_TORQUE_Z);
        
        const float rot_ref_x = pos_x[origin_data.rot_ref];
        const float rot_ref_y = pos_y[origin_data.rot_ref];
        const float rot_ref_z = pos_z[origin_data.rot_ref];
        
        dm_vec3 w = dm_vec3_set(w_x[origin_data.rot_ref], w_y[origin_data.rot_ref], w_z[origin_data.rot_ref]);
        w = dm_vec3_negate(w);
        dm_quat rot = dm_quat_set(rot_i[origin_data.rot_ref], rot_j[origin_data.rot_ref], rot_k[origin_data.rot_ref], rot_r[origin_data.rot_ref]);
        dm_quat delta_rot = dm_vec3_mul_quat(w, rot);
        delta_rot = dm_quat_scale(delta_rot, 0.5f * dm_physics_get_simulation_time_step());
        
        dm_vec3 axis = dm_vec3_norm(w);
        float   theta = dm_vec3_len(w) * dm_physics_get_simulation_time_step();
        dm_quat rotation = dm_quat_from_axis_angle(axis, theta);
        
        for(uint32_t i=0; i<entity_count; i++)
        {
            dm_entity entity = entities[i];
            
            if(entity == origin_data.pos_ref || entity == origin_data.rot_ref) continue;
            
            dm_vec3 p = dm_vec3_set(pos_x[entity] - rot_ref_x, pos_y[entity] - rot_ref_y, pos_z[entity] - rot_ref_z);
            p = dm_vec3_rotate(p, rotation);
            
            pos_x[entity] = p.x + rot_ref_x - ref_x;
            pos_y[entity] = p.y + rot_ref_y - ref_y;
            pos_z[entity] = p.z + rot_ref_z - ref_z;
            
            if(!dm_ecs_entity_has_component(entity, DM_COMPONENT_PHYSICS)) continue;
            
            dm_vec3 f = dm_vec3_set(force_x[entity], force_y[entity], force_z[entity]);
            dm_vec3 t = dm_vec3_set(torque_x[entity], torque_y[entity], torque_z[entity]);
            dm_vec3 v = dm_vec3_set(vel_x[entity], vel_y[entity], vel_z[entity]);
            dm_vec3 w = dm_vec3_set(w_x[entity], w_y[entity], w_z[entity]);
            
            f = dm_vec3_rotate(f, rotation);
            t = dm_vec3_rotate(t, rotation);
            v = dm_vec3_rotate(v, rotation);
            w = dm_vec3_rotate(w, rotation);
            
            force_x[entity] = f.x;
            force_y[entity] = f.y;
            force_z[entity] = f.z;
            
            torque_x[entity] = t.x;
            torque_y[entity] = t.y;
            torque_z[entity] = t.z;
            
            vel_x[entity] = v.x;
            vel_y[entity] = v.y;
            vel_z[entity] = v.z;
            
            w_x[entity] = w.x;
            w_y[entity] = w.y;
            w_z[entity] = w.z;
            
            dm_ecs_entity_update_world_aabb(entity);
        }
        
        // rotation reference entity
        dm_quat new_rot = dm_quat_add_quat(rot, delta_rot);
        new_rot = dm_quat_norm(new_rot);
        
        rot_i[origin_data.rot_ref] = new_rot.i;
        rot_j[origin_data.rot_ref] = new_rot.j;
        rot_k[origin_data.rot_ref] = new_rot.k;
        rot_r[origin_data.rot_ref] = new_rot.r;
        
        pos_x[origin_data.rot_ref] -= ref_x;
        pos_y[origin_data.rot_ref] -= ref_y;
        pos_z[origin_data.rot_ref] -= ref_z;
        
        dm_ecs_entity_update_world_aabb(origin_data.rot_ref);
        
        // reference entity
        pos_x[origin_data.pos_ref] = 0.0f;
        pos_y[origin_data.pos_ref] = 0.0f;
        pos_z[origin_data.pos_ref] = 0.0f;
        
        dm_ecs_entity_update_world_aabb(origin_data.pos_ref);
        
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
        
        dm_ecs_entity_update_world_aabb(entity);
    }
    
    // reference entity
    pos_x[origin_data.pos_ref] = 0.0f;
    pos_y[origin_data.pos_ref] = 0.0f;
    pos_z[origin_data.pos_ref] = 0.0f;
    
    dm_ecs_entity_update_world_aabb(origin_data.pos_ref);
    
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
